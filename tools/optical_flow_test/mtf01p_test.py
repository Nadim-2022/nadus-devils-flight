#!/usr/bin/env python3

# MTF-01P bench test on the PC, through a USB to UART adapter
# reads Micolink the same way Mtf01p.cpp does and prints it live
#
# python3 mtf01p_test.py
# python3 mtf01p_test.py --port /dev/ttyUSB1 --baud 115200
# python3 mtf01p_test.py --raw    (hex dump, to see what the sensor sends)
# python3 mtf01p_test.py --test height / covered / floor / direction

import argparse
import struct
import sys
import time

import serial

import bench_tests


MICOLINK_HEADER = 0xEF
MESSAGE_RANGE_SENSOR = 0x51
MAX_PAYLOAD_LENGTH = 64
RANGE_SENSOR_PAYLOAD_LENGTH = 20

# same as Mtf01p.hpp, status values not in the MicoAir docs
DISTANCE_STATUS_OK = 1
FLOW_STATUS_OK = 1
MIN_DISTANCE_MM = 10


class Mtf01pReader:

    def __init__(self):
        self.state = "wait_for_header"
        self.frame = bytearray()
        self.payload_length = 0
        self.checksum = 0
        self.good_frames = 0
        self.bad_frames = 0

    # one byte in, a reading out when a good range frame is done
    def add_byte(self, rx_byte):
        if self.state == "wait_for_header":
            if rx_byte == MICOLINK_HEADER:
                self.frame = bytearray([rx_byte])
                self.checksum = rx_byte
                self.state = "read_header"
            return None

        if self.state == "read_header":
            # device id, system id, message id, sequence, length
            self.frame.append(rx_byte)
            self.checksum = (self.checksum + rx_byte) & 0xFF
            if len(self.frame) == 6:
                self.payload_length = rx_byte
                if self.payload_length > MAX_PAYLOAD_LENGTH:
                    self.bad_frames += 1
                    self.state = "wait_for_header"
                elif self.payload_length == 0:
                    self.state = "read_checksum"
                else:
                    self.state = "read_payload"
            return None

        if self.state == "read_payload":
            self.frame.append(rx_byte)
            self.checksum = (self.checksum + rx_byte) & 0xFF
            if len(self.frame) == 6 + self.payload_length:
                self.state = "read_checksum"
            return None

        # checksum = sum of all bytes before it, 8 bit
        self.state = "wait_for_header"
        if rx_byte != self.checksum:
            self.bad_frames += 1
            return None

        message_id = self.frame[3]
        if message_id != MESSAGE_RANGE_SENSOR or self.payload_length < RANGE_SENSOR_PAYLOAD_LENGTH:
            return None

        self.good_frames += 1
        return decode_range_sensor(bytes(self.frame[6:6 + RANGE_SENSOR_PAYLOAD_LENGTH]))


# time_ms u32, distance u32 (mm), strength u8, precision u8, tof_status u8, reserved u8,
# flow_vel_x i16, flow_vel_y i16 (cm/s at 1 m), flow_quality u8, flow_status u8, reserved u16
def decode_range_sensor(payload):
    (sensor_time_ms, distance_mm, strength, precision, distance_status, reserved,
     flow_x_raw, flow_y_raw, flow_quality, flow_status, reserved_two) = struct.unpack("<IIBBBBhhBBH", payload)

    reading = {}
    reading["sensor_time_ms"] = sensor_time_ms
    reading["distance_mm"] = distance_mm
    reading["distance_m"] = distance_mm * 0.001
    reading["strength"] = strength
    reading["precision"] = precision
    reading["distance_status"] = distance_status
    reading["distance_ok"] = distance_mm >= MIN_DISTANCE_MM and distance_status == DISTANCE_STATUS_OK

    # cm/s at 1 m -> m/s at 1 m
    reading["flow_x"] = flow_x_raw * 0.01
    reading["flow_y"] = flow_y_raw * 0.01
    reading["flow_quality"] = flow_quality
    reading["flow_status"] = flow_status
    reading["flow_ok"] = flow_status == FLOW_STATUS_OK

    # flow is per metre, times the distance = ground speed
    reading["speed_x"] = reading["flow_x"] * reading["distance_m"]
    reading["speed_y"] = reading["flow_y"] * reading["distance_m"]
    return reading


def print_reading(reading, reader, frames_per_second):
    print("dist %6d mm ok %d status %3u strength %3u precision %3u | "
          "flow x %+6.2f y %+6.2f ok %d status %3u quality %3u | "
          "speed x %+5.2f y %+5.2f m/s | %3.0f Hz bad %lu"
          % (reading["distance_mm"], reading["distance_ok"], reading["distance_status"],
             reading["strength"], reading["precision"],
             reading["flow_x"], reading["flow_y"], reading["flow_ok"], reading["flow_status"], reading["flow_quality"],
             reading["speed_x"], reading["speed_y"],
             frames_per_second, reader.bad_frames))


# guess the protocol from the start bytes seen
def guess_protocol(data):
    guesses = []
    if data.count(b"\xef") > 0:
        guesses.append("0xEF (Micolink start) x%d" % data.count(b"\xef"))
    if data.count(b"\xfd") > 0:
        guesses.append("0xFD (MAVLink 2 start) x%d" % data.count(b"\xfd"))
    if data.count(b"\xfe") > 0:
        guesses.append("0xFE (MAVLink 1 start) x%d" % data.count(b"\xfe"))
    if data.count(b"$X") + data.count(b"$M") > 0:
        guesses.append("$X / $M (MSP, INAV) x%d" % (data.count(b"$X") + data.count(b"$M")))
    if len(guesses) == 0:
        return "no known start byte, maybe wrong baud"
    return ", ".join(guesses)


def raw_dump(port):
    # 16 bytes a line, with a protocol guess every second
    seen = bytearray()
    line = bytearray()
    last_guess = time.time()
    last_data = time.time()
    try:
        while True:
            data = port.read(256)
            now = time.time()
            if data:
                last_data = now
            for rx_byte in data:
                line.append(rx_byte)
                seen.append(rx_byte)
                if len(line) == 16:
                    print("%.2f  %s" % (now, " ".join("%02X" % value for value in line)))
                    line = bytearray()
            if now - last_guess >= 1.0:
                last_guess = now
                if now - last_data > 1.0:
                    print("--- no bytes for %.0f s" % (now - last_data))
                else:
                    print("--- %d bytes last second: %s" % (len(seen), guess_protocol(bytes(seen))))
                seen = bytearray()
    except KeyboardInterrupt:
        print()


def main():
    parser = argparse.ArgumentParser(description="MTF-01P bench test")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=10.0, help="prints per second")
    parser.add_argument("--raw", action="store_true", help="hex dump instead of decoding")
    parser.add_argument("--test", choices=sorted(bench_tests.TESTS.keys()), help="one guided test with a short summary")
    arguments = parser.parse_args()

    try:
        port = serial.Serial(arguments.port, arguments.baud, timeout=0.1)
    except serial.SerialException as error:
        print("Can't open %s: %s" % (arguments.port, error))
        sys.exit(1)

    print("Listening on %s at %d baud, Ctrl+C to stop" % (arguments.port, arguments.baud))

    if arguments.raw:
        raw_dump(port)
        port.close()
        return

    if arguments.test:
        try:
            bench_tests.TESTS[arguments.test](port, Mtf01pReader())
        except KeyboardInterrupt:
            print()
        port.close()
        return

    reader = Mtf01pReader()
    recent_bytes = bytearray()
    newest = None
    last_print = time.time()
    last_data = time.time()
    frames_at_last_print = 0

    try:
        while True:
            data = port.read(256)
            if data:
                last_data = time.time()
                recent_bytes += data
            for rx_byte in data:
                reading = reader.add_byte(rx_byte)
                if reading is not None:
                    newest = reading

            now = time.time()
            if now - last_print >= 1.0 / arguments.rate:
                frames_per_second = (reader.good_frames - frames_at_last_print) / (now - last_print)
                frames_at_last_print = reader.good_frames
                last_print = now

                if now - last_data > 1.0:
                    print("No bytes coming in, check wiring and baud")
                elif newest is None:
                    print("Bytes coming in but no Micolink frames (%s), set the sensor to Micolink in MicoAssistant"
                          % guess_protocol(bytes(recent_bytes)))
                else:
                    print_reading(newest, reader, frames_per_second)
                recent_bytes = bytearray()
    except KeyboardInterrupt:
        print("\n%d good frames, %d bad frames" % (reader.good_frames, reader.bad_frames))
    finally:
        port.close()


if __name__ == "__main__":
    main()
