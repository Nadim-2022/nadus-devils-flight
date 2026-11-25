#include "sensors/Mtf01p.hpp"
#include "debug/debug.hpp"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "Mtf01pUartRx.pio.h"


namespace Sensors
{

    Mtf01p *Mtf01p::active_sensor = nullptr;

    Mtf01p::Mtf01p(uint rx_pin, uint baud_rate) : pin(rx_pin), baud(baud_rate) {};

    bool Mtf01p::begin()
    {
        if (active_sensor != nullptr)
        {
            DEBUG_PRINTF("MTF-01P already started\n");
            return false;
        }

        if (!pio_claim_free_sm_and_add_program_for_gpio_range(&mtf01p_uart_rx_program, &pio, &state_machine, &program_offset, pin, 1, true))
        {
            DEBUG_PRINTF("MTF-01P: no free PIO state machine\n");
            return false;
        }

        pio_sm_set_consecutive_pindirs(pio, state_machine, pin, 1, false);
        pio_gpio_init(pio, pin);
        gpio_pull_up(pin); // line idles high

        pio_sm_config config = mtf01p_uart_rx_program_get_default_config(program_offset);
        sm_config_set_in_pins(&config, pin);
        sm_config_set_jmp_pin(&config, pin);
        sm_config_set_in_shift(&config, true, false, 32); // LSB first, byte ends up in the top 8 bits
        sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_RX);
        float clock_divider = (float)clock_get_hz(clk_sys) / (8.0f * baud);
        sm_config_set_clkdiv(&config, clock_divider);
        pio_sm_init(pio, state_machine, program_offset, &config);

        // every byte -> interrupt -> ring buffer
        active_sensor = this;
        uint irq_number = pio_get_irq_num(pio, 0);
        irq_set_exclusive_handler(irq_number, byte_received_isr);
        pio_set_irqn_source_enabled(pio, 0, pio_get_rx_fifo_not_empty_interrupt_source(state_machine), true);
        irq_set_enabled(irq_number, true);

        pio_sm_set_enabled(pio, state_machine, true);
        DEBUG_PRINTF("MTF-01P listening on GP%u at %u baud\n", pin, baud);
        return true;
    }

    void Mtf01p::byte_received_isr()
    {
        Mtf01p *sensor = active_sensor;
        while (!pio_sm_is_rx_fifo_empty(sensor->pio, sensor->state_machine))
        {
            uint8_t rx_byte = (uint8_t)(pio_sm_get(sensor->pio, sensor->state_machine) >> 24);
            sensor->received_bytes++;
            uint8_t next_write = sensor->ring_write + 1;
            if (next_write == sensor->ring_read)
            {
                sensor->lost_bytes++; // buffer full
            }
            else
            {
                sensor->ring_buffer[sensor->ring_write] = rx_byte;
                sensor->ring_write = next_write;
            }
        }
    }

    uint32_t Mtf01p::read_uint32(const uint8_t *bytes)
    {
        return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
    }

    int16_t Mtf01p::read_int16(const uint8_t *bytes)
    {
        return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
    }

    // payload of message 0x51, little endian
    // time_ms u32, distance u32 (mm), strength u8, precision u8, tof_status u8, reserved u8,
    // flow_vel_x i16, flow_vel_y i16 (cm/s at 1 m), flow_quality u8, flow_status u8, reserved u16
    void Mtf01p::decode_range_sensor(const uint8_t *payload)
    {
        reading.sensor_time_ms = read_uint32(&payload[0]);
        reading.distance_mm = read_uint32(&payload[4]);
        reading.strength = payload[8];
        reading.precision = payload[9];
        reading.distance_status = payload[10];

        int16_t flow_x_raw = read_int16(&payload[12]);
        int16_t flow_y_raw = read_int16(&payload[14]);
        reading.flow_quality = payload[16];
        reading.flow_status = payload[17];

        reading.distance_m = reading.distance_mm * 0.001f;
        reading.distance_ok = reading.distance_mm >= MIN_DISTANCE_MM && reading.distance_status == DISTANCE_STATUS_OK;

        // cm/s at 1 m -> m/s at 1 m
        reading.flow_x = flow_x_raw * 0.01f;
        reading.flow_y = flow_y_raw * 0.01f;
        reading.flow_ok = reading.flow_status == FLOW_STATUS_OK;

        reading.received_ms = to_ms_since_boot(get_absolute_time());
        reading.frame_count++;
    }

    // true when a new range / flow frame came in
    bool Mtf01p::read_frame()
    {
        bool got_frame = false;

        while (ring_read != ring_write)
        {
            uint8_t rx_byte = ring_buffer[ring_read];
            ring_read = ring_read + 1;

            switch (state)
            {
                case MicolinkState::WAIT_FOR_HEADER:
                    if (rx_byte == MICOLINK_HEADER)
                    {
                        frame[0] = rx_byte;
                        checksum = rx_byte;
                        state = MicolinkState::READ_DEVICE_ID;
                    }
                    break;

                case MicolinkState::READ_DEVICE_ID:
                    frame[1] = rx_byte;
                    checksum += rx_byte;
                    state = MicolinkState::READ_SYSTEM_ID;
                    break;

                case MicolinkState::READ_SYSTEM_ID:
                    frame[2] = rx_byte;
                    checksum += rx_byte;
                    state = MicolinkState::READ_MESSAGE_ID;
                    break;

                case MicolinkState::READ_MESSAGE_ID:
                    frame[3] = rx_byte;
                    checksum += rx_byte;
                    state = MicolinkState::READ_SEQUENCE;
                    break;

                case MicolinkState::READ_SEQUENCE:
                    frame[4] = rx_byte;
                    checksum += rx_byte;
                    state = MicolinkState::READ_LENGTH;
                    break;

                case MicolinkState::READ_LENGTH:
                    frame[5] = rx_byte;
                    checksum += rx_byte;
                    payload_length = rx_byte;
                    payload_index = 0;
                    if (payload_length > MAX_PAYLOAD_LENGTH)
                    {
                        bad_frames++;
                        state = MicolinkState::WAIT_FOR_HEADER;
                    }
                    else if (payload_length == 0)
                    {
                        state = MicolinkState::READ_CHECKSUM;
                    }
                    else
                    {
                        state = MicolinkState::READ_PAYLOAD;
                    }
                    break;

                case MicolinkState::READ_PAYLOAD:
                    frame[6 + payload_index] = rx_byte;
                    checksum += rx_byte;
                    payload_index++;
                    if (payload_index == payload_length)
                    {
                        state = MicolinkState::READ_CHECKSUM;
                    }
                    break;

                case MicolinkState::READ_CHECKSUM:
                    // checksum = sum of all bytes before it, 8 bit
                    if (rx_byte == checksum)
                    {
                        if (frame[3] == MESSAGE_RANGE_SENSOR && payload_length >= RANGE_SENSOR_PAYLOAD_LENGTH)
                        {
                            decode_range_sensor(&frame[6]);
                            got_frame = true;
                        }
                    }
                    else
                    {
                        bad_frames++;
                    }
                    state = MicolinkState::WAIT_FOR_HEADER;
                    break;
            }
        }

        return got_frame;
    }

}
