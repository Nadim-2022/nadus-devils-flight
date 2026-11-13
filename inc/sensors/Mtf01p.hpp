#ifndef MTF01P_HPP
#define MTF01P_HPP


#include "pico/stdlib.h"
#include "hardware/pio.h"


// MicoAir MTF-01P optical flow + laser range sensor
// Micolink protocol, 115200 baud, 100 Hz (MicoAir MTF-01P manual + Micolink page)
// Sensor comes set to AUTO, set it to Micolink in MicoAssistant first
// Both hardware UARTs are in use, so the sensor Tx is read with a PIO state machine


namespace Sensors
{

    enum class MicolinkState : uint8_t
    {
        WAIT_FOR_HEADER,
        READ_DEVICE_ID,
        READ_SYSTEM_ID,
        READ_MESSAGE_ID,
        READ_SEQUENCE,
        READ_LENGTH,
        READ_PAYLOAD,
        READ_CHECKSUM
    };

    struct OpticalFlowReading
    {
        uint32_t sensor_time_ms = 0; // sensor's own clock
        uint32_t received_ms = 0; // pico time when the frame came in
        uint32_t frame_count = 0; // goes up by one every good frame

        uint32_t distance_mm = 0; // 0 = no reading
        float distance_m = 0.0f;
        uint8_t strength = 0;
        uint8_t precision = 0;
        uint8_t distance_status = 0;
        bool distance_ok = false;

        // flow in m/s at 1 m height, same as rad/s
        // ground speed = flow * height
        float flow_x = 0.0f;
        float flow_y = 0.0f;
        uint8_t flow_quality = 0;
        uint8_t flow_status = 0;
        bool flow_ok = false;
    };


    class Mtf01p
    {

        public:
            explicit Mtf01p(uint rx_pin, uint baud_rate = 115200);
            ~Mtf01p() = default;
            bool begin();
            bool read_frame();
            uint32_t get_bad_frames() const { return bad_frames; }
            uint32_t get_lost_bytes() const { return lost_bytes; }

            OpticalFlowReading reading;

            // status values taken as good, not in the MicoAir docs, check on the bench
            static constexpr uint8_t DISTANCE_STATUS_OK = 1;
            static constexpr uint8_t FLOW_STATUS_OK = 1;
            static constexpr uint32_t MIN_DISTANCE_MM = 10; // Micolink page: below 10 not valid


        private:
            static constexpr uint8_t MICOLINK_HEADER = 0xEF;
            static constexpr uint8_t MESSAGE_RANGE_SENSOR = 0x51;
            static constexpr uint8_t MAX_PAYLOAD_LENGTH = 64;
            static constexpr uint8_t RANGE_SENSOR_PAYLOAD_LENGTH = 20;

            static void byte_received_isr();
            void decode_range_sensor(const uint8_t *payload);
            static uint32_t read_uint32(const uint8_t *bytes);
            static int16_t read_int16(const uint8_t *bytes);

            uint pin;
            uint baud;
            PIO pio = nullptr;
            uint state_machine = 0;
            uint program_offset = 0;

            // ring buffer filled by the PIO interrupt
            static Mtf01p *active_sensor;
            volatile uint8_t ring_buffer[256];
            volatile uint8_t ring_write = 0;
            volatile uint8_t ring_read = 0;
            volatile uint32_t lost_bytes = 0;

            MicolinkState state = MicolinkState::WAIT_FOR_HEADER;
            uint8_t frame[6 + MAX_PAYLOAD_LENGTH];
            uint8_t payload_length = 0;
            uint8_t payload_index = 0;
            uint8_t checksum = 0;
            uint32_t bad_frames = 0;

    };

}

#endif // MTF01P_HPP
