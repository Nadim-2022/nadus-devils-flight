#ifndef ELRS_HPP
#define ELRS_HPP


#include "FreeRTOS.h"
#include "task.h"
#include "hardware/uart.h"


namespace Receiver 
{

    enum class ElrsPacketType : uint8_t 
    {
        RC_CHANNELS = 0x16,
        LINK_STATISTICS = 0x14,
        // Add other packet types as needed
    };

    enum class ElrsState : uint8_t 
    {
        WAIT_FOR_SYNC,
        READ_LENGTH,
        READ_PAYLOAD
    };


   
    class Elrs 
    {

        public:
            explicit Elrs(uart_inst_t *uart_instance);
            ~Elrs() = default;
            void read_packet();  
            bool is_connected();
            bool set_keep_running(bool value);
            
            
                
        private:
            uint8_t crc8(uint8_t *data, uint8_t len);
            ElrsState get_state();
            long map_val(long x, long in_min, long in_max, long out_min, long out_max);
            void process_packet(uint8_t *payload, uint8_t payload_len);
            uart_inst_t *uart;
            uint8_t rx_buffer_[64];
            ElrsState state= ElrsState::WAIT_FOR_SYNC;
            uint8_t payload_len_ = 0;
            uint8_t index= 0;
            uint16_t rc_channels[16] = {0};

            TickType_t last_packet_time = 0;
            uint8_t link_quality = 0;
            uint8_t rssi_dbm = 0;
            int8_t snr = 0;
            bool is_tx_connected= false;
            bool keep_running = true;


    };

}

#endif // ELRS_HPP