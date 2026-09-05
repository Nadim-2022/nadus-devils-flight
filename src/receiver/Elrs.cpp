#include "receiver/Elrs.hpp"
#include "debug/debug.hpp"




namespace Receiver 
{

    Elrs::Elrs(uart_inst_t *uart_instance) : uart(uart_instance) {};

    uint8_t Elrs::crc8(uint8_t *data, uint8_t len) 
    {
        uint8_t crc = 0;
        for (uint8_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ 0xD5;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }

    bool Elrs::is_connected() 
    {
        return is_tx_connected;
    }

    bool Elrs::set_keep_running(bool value) 
    {
        keep_running = value;
        return keep_running;
    }

    ElrsState Elrs::get_state() 
    {
        return state;
    }

    long Elrs::map_val(long x, long in_min, long in_max, long out_min, long out_max) 
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    void Elrs::process_packet(uint8_t *payload, uint8_t payload_len) 
    {
        // Placeholder for processing the payload based on its type
        // For example, if the packet type is RC_CHANNELS, unpack the channels
        if (payload_len > 0) 
        {
            switch (payload[0]) 
            {
                case static_cast<uint8_t>(ElrsPacketType::RC_CHANNELS):
                    // Unpack 10 channels (11 bits each)
                    rc_channels[0] = (payload[1]       | payload[2] << 8)                     & 0x07FF;
                    rc_channels[1] = (payload[2] >> 3  | payload[3] << 5)                     & 0x07FF;
                    rc_channels[2] = (payload[3] >> 6  | payload[4] << 2  | payload[5] << 10) & 0x07FF;
                    rc_channels[3] = (payload[5] >> 1  | payload[6] << 7)                     & 0x07FF;
                    rc_channels[4] = (payload[6] >> 4  | payload[7] << 4)                     & 0x07FF;
                    rc_channels[5] = (payload[7] >> 7  | payload[8] << 1  | payload[9] << 9)  & 0x07FF;
                    rc_channels[6] = (payload[9] >> 2  | payload[10] << 6)                     & 0x07FF;
                    rc_channels[7] = (payload[10] >> 5 | payload[11] << 3)                    & 0x07FF;
                    rc_channels[8] = (payload[12]      | payload[13] << 8)                    & 0x07FF;
                    rc_channels[9] = (payload[13] >> 3 | payload[14] << 5)                    & 0x07FF;

                    for (int i = 0; i < 10; i++) 
                    {
                        rc_channels[i] = map_val(rc_channels[i], 191, 1792, 1000, 2000);
                    }

                   

                    break;
                case static_cast<uint8_t>(ElrsPacketType::LINK_STATISTICS):
                    // Process link statistics payload
                    if (payload_len >= 4) {
                        link_quality = payload[1];
                        rssi_dbm = payload[2];
                        snr = static_cast<int8_t>(payload[3]);
                        is_tx_connected = true; // Assuming we consider link statistics as a sign of connection
                    }
                    break;
                default:
                    // Handle unknown packet types
                    break;
            }
        }
    }


    void Elrs::read_packet() 
    {
     
        while (uart_is_readable(uart)) 
        {
            uint8_t rx_byte = uart_getc(uart);

            switch (get_state()) 
            {
                case ElrsState::WAIT_FOR_SYNC: // Looking for Sync Byte
                    if (rx_byte == 0xC8) 
                    {
                        state = ElrsState::READ_LENGTH;
                    }
                    break;
                    
                case ElrsState::READ_LENGTH: // Reading Length
                    payload_len_ = rx_byte;
                    // FIX: Allow any valid CRSF payload length (max 62), 
                    // otherwise we will throw away telemetry frames!
                    if (payload_len_ > 1 && payload_len_ <= 62) 
                    { 
                        index = 0;
                        state = ElrsState::READ_PAYLOAD;
                    } else 
                    {
                        state = ElrsState::WAIT_FOR_SYNC; // Invalid length, reset
                    }
                    break;
                    
                case ElrsState::READ_PAYLOAD: // Reading Type, Payload, and CRC

                    rx_buffer_[index++] = rx_byte;
                    if (index == payload_len_) 
                    {
                        uint8_t calculated_crc = crc8(rx_buffer_, payload_len_ - 1);
                        
                        // Separate CRC check from Type check
                        if (calculated_crc == rx_buffer_[payload_len_ - 1]) 
                        {
                            // We got a mathematically valid packet! Update the timeout tracker.
                            last_packet_time = xTaskGetTickCount();

                            // Process the packet based on its type
                            process_packet(&rx_buffer_[0], payload_len_ - 1); // Skip the type byte


                            DEBUG_PRINTF("CH1:%u CH2:%u CH3:%u CH4:%u CH5:%u CH6:%u CH7:%u CH8:%u CH9:%u CH10:%u\n", 
                                rc_channels[0], rc_channels[1], rc_channels[2], rc_channels[3], 
                                rc_channels[4], rc_channels[5], rc_channels[6], rc_channels[7],
                                rc_channels[8], rc_channels[9]);

                            
                        }
                        
                        // Reset to catch the next packet
                        state = ElrsState::WAIT_FOR_SYNC; 
                    }
                                
                    break;


            }
                        
        }
    


        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_packet_time) > pdMS_TO_TICKS(500)) 
        {
            if (is_tx_connected)
            {
                DEBUG_PRINTF("⚠️ FAILSAFE: Receiver Disconnected (UART Timeout)\n");
                is_tx_connected = false;  
            }
        }
        else 
        {
            if (!is_tx_connected) 
            {
                DEBUG_PRINTF("✅ LINK RESTORED!\n");
                is_tx_connected = true;
            }
        }                       
    
    }

} // namespace Receiver
