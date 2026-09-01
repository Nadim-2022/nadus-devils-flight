/*

void core0_elrs_task(void *pvParameters) {
    (void)pvParameters;
    
    printf("[Core 0] ELRS CRSF Task Initialized at 420k Baud\n");

    uint8_t rx_buffer[64];
    uint8_t state = 0;
    uint8_t payload_len = 0;
    uint8_t index = 0;
    
    // Array to hold our final 11-bit channel values
    uint16_t rc_channels[16] = {0}; 

    while (true) {
        while (uart_is_readable(ELRS_UART)) {
            uint8_t rx_byte = uart_getc(ELRS_UART);

            switch (state) {
            case 0: // Looking for Sync Byte
                if (rx_byte == 0xC8) {
                    state = 1;
                }
                break;
                
            case 1: // Reading Length
                payload_len = rx_byte;
                // A standard RC channels packet length is 24 (Type + 22 Payload + CRC)
                if (payload_len == 24) { 
                    index = 0;
                    state = 2;
                } else {
                    state = 0; // Invalid length for RC data, reset
                }
                break;
                
            case 2: // Reading Type, Payload, and CRC
                rx_buffer[index++] = rx_byte;
                
                if (index == payload_len) {
                    // The packet is fully received. Now verify it.
                    uint8_t calculated_crc = crsf_crc8(rx_buffer, payload_len - 1);
                    
                    if (calculated_crc == rx_buffer[payload_len - 1] && rx_buffer[0] == 0x16) {
                        // CRC matches and Type is 0x16 (RC Channels)
                        // Skip rx_buffer[0] (Type), payload starts at rx_buffer[1]
                        uint8_t *payload = &rx_buffer[1];

                        // Unpack 10 channels (11 bits each)
                        rc_channels[0] = (payload[0]       | payload[1] << 8)                     & 0x07FF;
                        rc_channels[1] = (payload[1] >> 3  | payload[2] << 5)                     & 0x07FF;
                        rc_channels[2] = (payload[2] >> 6  | payload[3] << 2  | payload[4] << 10) & 0x07FF;
                        rc_channels[3] = (payload[4] >> 1  | payload[5] << 7)                     & 0x07FF;
                        rc_channels[4] = (payload[5] >> 4  | payload[6] << 4)                     & 0x07FF;
                        rc_channels[5] = (payload[6] >> 7  | payload[7] << 1  | payload[8] << 9)  & 0x07FF;
                        rc_channels[6] = (payload[8] >> 2  | payload[9] << 6)                     & 0x07FF;
                        rc_channels[7] = (payload[9] >> 5  | payload[10] << 3)                    & 0x07FF;
                        
                        // The bit-shifting pattern repeats every 8 channels (11 bytes)!
                        rc_channels[8] = (payload[11]      | payload[12] << 8)                    & 0x07FF;
                        rc_channels[9] = (payload[12] >> 3 | payload[13] << 5)                    & 0x07FF;
                       
                       
                        // Optional: Map raw 172-1811 values to standard 1000-2000 PWM values
                        // Button have lowest 191 and highest 1792, so we map that to 1000-2000 range for easier use in flight controllers
                        for (int i = 0; i < 10; i++) {
                            rc_channels[i] = map_val(rc_channels[i], 191, 1792, 1000, 2000);
                        }
                        

                        // Print the first 10 channels
                        printf("CH1:%u CH2:%u CH3:%u CH4:%u CH5:%u CH6:%u CH7:%u CH8:%u CH9:%u CH10:%u\n", 
                            rc_channels[0], rc_channels[1], rc_channels[2], rc_channels[3], 
                            rc_channels[4], rc_channels[5], rc_channels[6], rc_channels[7],
                            rc_channels[8], rc_channels[9]);
                    }
                    
                    // Reset to catch the next packet
                    state = 0; 
                }
                break;
        }
        }
        
        // Yield to let other system tasks run
        vTaskDelay(pdMS_TO_TICKS(4));
    }
}


*/


/*
void core0_elrs_task(void *pvParameters) {
    (void)pvParameters;
    
    printf("[Core 0] ELRS CRSF Task Initialized at 420k Baud\n");

    uint8_t rx_buffer[64];
    uint8_t state = 0;
    uint8_t payload_len = 0;
    uint8_t index = 0;
    uint16_t rc_channels[16] = {0}; 
    
    // --- NEW: Connection Tracking Variables ---
    TickType_t last_packet_time = 0;
    uint8_t link_quality = 0;
    bool is_connected = false;

    while (true) {
        while (uart_is_readable(ELRS_UART)) {
            uint8_t rx_byte = uart_getc(ELRS_UART);

            switch (state) {
                case 0: // Looking for Sync Byte
                    if (rx_byte == 0xC8) {
                        state = 1;
                    }
                    break;
                    
                case 1: // Reading Length
                    payload_len = rx_byte;
                    // FIX: Allow any valid CRSF payload length (max 62), 
                    // otherwise we will throw away telemetry frames!
                    if (payload_len > 1 && payload_len <= 62) { 
                        index = 0;
                        state = 2;
                    } else {
                        state = 0; // Invalid length, reset
                    }
                    break;
                    
                case 2: // Reading Type, Payload, and CRC
                    rx_buffer[index++] = rx_byte;
                    
                    if (index == payload_len) {
                        uint8_t calculated_crc = crsf_crc8(rx_buffer, payload_len - 1);
                        
                        // Separate CRC check from Type check
                        if (calculated_crc == rx_buffer[payload_len - 1]) {
                            
                            // We got a mathematically valid packet! Update the timeout tracker.
                            last_packet_time = xTaskGetTickCount();

                            // ------------------------------------------------
                            // TYPE 0x16: RC CHANNELS
                            // ------------------------------------------------
                            if (rx_buffer[0] == 0x16) {
                                uint8_t *payload = &rx_buffer[1];

                                // Unpack 10 channels (11 bits each)
                                rc_channels[0] = (payload[0]       | payload[1] << 8)                     & 0x07FF;
                                rc_channels[1] = (payload[1] >> 3  | payload[2] << 5)                     & 0x07FF;
                                rc_channels[2] = (payload[2] >> 6  | payload[3] << 2  | payload[4] << 10) & 0x07FF;
                                rc_channels[3] = (payload[4] >> 1  | payload[5] << 7)                     & 0x07FF;
                                rc_channels[4] = (payload[5] >> 4  | payload[6] << 4)                     & 0x07FF;
                                rc_channels[5] = (payload[6] >> 7  | payload[7] << 1  | payload[8] << 9)  & 0x07FF;
                                rc_channels[6] = (payload[8] >> 2  | payload[9] << 6)                     & 0x07FF;
                                rc_channels[7] = (payload[9] >> 5  | payload[10] << 3)                    & 0x07FF;
                                rc_channels[8] = (payload[11]      | payload[12] << 8)                    & 0x07FF;
                                rc_channels[9] = (payload[12] >> 3 | payload[13] << 5)                    & 0x07FF;
                               
                                for (int i = 0; i < 10; i++) {
                                    rc_channels[i] = map_val(rc_channels[i], 191, 1792, 1000, 2000);
                                }
                            }
                            // ------------------------------------------------
                            // TYPE 0x14: LINK STATISTICS (Telemetry)
                            // ------------------------------------------------
                            else if (rx_buffer[0] == 0x14) {
                                uint8_t *payload = &rx_buffer[1];
                                
                                // CRSF sends RSSI inverted. If payload is 100, it means -100dBm
                                uint8_t rssi_dbm = payload[0]; 
                                link_quality     = payload[2];       // 0 to 100%
                                int8_t snr       = (int8_t)payload[3]; // Signed SNR value
                                
                                // Optional: Uncomment to monitor RF stats in your console
                                printf("LQ: %u%% | RSSI: -%udBm | SNR: %ddB\n", link_quality, rssi_dbm, snr);
                            }
                        }
                        
                        state = 0; 
                    }
                    break;
            }
        }
        
        // --- FAILSAFE EVALUATION ---
        TickType_t current_time = xTaskGetTickCount();
        
        // Condition 1: Pico is no longer receiving UART data (Wire cut / No power)
        if ((current_time - last_packet_time) > pdMS_TO_TICKS(500)) {
            if (is_connected) {
                printf("FAILSAFE: Receiver disconnected from Pico! (Timeout)\n");
                is_connected = false;
                link_quality = 0; 
                // TODO: Force rc_channels to failsafe values here (e.g. Throttle to 1000)
            }
        } 
        // Condition 2: UART is fine, but Radio is off / Out of range
        else if (link_quality == 0) {
            if (is_connected) {
                printf("FAILSAFE: Radio Link Lost! (LQ = 0)\n");
                is_connected = false;
                // TODO: Force rc_channels to failsafe values here
            }
        } 
        // Condition 3: Link is active and healthy
        else {
            if (!is_connected) {
                printf("LINK RESTORED!\n");
                is_connected = true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(4));
    }
}

*/

/*
void core0_elrs_task(void *pvParameters) {
    (void)pvParameters;
    
    printf("[Core 0] ELRS CRSF Task Initialized\n");

    uint8_t rx_buffer[64];
    uint8_t state = 0;
    uint8_t payload_len = 0;
    uint8_t index = 0;
    uint16_t rc_channels[16] = {0}; 
    
    // --- Connection Tracking Variables ---
    TickType_t last_packet_time = 0;
    TickType_t last_print_time = 0;
    uint8_t link_quality = 0;
    uint8_t rssi_dbm = 0;
    int8_t snr = 0;
    bool is_connected = false;

    while (true) {
        while (uart_is_readable(ELRS_UART)) {
            uint8_t rx_byte = uart_getc(ELRS_UART);

            switch (state) {
                case 0: // Looking for Sync Byte
                    if (rx_byte == 0xC8) {
                        state = 1;
                    }
                    break;
                    
                case 1: // Reading Length
                    payload_len = rx_byte;
                    // Accept any valid CRSF frame length (max 62)
                    if (payload_len > 1 && payload_len <= 62) { 
                        index = 0;
                        state = 2;
                    } else {
                        state = 0;
                    }
                    break;
                    
                case 2: // Reading Type, Payload, and CRC
                    rx_buffer[index++] = rx_byte;
                    
                    if (index == payload_len) {
                        uint8_t calculated_crc = crsf_crc8(rx_buffer, payload_len - 1);
                        
                        if (calculated_crc == rx_buffer[payload_len - 1]) {
                            // Valid CRC: update timestamp
                            last_packet_time = xTaskGetTickCount();

                            // ------------------------------------------------
                            // TYPE 0x16: RC CHANNELS
                            // ------------------------------------------------
                            if (rx_buffer[0] == 0x16) {
                                uint8_t *payload = &rx_buffer[1];

                                // Unpack 10 channels (11 bits each)
                                rc_channels[0] = (payload[0]       | payload[1] << 8)                     & 0x07FF;
                                rc_channels[1] = (payload[1] >> 3  | payload[2] << 5)                     & 0x07FF;
                                rc_channels[2] = (payload[2] >> 6  | payload[3] << 2  | payload[4] << 10) & 0x07FF;
                                rc_channels[3] = (payload[4] >> 1  | payload[5] << 7)                     & 0x07FF;
                                rc_channels[4] = (payload[5] >> 4  | payload[6] << 4)                     & 0x07FF;
                                rc_channels[5] = (payload[6] >> 7  | payload[7] << 1  | payload[8] << 9)  & 0x07FF;
                                rc_channels[6] = (payload[8] >> 2  | payload[9] << 6)                     & 0x07FF;
                                rc_channels[7] = (payload[9] >> 5  | payload[10] << 3)                    & 0x07FF;
                                rc_channels[8] = (payload[11]      | payload[12] << 8)                    & 0x07FF;
                                rc_channels[9] = (payload[12] >> 3 | payload[13] << 5)                    & 0x07FF;
                               
                                for (int i = 0; i < 10; i++) {
                                    rc_channels[i] = map_val(rc_channels[i], 191, 1792, 1000, 2000);
                                }
                            }
                            // ------------------------------------------------
                            // TYPE 0x14: LINK STATISTICS
                            // ------------------------------------------------
                            else if (rx_buffer[0] == 0x14) {
                                uint8_t *payload = &rx_buffer[1];
                                rssi_dbm     = payload[0]; 
                                link_quality = payload[2];
                                snr          = (int8_t)payload[3];
                            }
                        }
                        
                        state = 0; 
                    }
                    break;
            }
        }
        
        // --- FAILSAFE & CONNECTION EVALUATION ---
        TickType_t current_time = xTaskGetTickCount();
        
        // If no valid CRSF packet received in 500ms -> Physical Disconnect / Power Loss
        if (last_packet_time == 0 || (current_time - last_packet_time) > pdMS_TO_TICKS(500)) {
            if (is_connected) {
                printf("⚠️ FAILSAFE: Receiver Disconnected (UART Timeout)\n");
                is_connected = false;
            }
        } 
        // Packet arrived -> Link is alive
        else {
            if (!is_connected) {
                printf("✅ LINK RESTORED!\n");
                is_connected = true;
            }
        }

        // --- PRINT DATA (Rate-limited to every 100ms so serial doesn't lag) ---
        if (is_connected && (current_time - last_print_time) > pdMS_TO_TICKS(100)) {
            last_print_time = current_time;
            printf("LQ:%3u%% | RSSI:-%3udBm | CH1:%4u CH2:%4u CH3:%4u CH4:%4u CH5:%4u\n", 
                   link_quality, rssi_dbm, 
                   rc_channels[0], rc_channels[1], rc_channels[2], rc_channels[3], rc_channels[4]);
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
*/