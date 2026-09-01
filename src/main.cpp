#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <pico/time.h>
#include "hardware/clocks.h" 
#include "hardware/vreg.h"   
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "debug/debug.hpp"

extern "C" {
    uint32_t read_runtime_ctr(void) { return timer_hw->timerawl; }
}

#define ELRS_UART uart0
#define ELRS_BAUD 416666 //416666
#define ELRS_TX_PIN 0
#define ELRS_RX_PIN 1

TaskHandle_t elrs_task_handle = NULL;
TaskHandle_t core0_task_handle = NULL;
TaskHandle_t core1_task_handle = NULL;


// CRSF standard CRC8 polynomial is 0xD5
uint8_t crsf_crc8(uint8_t *data, uint8_t len) {
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

long map_val(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ========================================================
// CORE 1: Clock Speed Verification
// ========================================================
void core1_flight_task(void *pvParameters) {
    (void)pvParameters;
    
    while (true) {
        // Read the actual system clock frequency directly from the hardware
        uint32_t clock_hz = clock_get_hz(clk_sys);
        
        // Print it in Hz and MHz
        DEBUG_PRINTF("[Core 1] CPU Clock Speed: %lu Hz (%.1f MHz)\n", clock_hz, clock_hz / 1000000.0f);

        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

// ========================================================
// CORE 0: Temperature Verification
// ========================================================
void core0_system_task(void *pvParameters) {
    (void)pvParameters;
    
    // The ADC is 12-bit, powered by 3.3V
    const float conversion_factor = 3.3f / (1 << 12);
    
    while (true) {
        // Select ADC input 4, which is internally wired to the temperature sensor
        adc_select_input(4); 
        uint16_t raw_adc = adc_read();
        
        // Convert raw ADC reading to voltage
        float voltage = raw_adc * conversion_factor;
        
        // Convert voltage to Celsius using the standard RP-series formula
        float temp_c = 27.0f - (voltage - 0.706f) / 0.001721f;
        
        DEBUG_PRINTF("[Core 0] Internal Temperature: %.2f C\n", temp_c);
        
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

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
                            DEBUG_PRINTF("CH1:%u CH2:%u CH3:%u CH4:%u CH5:%u CH6:%u CH7:%u CH8:%u CH9:%u CH10:%u\n", 
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

int main() {
    sleep_ms(2000);
    // 1. Overclocking Sequence
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_ms(10); 
    set_sys_clock_khz(300000, true);
    // 2. Standard Initialization
    stdio_init_all();
    
    // 3. Initialize ADC and enable the temperature sensor hardware
    adc_init();
    adc_set_temp_sensor_enabled(true);

    sleep_ms(2000); 

    // Initialize UART 0 for ELRS
    uart_init(ELRS_UART, ELRS_BAUD);
    gpio_set_function(ELRS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(ELRS_RX_PIN, GPIO_FUNC_UART);

    DEBUG_PRINTF("========================================\n");
    DEBUG_PRINTF("Nadus Devil's Flight - Hardware Verification\n");
    DEBUG_PRINTF("========================================\n");

    /*
    
    xTaskCreate(
        core0_system_task, 
        "TempTask", 
        256, 
        NULL, 
        1, 
        &core0_task_handle 
    );
    */

    xTaskCreate(
        core1_flight_task, 
        "ClockTask", 
        256, 
        NULL, 
        1, 
        &core1_task_handle
    );

    xTaskCreate(
        core0_elrs_task, 
        "ELRSTask", 
        512,        
        NULL, 
        2,          
        &elrs_task_handle
    );


    vTaskCoreAffinitySet(elrs_task_handle, (1 << 0));
   // vTaskCoreAffinitySet(core0_task_handle, (1 << 0));
    vTaskCoreAffinitySet(core1_task_handle, (1 << 1));

    vTaskStartScheduler();

    while (true) {
        tight_loop_contents();
    }
    return 0;
}
