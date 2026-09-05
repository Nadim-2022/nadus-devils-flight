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
#include "hardware/spi.h"
#include "receiver/Elrs.hpp"

extern "C" {
    uint32_t read_runtime_ctr(void) { return timer_hw->timerawl; }
}

#define ELRS_UART uart1
#define ELRS_BAUD 416666 //416666
#define ELRS_TX_PIN 4
#define ELRS_RX_PIN 5

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
    Receiver::Elrs elrs(ELRS_UART);

    while(1)
    {
        elrs.read_packet();
        
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
    //vTaskCoreAffinitySet(core0_task_handle, (1 << 0));
    vTaskCoreAffinitySet(core1_task_handle, (1 << 1));

    vTaskStartScheduler();

    while (true) {
        //tight_loop_contents();
    }
    return 0;
}
