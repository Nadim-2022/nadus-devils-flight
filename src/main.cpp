#include <stdio.h>
#include <cstring>
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
#include "sensors/BMP5xx.h"


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
TaskHandle_t bmp581_task_handle = NULL;



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
    
    DEBUG_PRINTF("[Core 0] ELRS CRSF Task Initialized at 420k Baud\n");
    Receiver::Elrs elrs(ELRS_UART);

    while(1)
    {
        elrs.read_packet();
        
        // Yield to let other system tasks run
        vTaskDelay(pdMS_TO_TICKS(4));
    }
}






void bmp581_task(void *pvParameters) {
    (void)pvParameters;
    BMP5xx bmp;
    if (!bmp.begin(spi1, 10, 11, 12, 16, 1200000)) 
    {
        DEBUG_PRINTF("BMP5xx not found, status %d\n", bmp.lastStatus());
        
    }
    
    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);

   
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);

    
    bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_127);

    
    bmp.setOutputDataRate(BMP5XX_ODR_80_HZ);

   
    bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);

    
    bmp.enablePressure(true);

   
    bmp.configureInterrupt(BMP5XX_INTERRUPT_PULSED,
                            BMP5XX_INTERRUPT_ACTIVE_HIGH,
                            BMP5XX_INTERRUPT_PUSH_PULL,
                            BMP5XX_INTERRUPT_DATA_READY,
                            true);
    DEBUG_PRINTF("============================================================\n");
    DEBUG_PRINTF("Current settings:\n");
    DEBUG_PRINTF("  Temperature OSR: %d\n", bmp.getTemperatureOversampling());
    DEBUG_PRINTF("  Pressure OSR: %d\n", bmp.getPressureOversampling());
    DEBUG_PRINTF("  IIR Filter Coeff: %d\n", bmp.getIIRFilterCoeff());
    DEBUG_PRINTF("  Output Data Rate: %d\n", bmp.getOutputDataRate());
    DEBUG_PRINTF("============================================================\n");
    

    while (true) 
    {   
        if(bmp.dataReady())
        {

            if ( bmp.performReading())
            {
                float altitude = 44330.0f * (1.0f - powf(bmp.pressure / 1013.25, 0.1903f));
                DEBUG_PRINTF("Tempreture: %0.2f Pressure: %0.2f Altitude: %0.2f\n", bmp.temperature, bmp.pressure, altitude);
            }
            
        }
             
        vTaskDelay(pdMS_TO_TICKS(10)); // Read every second
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

    
    
    //xTaskCreate(core0_system_task, "TempTask", 256, NULL, 1, &core0_task_handle);

    xTaskCreate(bmp581_task,"BMPTask",256,NULL,1,&bmp581_task_handle);

    xTaskCreate(core1_flight_task, "ClockTask", 256, NULL, 1, &core1_task_handle);

    //xTaskCreate(core0_elrs_task, "ELRSTask", 256, NULL, 1, &elrs_task_handle);


    //vTaskCoreAffinitySet(elrs_task_handle, (1 << 0));
    vTaskCoreAffinitySet(bmp581_task_handle, (1 << 1));
    //vTaskCoreAffinitySet(core0_task_handle, (1 << 0));
    vTaskCoreAffinitySet(core1_task_handle, (1 << 1));
    
    vTaskStartScheduler();

    while (true) {
        //tight_loop_contents();
    }
    return 0;
}


 
