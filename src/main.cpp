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
#include "sensors/BNO08x.h"
#include "math.h"
#include <ctype.h>


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
TaskHandle_t bno085_task_handle = NULL;


static constexpr float RAD2DEG = 57.2957795f;

enum SerialCmd {
    CMD_NONE = 0,
    CMD_CALIBRATE,   // "cal"
    CMD_STATUS,      // "status"
    CMD_UNKNOWN
};


/*!
 * @brief Check the serial console for a command (non-blocking).
 *
 * Collects characters until Enter is pressed, then returns the command.
 * Call it regularly (e.g. once per loop); it returns CMD_NONE while a
 * line is still being typed. Commands are not case-sensitive.
 */
SerialCmd serial_poll_command(void) {
    static char line[32];
    static size_t len = 0;

    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\r' || c == '\n') {
            if (len == 0) continue;          // ignore empty lines / "\r\n"
            line[len] = '\0';
            len = 0;

            if (strcmp(line, "cal") == 0)    return CMD_CALIBRATE;
            if (strcmp(line, "status") == 0) return CMD_STATUS;
            return CMD_UNKNOWN;
        }
        if (len < sizeof(line) - 1) {
            line[len++] = (char)tolower(c);
        }
    }
    return CMD_NONE;
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
        //DEBUG_PRINTF("[Core 1] CPU Clock Speed: %lu Hz (%.1f MHz)\n", clock_hz, clock_hz / 1000000.0f);

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
       
        uint32_t clock_hz = clock_get_hz(clk_sys);
        
        // Print it in Hz and MHz
        //DEBUG_PRINTF("[Core 1] CPU Clock Speed: %lu Hz (%.1f MHz)\n", clock_hz, clock_hz / 1000000.0f);

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

    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for system stabilization
    BMP5xx bmp;

    if (!bmp.begin(spi1, 10, 11, 12, 16, 3000000)) 
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
             
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}

static void bno_int_isr(uint gpio, uint32_t events) {
    if (gpio == 14 && bno085_task_handle) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(bno085_task_handle, &woken);
        portYIELD_FROM_ISR(woken);
    }
}



void bno085_task(void *pvParameters){

    BNO08x bno(spi1, 10, 11, 12, 13, 14,
                  15);

    if (!bno.begin()) 
    {
        DEBUG_PRINTF("Failed to find BNO08x\n");
        while (true) 
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    DEBUG_PRINTF("BNO08x found\n");
    for (int n = 0; n < bno.prodIds.numEntries; n++) 
    {
        DEBUG_PRINTF("Part %lu: v%u.%u.%u build %lu\n",
            (unsigned long)bno.prodIds.entry[n].swPartNumber,
            bno.prodIds.entry[n].swVersionMajor,
            bno.prodIds.entry[n].swVersionMinor,
            bno.prodIds.entry[n].swVersionPatch,
            (unsigned long)bno.prodIds.entry[n].swBuildNumber);
    }

   /*  while(serial_poll_command() != CMD_CALIBRATE) 
    {
        DEBUG_PRINTF("Type 'cal' to start calibration\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (bno.calibrate()) 
    {          // up to 2 minutes
        DEBUG_PRINTF("Calibration saved\n");
    } 
    else 
    {
        DEBUG_PRINTF("Calibration failed\n");
    }

 */

    bno.setDcdAutoSave(false); 
    if (!bno.enableReport(SH2_ROTATION_VECTOR, 5000)) 
    { 
        DEBUG_PRINTF("Could not enable rotation vector\n");
    }

    //--------------------BMP581-------------------------//

    BMP5xx bmp;

    if (!bmp.begin(spi1, 10, 11, 12, 16, 3000000)) 
    {
        DEBUG_PRINTF("BMP5xx not found, status %d\n", bmp.lastStatus());
    }
    
    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);   
    bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_127);   
    bmp.setOutputDataRate(BMP5XX_ODR_240_HZ);  
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
    
    // ========================================================//

    
    sh2_SensorValue_t value;
    static uint32_t count = 0, last_ms = 0, countPressure = 0;
    float qr = 1.0, qi = 0.0, qj = 0.0, qk = 0.0;
    float roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;

    gpio_set_irq_enabled_with_callback(14, GPIO_IRQ_EDGE_FALL,true, &bno_int_isr);

    while (true) 
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        if (bno.wasReset()) {
            DEBUG_PRINTF("Sensor was reset\n");
            bno.enableReport(SH2_ROTATION_VECTOR, 5000);
            sh2_SensorConfig_t cfg;
            if (sh2_getSensorConfig(SH2_ROTATION_VECTOR, &cfg) == SH2_OK)
            {
                DEBUG_PRINTF("RV actual interval: %lu us (%.1f Hz)\n",
                            (unsigned long)cfg.reportInterval_us,
                            cfg.reportInterval_us ? 1e6f / cfg.reportInterval_us : 0.0f);
            }

        }

        bool new_att = false;
        while (bno.getSensorEvent(&value)) {
            if (value.sensorId == SH2_ROTATION_VECTOR) {
                count++;
                qr = value.un.rotationVector.real;
                qi = value.un.rotationVector.i;
                qj = value.un.rotationVector.j;
                qk = value.un.rotationVector.k;
                new_att = true;
            }
        }

        
        if (new_att) {
            uint32_t t0 = time_us_32();
            float sinp = 2.0f * (qr * qj - qk * qi);
            if (sinp >  1.0f) sinp =  1.0f;   // guard against NaN
            if (sinp < -1.0f) sinp = -1.0f;

            roll_deg  = atan2f(2.0f * (qr * qi + qj * qk),
                            1.0f - 2.0f * (qi * qi + qj * qj)) * RAD2DEG;
            pitch_deg = asinf(sinp) * RAD2DEG;
            yaw_deg   = atan2f(2.0f * (qr * qk + qi * qj),
                            1.0f - 2.0f * (qj * qj + qk * qk)) * RAD2DEG;

            // run PID here, once per new attitude
           // uint32_t work_us = time_us_32() - t0; 
           // printf("Works time : %lu\n", work_us);
           //DEBUG_PRINTF("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll_deg, pitch_deg, yaw_deg);
            float heading = 90.0f - yaw_deg;
            if (heading < 0.0f)    heading += 360.0f;
            if (heading >= 360.0f) heading -= 360.0f;

            //printf("Orientation: %.2f, %.2f, %.2f, %.2f\n", yaw_deg, pitch_deg, roll_deg, heading);
            if(bmp.dataReady())
            {

                if ( bmp.performReading())
                {
                    float altitude = 44330.0f * (1.0f - powf(bmp.pressure / 1013.25, 0.1903f));
                    //DEBUG_PRINTF("Tempreture: %0.2f Pressure: %0.2f Altitude: %0.2f\n", bmp.temperature, bmp.pressure, altitude);
                    printf("Altitude: %0.2f, Pressure: %0.2f, Temperature: %0.2f\n", altitude, bmp.pressure, bmp.temperature);
                    countPressure++;
                }
                
            }
        }
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_ms >= 1000) {
            DEBUG_PRINTF("RV reports/s: %lu\n", (unsigned long)count);
            DEBUG_PRINTF("Pressure readings/s: %lu\n", (unsigned long)countPressure);
            //DEBUG_PRINTF("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll_deg, pitch_deg, yaw_deg);
            count = 0;
            countPressure = 0;
            last_ms = now;
        }
        
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

    
    
    xTaskCreate(core0_system_task, "TempTask", 256, NULL, 1, &core0_task_handle);

    //xTaskCreate(core1_flight_task, "ClockTask", 256, NULL, 1, &core1_task_handle);

    //xTaskCreate(core0_elrs_task, "ELRSTask", 256, NULL, 1, &elrs_task_handle);
    xTaskCreate(bno085_task, "ImuTask", 1024, NULL, 1, &bno085_task_handle);
    //xTaskCreate(bmp581_task,"BMPTask",1024,NULL,1,&bmp581_task_handle);


    //vTaskCoreAffinitySet(elrs_task_handle, (1 << 0));
    vTaskCoreAffinitySet(core0_task_handle, (1 << 0));
    //vTaskCoreAffinitySet(core1_task_handle, (1 << 1));
    vTaskCoreAffinitySet(bno085_task_handle, (1 << 1));
    //vTaskCoreAffinitySet(bmp581_task_handle, (1 << 1));

    
    vTaskStartScheduler();

    while (true) {
        //tight_loop_contents();
    }
    return 0;
}


 
