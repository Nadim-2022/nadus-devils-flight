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
#include "motor/Motors.hpp"
#include "math.h"
#include <ctype.h>
#include "filter/AltKalman.h"
#include "queue.h"

extern "C" {
    uint32_t read_runtime_ctr(void) { return timer_hw->timerawl; }
}

#define ELRS_UART uart1
#define ELRS_BAUD 416666 //416666
#define ELRS_TX_PIN 4
#define ELRS_RX_PIN 5

// BNO08x bno(spi1, 10, 11, 12, 13, 14, 15);
//bmp.begin(spi1, 10, 11, 12, 16, 3000000)

// Pin definitions for SPI1
#define SPI1_SCK_PIN 10
#define SPI1_MOSI_PIN 11
#define SPI1_MISO_PIN 12
#define SPI1_BUADRATE 3000000 // 3 MHz

// Pin definitions for BNO08x
#define BNO08x_CS_PIN 13
#define BNO08x_INT_PIN 14
#define BNO08x_RESET_PIN 15

// Pin definitions for BMP5xx
#define BMP5xx_CS_PIN 16

//Motor pins
#define MOTOR0_PIN 18
#define MOTOR1_PIN 19
#define MOTOR2_PIN 20
#define MOTOR3_PIN 21


TaskHandle_t elrs_task_handle = NULL;
TaskHandle_t core0_task_handle = NULL;
TaskHandle_t core1_task_handle = NULL;
TaskHandle_t bmp581_task_handle = NULL;
TaskHandle_t bno085_task_handle = NULL;
TaskHandle_t motor_test_handle = NULL;


static constexpr float RAD2DEG = 57.2957795f;



struct RcInput {
    uint16_t ch[16];     // channel values from ELRS
    uint32_t time_ms;    // when they were received (for failsafe)
};

static QueueHandle_t rc_mailbox;

// --- Channel setup (ELRS default AETR order) ---
#define CH_THROTTLE   2      // CH3 = index 2
#define CH_ARM        4     // CH5 = index 4 (AUX1 switch)
#define CH_MIN        992    // stick fully down  (CRSF raw values)
#define CH_MAX        2011   // stick fully up
#define CH_MID        1500
#define FAILSAFE_MS   250    // no new packet for this long -> stop motors



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

    while (1) {
        if (elrs.read_packet()) {          // only when a new packet arrived
            RcInput rc;
            for (int i = 0; i < 16; i++) rc.ch[i] = elrs.rc_channels[i];
            /* printf("RC Channels: ");
            for (int i = 0; i < 16; i++) {
                printf("%d ", rc.ch[i]);
            }
            printf("\n"); */
            rc.time_ms = to_ms_since_boot(get_absolute_time());
            xQueueOverwrite(rc_mailbox, &rc);   // always keep only the newest
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}




void bmp581_task(void *pvParameters) {

    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for system stabilization
    BMP5xx bmp (spi1, BMP5xx_CS_PIN); // Initialize BMP5xx with SPI1 and CS pin 16

    if (!bmp.initialize()) 
    {
        DEBUG_PRINTF("BMP5xx not found, status %d\n", bmp.lastStatus());
    }
    
    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_128X);
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_128X);   
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


static float g_ground_pa = 101325.0f;   // set at startup by baro_set_ground()

// Average the pressure for a moment while the drone sits on the ground
void baro_set_ground(float avg_pressure_pa) { g_ground_pa = avg_pressure_pa; }

// Height above the takeoff point, in metres
float baro_altitude_m(float pressure_pa) {
    return 44330.0f * (1.0f - powf(pressure_pa / g_ground_pa, 0.190295f));
}


float world_up_accel(float qr, float qi, float qj, float qk,
                     float ax, float ay, float az) {
    return 2.0f * (qi * qk - qr * qj) * ax +
           2.0f * (qj * qk + qr * qi) * ay +
           (1.0f - 2.0f * (qi * qi + qj * qj)) * az;
}

static AltKalman kf;

void bno085_task(void *pvParameters){


    // Initialize the SPI1 interface for the BNO08x & BMP5xx sensors
    spi_init(spi1, SPI1_BUADRATE);
    spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI1_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_MISO_PIN, GPIO_FUNC_SPI);

    
    //  BNO08x(spi_inst_t *spi, uint cs_pin, uint int_pin, int reset_pin = -1, int wake_pin = -1);
    BNO08x bno(spi1, BNO08x_CS_PIN, BNO08x_INT_PIN, BNO08x_RESET_PIN);

    if (!bno.initialize()) 
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
    
    
    if (!bno.enableReport(SH2_LINEAR_ACCELERATION, 5000))
    { 
        DEBUG_PRINTF("Could not enable linear acceleration\n");
    }
    if (!bno.enableReport(SH2_ROTATION_VECTOR, 2500)) 
    { 
        DEBUG_PRINTF("Could not enable rotation vector\n");
    }

    //--------------------BMP581-------------------------//

    BMP5xx bmp(spi1, BMP5xx_CS_PIN); // Initialize BMP5xx with SPI1 and CS pin 16

    if (!bmp.initialize()) 
    {
        DEBUG_PRINTF("BMP5xx not found, status %d\n", bmp.lastStatus());
    }
    
    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_1X);
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_8X);   
    bmp.setIIRFilterCoeff( BMP5XX_IIR_FILTER_BYPASS);   
    bmp.setOutputDataRate(BMP5XX_ODR_10_HZ);  
    bmp.setPowerMode(BMP5XX_POWERMODE_CONTINUOUS);

    
    bmp.enablePressure(true);

   
    bmp.configureInterrupt(BMP5XX_INTERRUPT_LATCHED,
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
    
    
    kf.setAccelNoise(0.1f);    // was 0.5
    kf.setBiasWalk(0.001f);    // was 0.01
    kf.setBaroNoise(0.3f);     // was 0.2
    kf.setGateSigma(4.0f);    // 0 = no spike rejection
    kf.reset(0.0f);

    sh2_SensorValue_t value;
    
    static uint32_t count = 0, last_ms = 0, countPressure = 0;
    float qr = 1.0, qi = 0.0, qj = 0.0, qk = 0.0;
    float roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;

    
    float lin_ax = 0, lin_ay = 0, lin_az = 0;
    float baro_h = 0.0f;
    uint32_t last_baro_ms  = 0;
    float a_up;
    uint32_t last_alt_us = time_us_32();

    gpio_set_irq_enabled_with_callback(14, GPIO_IRQ_EDGE_FALL,true, &bno_int_isr);

    float avg_dt = 0.0f, avg_alt=0.0f, avg_a_up=0.0f, avg_baro_h=0.0f, avg_alt_v=0.0f;
    int count_avg = 0;
    int count_linear_accel = 0;


    //==========================================================================================
        // Let the BMP581 settle after configuration (IIR filter restarts)
    uint32_t t_settle = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - t_settle < 1000) {
        if (bmp.dataReady()) bmp.performReading();   // read and discard
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Average ~2 s of readings (200 samples at 100 Hz)
    const int num_samples = 20;
    double sum = 0.0;                     // double: no precision loss
    float  p_min = 1e9f, p_max = 0.0f;
    for (int i = 0; i < num_samples; ) {
        if (bmp.dataReady() && bmp.performReading()) {
            sum += bmp.pressure;
            if (bmp.pressure < p_min) p_min = bmp.pressure;
            if (bmp.pressure > p_max) p_max = bmp.pressure;
            i++;
            
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    float avg_pressure = (float)(sum / num_samples);
    baro_set_ground(avg_pressure);
    printf("Ground pressure: %.4f hPa  (spread %.2f Pa)\n",
    avg_pressure, (p_max - p_min) * 100.0f);
    //=================================================================================





    while (true) 
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        if (bno.wasReset()) {
            DEBUG_PRINTF("Sensor was reset\n");
            bno.enableReport(SH2_ROTATION_VECTOR, 2500);
            sh2_SensorConfig_t cfg;
            if (sh2_getSensorConfig(SH2_ROTATION_VECTOR, &cfg) == SH2_OK)
            {
                DEBUG_PRINTF("RV actual interval: %lu us (%.1f Hz)\n",
                            (unsigned long)cfg.reportInterval_us,
                            cfg.reportInterval_us ? 1e6f / cfg.reportInterval_us : 0.0f);
            }
            if (sh2_getSensorConfig(SH2_LINEAR_ACCELERATION, &cfg) == SH2_OK)
            {
                DEBUG_PRINTF("LA actual interval: %lu us (%.1f Hz)\n",
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
            else if (value.sensorId == SH2_LINEAR_ACCELERATION) {
                lin_ax = value.un.linearAcceleration.x;
                lin_ay = value.un.linearAcceleration.y;
                lin_az = value.un.linearAcceleration.z;
                count_linear_accel++;
                //new_att = true;
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
                    //float altitude = 44330.0f * (1.0f - powf(bmp.pressure / 1013.25, 0.1903f));
                    baro_h = baro_altitude_m(bmp.pressure);
                    kf.updateBaro(baro_h);
                    uint32_t now_us = time_us_32();
                    float dt = (now_us - last_alt_us) * 1e-6f;
                    last_alt_us = now_us;

                    a_up = world_up_accel(qr, qi, qj, qk, lin_ax, lin_ay, lin_az);
                    
                    // low-pass filter the vertical acceleration to reduce noise
                    static float a_up_filt = 0.0f;
                    const float alpha = 0.1f; 
                    a_up_filt = alpha * a_up + (1.0f - alpha) * a_up_filt;
                    a_up = a_up_filt;
                    kf.predict(a_up, dt); 
                    //printf("Altitude: %.2f\n", kf.altitude());
                    //if (baro_h < 0.0f) baro_h = 0.0f;
                    printf("%0.4f \n", kf.altitude());
                
                    countPressure++;
                }
                
            }

            /* uint32_t now_us = time_us_32();
            float dt = (now_us - last_alt_us) * 1e-6f;
            last_alt_us = now_us;

            a_up = world_up_accel(qr, qi, qj, qk, lin_ax, lin_ay, lin_az);
            
            // low-pass filter the vertical acceleration to reduce noise
            static float a_up_filt = 0.0f;
            const float alpha = 0.1f; 
            a_up_filt = alpha * a_up + (1.0f - alpha) * a_up_filt;
            a_up = a_up_filt;
            kf.predict(a_up, dt); 
            printf("Altitude: %.2f\n", kf.altitude()); */
           
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_ms >= 1000) {
            DEBUG_PRINTF("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll_deg, pitch_deg, yaw_deg);
            //Print all the counts
            //DEBUG_PRINTF("Counts in last second: Attitude: %u, Linear Accel: %u, Pressure: %u\n", count, count_linear_accel, countPressure);
            count = 0;
            count_linear_accel = 0;
            countPressure = 0;
            last_ms = now;


        }
        
    }
   
}


// ========================================================
// BNO085 only: attitude (roll / pitch / yaw), no BMP581
// Uses bno085_task_handle so the existing bno_int_isr wakes it.
// ========================================================
void bno085_imu_task(void *pvParameters){


    // Initialize the SPI1 interface for the BNO08x & BMP5xx sensors
    spi_init(spi1, SPI1_BUADRATE);
    spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI1_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_MISO_PIN, GPIO_FUNC_SPI);

    
    //  BNO08x(spi_inst_t *spi, uint cs_pin, uint int_pin, int reset_pin = -1, int wake_pin = -1);
    BNO08x bno(spi1, BNO08x_CS_PIN, BNO08x_INT_PIN, BNO08x_RESET_PIN);

    if (!bno.initialize()) 
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
    
    
    if (!bno.enableReport(SH2_LINEAR_ACCELERATION, 5000))
    { 
        DEBUG_PRINTF("Could not enable linear acceleration\n");
    }
    // Game Rotation Vector = gyro + accel only, no magnetometer.
    // Motor current can't pull roll/pitch/yaw around. Yaw is relative to power-up.
    if (!bno.enableReport(SH2_GAME_ROTATION_VECTOR, 2500)) 
    { 
        DEBUG_PRINTF("Could not enable game rotation vector\n");
    }
    // Rotation Vector (uses the magnetometer) only for the compass heading, 50 Hz
    if (!bno.enableReport(SH2_ROTATION_VECTOR, 20000)) 
    { 
        DEBUG_PRINTF("Could not enable rotation vector\n");
    }

    sh2_SensorValue_t value;
    
    static uint32_t count = 0, last_ms = 0;
    float qr = 1.0, qi = 0.0, qj = 0.0, qk = 0.0;
    float roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
    float compass_deg = 0.0f, compass_acc_deg = 0.0f;   // magnetometer heading, 0 = north, clockwise

    
    float lin_ax = 0, lin_ay = 0, lin_az = 0;

    gpio_set_irq_enabled_with_callback(14, GPIO_IRQ_EDGE_FALL,true, &bno_int_isr);

    int count_linear_accel = 0;


    while (true) 
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        if (bno.wasReset()) {
            DEBUG_PRINTF("Sensor was reset\n");
            bno.enableReport(SH2_GAME_ROTATION_VECTOR, 2500);
            bno.enableReport(SH2_ROTATION_VECTOR, 20000);
            sh2_SensorConfig_t cfg;
            if (sh2_getSensorConfig(SH2_GAME_ROTATION_VECTOR, &cfg) == SH2_OK)
            {
                DEBUG_PRINTF("GRV actual interval: %lu us (%.1f Hz)\n",
                            (unsigned long)cfg.reportInterval_us,
                            cfg.reportInterval_us ? 1e6f / cfg.reportInterval_us : 0.0f);
            }
            if (sh2_getSensorConfig(SH2_ROTATION_VECTOR, &cfg) == SH2_OK)
            {
                DEBUG_PRINTF("RV actual interval: %lu us (%.1f Hz)\n",
                            (unsigned long)cfg.reportInterval_us,
                            cfg.reportInterval_us ? 1e6f / cfg.reportInterval_us : 0.0f);
            }
            if (sh2_getSensorConfig(SH2_LINEAR_ACCELERATION, &cfg) == SH2_OK)
            {
                DEBUG_PRINTF("LA actual interval: %lu us (%.1f Hz)\n",
                            (unsigned long)cfg.reportInterval_us,
                            cfg.reportInterval_us ? 1e6f / cfg.reportInterval_us : 0.0f);
            }

        }

        bool new_att = false;
        while (bno.getSensorEvent(&value)) {
            if (value.sensorId == SH2_GAME_ROTATION_VECTOR) {
                count++;
                qr = value.un.gameRotationVector.real;
                qi = value.un.gameRotationVector.i;
                qj = value.un.gameRotationVector.j;
                qk = value.un.gameRotationVector.k;
                new_att = true;
            }
            else if (value.sensorId == SH2_ROTATION_VECTOR) {
                // Magnetometer-referenced yaw -> compass heading (not used for control)
                float mr = value.un.rotationVector.real;
                float mi = value.un.rotationVector.i;
                float mj = value.un.rotationVector.j;
                float mk = value.un.rotationVector.k;
                float mag_yaw = atan2f(2.0f * (mr * mk + mi * mj),
                                       1.0f - 2.0f * (mj * mj + mk * mk)) * RAD2DEG;
                compass_deg = 90.0f - mag_yaw;
                if (compass_deg < 0.0f)    compass_deg += 360.0f;
                if (compass_deg >= 360.0f) compass_deg -= 360.0f;
                compass_acc_deg = value.un.rotationVector.accuracy * RAD2DEG;
            }
            else if (value.sensorId == SH2_LINEAR_ACCELERATION) {
                lin_ax = value.un.linearAcceleration.x;
                lin_ay = value.un.linearAcceleration.y;
                lin_az = value.un.linearAcceleration.z;
                count_linear_accel++;
                //new_att = true;
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
            //printf("Orientation: %.2f, %.2f, %.2f, %.2f\n", yaw_deg, pitch_deg, roll_deg, compass_deg);
           
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_ms >= 1000) {
            DEBUG_PRINTF("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Compass: %.1f deg (+/- %.1f), Az: %.3f m/s^2\n",
                         roll_deg, pitch_deg, yaw_deg, compass_deg, compass_acc_deg, lin_az);
            //Print all the counts
            //DEBUG_PRINTF("Counts in last second: Attitude: %u, Linear Accel: %u\n", count, count_linear_accel);
            count = 0;
            count_linear_accel = 0;
            last_ms = now;


        }
        
    }
   
}


static float stick_to_throttle(uint16_t v) {
    float t = (float)(v - CH_MIN) / (float)(CH_MAX - CH_MIN);
    if (t < 0.02f) t = 0.0f;        // bottom 2 % of the stick = idle
    if (t > 1.0f)  t = 1.0f;
    return t;
}

void motor_task(void *pvParameters) {
    (void)pvParameters;

    Motors::Config cfg;
    cfg.pins[0] = 18;  cfg.pins[1] = 19;  cfg.pins[2] = 20;  cfg.pins[3] = 21;
    cfg.count   = 4;
    static Motors motors(cfg);

    if (!motors.begin()) {
        DEBUG_PRINTF("Motors error: %s\n", motors.lastError());
        vTaskDelete(NULL);
    }
    printf("Motors initialized\n");
    vTaskDelay(pdMS_TO_TICKS(4000));          // ESC initialisation
    

   /*  motors.arm();
    motors.setAll(1.0f);                                  // 2000 us = MAX
    DEBUG_PRINTF("ESC CAL: MAX (2000 us). CONNECT BATTERY NOW.\n");
    vTaskDelay(pdMS_TO_TICKS(10000));                     // 10 s to plug in + beeps

    motors.disarm();                                      // 1000 us = MIN
    DEBUG_PRINTF("ESC CAL: MIN (1000 us). Wait for confirmation beeps.\n");
    vTaskDelay(pdMS_TO_TICKS(5000));

    DEBUG_PRINTF("ESC CAL: DONE. Disconnect battery.\n");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000)); */

    RcInput rc = {};
    bool have_rc = false;
    uint32_t last_print = 0;

    while (true) {
        if (xQueuePeek(rc_mailbox, &rc, 0) == pdTRUE) have_rc = true;

        uint32_t now = to_ms_since_boot(get_absolute_time());
        bool link_ok   = have_rc && (now - rc.time_ms) < FAILSAFE_MS;
        bool arm_sw    = link_ok && rc.ch[CH_ARM] > CH_MID;

        //printf("link_ok=%d, arm_sw=%d, throttle=%.2f rc.ch[CH_ARM]=%d\n", link_ok, arm_sw, stick_to_throttle(rc.ch[CH_THROTTLE]), rc.ch[CH_ARM]);
        float throttle = link_ok ? stick_to_throttle(rc.ch[CH_THROTTLE]) : 0.0f;

        if (!motors.isArmed()) {
            // Arm only with the switch ON and the throttle stick DOWN
            if (arm_sw && throttle < 0.05f) {
                motors.arm();
                DEBUG_PRINTF("ARMED\n");
            }
        } else if (!arm_sw) {
            // Switch OFF, or signal lost (failsafe)
            motors.disarm();
            //DEBUG_PRINTF(link_ok ? "DISARMED\n" : "FAILSAFE: signal lost, DISARMED\n");
            printf("Disarmed: link_ok=%d, arm_sw=%d, throttle=%.2f\n", link_ok, arm_sw, throttle);
        }

        motors.setAll(throttle);              // ignored (stop pulse) while disarmed

        if (now - last_print >= 1000) {
            last_print = now;
           /*  DEBUG_PRINTF("link:%d armed:%d throttle:%.2f pulse:%u us\n",
                         link_ok, motors.isArmed(), (double)throttle, motors.pulseUs(0)); */
            /* DEBUG_PRINTF("pulses: %u %u %u %u us\n",
             motors.pulseUs(0), motors.pulseUs(1), motors.pulseUs(2), motors.pulseUs(3)); */
        }
        vTaskDelay(pdMS_TO_TICKS(5));         // 200 Hz
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

    rc_mailbox = xQueueCreate(1, sizeof(RcInput));
    
    xTaskCreate(core0_system_task, "TempTask", 256, NULL, 1, &core0_task_handle);

    //xTaskCreate(core1_flight_task, "ClockTask", 256, NULL, 1, &core1_task_handle);

    xTaskCreate(core0_elrs_task, "ELRSTask", 256, NULL, 1, &elrs_task_handle);
    //xTaskCreate(bno085_task, "ImuTask", 1024, NULL, 1, &bno085_task_handle);
    xTaskCreate(bno085_imu_task, "BnoTask", 1024, NULL, 1, &bno085_task_handle);
    //xTaskCreate(bmp581_task,"BMPTask",1024,NULL,1,&bmp581_task_handle);
    xTaskCreate(motor_task, "MotorTest", 1024, NULL, 1, &motor_test_handle);

    vTaskCoreAffinitySet(elrs_task_handle, (1 << 0));
    //vTaskCoreAffinitySet(core0_task_handle, (1 << 0));
    //vTaskCoreAffinitySet(core1_task_handle, (1 << 1));
    vTaskCoreAffinitySet(bno085_task_handle, (1 << 1));
    //vTaskCoreAffinitySet(bmp581_task_handle, (1 << 1));
    vTaskCoreAffinitySet(motor_test_handle, (1 << 1));

    
    vTaskStartScheduler();

    while (true) {
        //tight_loop_contents();
    }
    return 0;
}


 
