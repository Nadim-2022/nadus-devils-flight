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
#include "pid/Pid.hpp"

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


// ========================================================
// FLIGHT CONTROL
// roll / pitch : self level, stick = tilt angle
// yaw : stick = turn speed
// throttle : straight from the stick, no altitude hold, no BMP581
//
// Same BNO085 setup as bno085_imu_task + calibrated gyro at 400 Hz.
// Replaces bno085_imu_task and motor_task, don't run them together.
//
// Directions used in this part (looking from behind):
// roll + right side down
// pitch + nose up
// yaw + nose turns right
// ========================================================

// Stick channels (AETR)
#define CH_ROLL 0 // CH1
#define CH_PITCH 1 // CH2
#define CH_YAW 3 // CH4

// Stick direction, -1 to reverse
static constexpr float ROLL_STICK_DIRECTION = +1.0f; // stick right = roll right
static constexpr float PITCH_STICK_DIRECTION = +1.0f; // stick forward = nose down
static constexpr float YAW_STICK_DIRECTION = +1.0f; // stick right = yaw right
static constexpr float STICK_DEADBAND = 0.03f;

// BNO085 mounted flat, X arrow to the front, Y to the left, Z up
static constexpr float IMU_ROLL_DIRECTION = +1.0f;
static constexpr float IMU_PITCH_DIRECTION = -1.0f;
static constexpr float IMU_YAW_DIRECTION = -1.0f;

// Angle the drone reads when it sits on a level floor
static constexpr float ROLL_TRIM_DEG = 0.0f;
static constexpr float PITCH_TRIM_DEG = 0.0f;


// How much each motor does for roll / pitch / yaw (Quad X)
// roll +1 left motor, -1 right motor
// pitch +1 front motor, -1 rear motor
// yaw +1 prop turns counter clockwise, -1 clockwise
// Layout not checked on the frame yet (Betaflight props in order)
struct MotorMix
{
    float roll;
    float pitch;
    float yaw;
};

static const MotorMix motor_mix[4] =
{
    { -1.0f, -1.0f, -1.0f }, // motor 0 GP18 rear right CW
    { -1.0f, +1.0f, +1.0f }, // motor 1 GP19 front right CCW
    { +1.0f, -1.0f, +1.0f }, // motor 2 GP20 rear left CCW
    { +1.0f, +1.0f, -1.0f }, // motor 3 GP21 front left CW
};

// Flight limits
static constexpr float MAX_TILT_ANGLE_DEG = 25.0f; // full stick
static constexpr float MAX_TILT_SPEED_DEG_S = 150.0f; // max output of the angle loop
static constexpr float MAX_YAW_SPEED_DEG_S = 120.0f; // full yaw stick
static constexpr float PID_START_THROTTLE = 0.05f; // below this motors idle and PID off
static constexpr float INTEGRAL_START_THROTTLE = 0.15f; // below this I stays frozen (on the ground)

// Safety
static constexpr float ARM_MAX_TILT_DEG = 20.0f;
static constexpr float CRASH_TILT_DEG = 60.0f;
static constexpr float ARM_MAX_THROTTLE = 0.05f;
static constexpr uint32_t IMU_TIMEOUT_US = 100000; // 100 ms
static constexpr uint32_t ESC_STARTUP_MS = 4000;


// PID gains, first guesses, not tuned yet
// angle loop : in deg, out deg/s
// speed loop : in deg/s, out motor power (0..1)
static Pid::Settings angle_pid_settings()
{
    Pid::Settings settings;
    settings.proportional_gain = 4.0f;
    settings.output_limit = MAX_TILT_SPEED_DEG_S;
    return settings;
}

static Pid::Settings roll_pitch_speed_pid_settings()
{
    Pid::Settings settings;
    settings.proportional_gain = 0.0010f;
    settings.integral_gain = 0.0015f;
    settings.derivative_gain = 0.00002f;
    settings.integral_limit = 0.10f;
    settings.output_limit = 0.30f;
    settings.derivative_filter_hz = 40.0f;
    return settings;
}

static Pid::Settings yaw_speed_pid_settings()
{
    Pid::Settings settings;
    settings.proportional_gain = 0.0025f;
    settings.integral_gain = 0.0020f;
    settings.derivative_gain = 0.0f;
    settings.integral_limit = 0.10f;
    settings.output_limit = 0.20f;
    return settings;
}


enum DisarmReason : uint8_t
{
    DISARM_NONE = 0,
    DISARM_SWITCH,
    DISARM_FAILSAFE,
    DISARM_IMU_TIMEOUT,
    DISARM_CRASH_TILT,
};

static const char *disarm_reason_text(uint8_t reason)
{
    switch (reason)
    {
        case DISARM_SWITCH: return "arm switch off";
        case DISARM_FAILSAFE: return "FAILSAFE: RC link lost";
        case DISARM_IMU_TIMEOUT: return "IMU stopped sending";
        case DISARM_CRASH_TILT: return "CRASH: tilt over limit";
        default: return "-";
    }
}

// Sent from the flight task to the print task
struct FlightStatus
{
    bool armed;
    bool link_ok;
    uint8_t disarm_reason;
    const char *arm_blocked_by; // nullptr when nothing stops arming
    float throttle;
    float roll_deg, pitch_deg, yaw_deg;
    float roll_speed, pitch_speed, yaw_speed; // deg/s
    float target_roll, target_pitch, target_yaw_speed;
    float motor_power[4];
    float compass_deg;
    uint16_t loop_hz;
    uint32_t time_ms;
};

static QueueHandle_t status_mailbox;


// Centre stick -> -1 .. +1 with a small dead band
static float stick_position(uint16_t value)
{
    float position = ((float)value - (float)CH_MID) / (float)(CH_MAX - CH_MID);
    if (position > 1.0f) position = 1.0f;
    if (position < -1.0f) position = -1.0f;
    if (position > -STICK_DEADBAND && position < STICK_DEADBAND) return 0.0f;

    if (position > 0.0f)
    {
        return (position - STICK_DEADBAND) / (1.0f - STICK_DEADBAND);
    }
    return (position + STICK_DEADBAND) / (1.0f - STICK_DEADBAND);
}

static void enable_flight_reports(BNO08x &bno)
{
    if (!bno.enableReport(SH2_GAME_ROTATION_VECTOR, 2500)) // 400 Hz, no magnetometer
    {
        DEBUG_PRINTF("Could not enable game rotation vector\n");
    }
    if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 2500)) // 400 Hz
    {
        DEBUG_PRINTF("Could not enable gyroscope\n");
    }
    if (!bno.enableReport(SH2_ROTATION_VECTOR, 20000)) // 50 Hz, compass only
    {
        DEBUG_PRINTF("Could not enable rotation vector\n");
    }
}


static Pid roll_angle_pid(angle_pid_settings());
static Pid pitch_angle_pid(angle_pid_settings());
static Pid roll_speed_pid(roll_pitch_speed_pid_settings());
static Pid pitch_speed_pid(roll_pitch_speed_pid_settings());
static Pid yaw_speed_pid(yaw_speed_pid_settings());

static void reset_all_pids()
{
    roll_angle_pid.reset();
    pitch_angle_pid.reset();
    roll_speed_pid.reset();
    pitch_speed_pid.reset();
    yaw_speed_pid.reset();
}


void flight_control_task(void *pvParameters)
{
    (void)pvParameters;

    //--------------------Motors-------------------------//
    // first, so the ESCs get the stop pulse straight away
    Motors::Config motor_config;
    motor_config.pins[0] = MOTOR0_PIN;
    motor_config.pins[1] = MOTOR1_PIN;
    motor_config.pins[2] = MOTOR2_PIN;
    motor_config.pins[3] = MOTOR3_PIN;
    motor_config.count = 4;
    static Motors motors(motor_config);

    if (!motors.begin())
    {
        DEBUG_PRINTF("Motors error: %s\n", motors.lastError());
        vTaskDelete(NULL);
    }
    uint32_t motors_start_ms = to_ms_since_boot(get_absolute_time());
    DEBUG_PRINTF("Motors initialized\n");

    //--------------------BNO085-------------------------//
    spi_init(spi1, SPI1_BUADRATE);
    spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI1_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_MISO_PIN, GPIO_FUNC_SPI);

    BNO08x bno(spi1, BNO08x_CS_PIN, BNO08x_INT_PIN, BNO08x_RESET_PIN);

    if (!bno.initialize())
    {
        // motors stay stopped, never arms
        DEBUG_PRINTF("Failed to find BNO08x, no flight control\n");
        while (true)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    DEBUG_PRINTF("BNO08x found\n");

    bno.setDcdAutoSave(false);
    enable_flight_reports(bno);

    gpio_set_irq_enabled_with_callback(BNO08x_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &bno_int_isr);

    // ========================================================//

    sh2_SensorValue_t value;

    float qr = 1.0f, qi = 0.0f, qj = 0.0f, qk = 0.0f;
    float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f; // rad/s, BNO axes
    float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
    float roll_speed = 0.0f, pitch_speed = 0.0f, yaw_speed = 0.0f; // deg/s
    float compass_deg = 0.0f;

    uint32_t last_attitude_us = 0, last_gyro_us = 0, last_control_us = 0;
    bool have_attitude = false, have_gyro = false;

    RcInput rc = {};
    bool have_rc = false;
    bool armed = false;
    bool arm_switch_was_off = false; // switch has to be seen OFF before arming
    bool motors_at_limit = false;
    uint8_t disarm_reason = DISARM_NONE;

    float target_roll = 0.0f, target_pitch = 0.0f, target_yaw_speed = 0.0f;
    float motor_power[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    uint32_t loop_count = 0, loop_hz = 0, last_hz_ms = 0, last_status_ms = 0;


    while (true)
    {
        // short wait so the safety checks still run if the IMU stops
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));

        if (bno.wasReset())
        {
            DEBUG_PRINTF("Sensor was reset\n");
            enable_flight_reports(bno);
        }

        bool new_attitude = false;
        while (bno.getSensorEvent(&value))
        {
            if (value.sensorId == SH2_GAME_ROTATION_VECTOR)
            {
                qr = value.un.gameRotationVector.real;
                qi = value.un.gameRotationVector.i;
                qj = value.un.gameRotationVector.j;
                qk = value.un.gameRotationVector.k;
                new_attitude = true;
            }
            else if (value.sensorId == SH2_GYROSCOPE_CALIBRATED)
            {
                gyro_x = value.un.gyroscope.x;
                gyro_y = value.un.gyroscope.y;
                gyro_z = value.un.gyroscope.z;
                last_gyro_us = time_us_32();
                have_gyro = true;
            }
            else if (value.sensorId == SH2_ROTATION_VECTOR)
            {
                // compass heading, print only
                float mr = value.un.rotationVector.real;
                float mi = value.un.rotationVector.i;
                float mj = value.un.rotationVector.j;
                float mk = value.un.rotationVector.k;
                float mag_yaw = atan2f(2.0f * (mr * mk + mi * mj),
                                       1.0f - 2.0f * (mj * mj + mk * mk)) * RAD2DEG;
                compass_deg = 90.0f - mag_yaw;
                if (compass_deg < 0.0f) compass_deg += 360.0f;
                if (compass_deg >= 360.0f) compass_deg -= 360.0f;
            }
        }

        uint32_t now_us = time_us_32();

        if (new_attitude)
        {
            float sinp = 2.0f * (qr * qj - qk * qi);
            if (sinp > 1.0f) sinp = 1.0f; // guard against NaN
            if (sinp < -1.0f) sinp = -1.0f;

            float bno_roll = atan2f(2.0f * (qr * qi + qj * qk),
                                     1.0f - 2.0f * (qi * qi + qj * qj)) * RAD2DEG;
            float bno_pitch = asinf(sinp) * RAD2DEG;
            float bno_yaw = atan2f(2.0f * (qr * qk + qi * qj),
                                     1.0f - 2.0f * (qj * qj + qk * qk)) * RAD2DEG;

            roll_deg = IMU_ROLL_DIRECTION * bno_roll - ROLL_TRIM_DEG;
            pitch_deg = IMU_PITCH_DIRECTION * bno_pitch - PITCH_TRIM_DEG;
            yaw_deg = IMU_YAW_DIRECTION * bno_yaw;

            last_attitude_us = now_us;
            have_attitude = true;
            loop_count++;
        }

        roll_speed = IMU_ROLL_DIRECTION * gyro_x * RAD2DEG;
        pitch_speed = IMU_PITCH_DIRECTION * gyro_y * RAD2DEG;
        yaw_speed = IMU_YAW_DIRECTION * gyro_z * RAD2DEG;

        //--------------------RC-------------------------//
        if (xQueuePeek(rc_mailbox, &rc, 0) == pdTRUE) have_rc = true;

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        bool link_ok = have_rc && (now_ms - rc.time_ms) < FAILSAFE_MS;
        bool arm_switch = link_ok && rc.ch[CH_ARM] > CH_MID;
        float throttle = link_ok ? stick_to_throttle(rc.ch[CH_THROTTLE]) : 0.0f;

        if (link_ok && rc.ch[CH_ARM] <= CH_MID) arm_switch_was_off = true;

        bool imu_ok = have_attitude && have_gyro &&
                      (now_us - last_attitude_us) < IMU_TIMEOUT_US &&
                      (now_us - last_gyro_us) < IMU_TIMEOUT_US;

        //--------------------Arm / Disarm-------------------------//
        const char *arm_blocked_by = nullptr;

        if (!armed)
        {
            if (now_ms - motors_start_ms < ESC_STARTUP_MS)
            {
                arm_blocked_by = "ESCs starting";
            }
            else if (!link_ok)
            {
                arm_blocked_by = "no RC link";
            }
            else if (!imu_ok)
            {
                arm_blocked_by = "IMU not ready";
            }
            else if (!arm_switch_was_off)
            {
                arm_blocked_by = "arm switch OFF first";
            }
            else if (throttle > ARM_MAX_THROTTLE)
            {
                arm_blocked_by = "throttle not down";
            }
            else if (fabsf(roll_deg) > ARM_MAX_TILT_DEG || fabsf(pitch_deg) > ARM_MAX_TILT_DEG)
            {
                arm_blocked_by = "not level";
            }

            if (arm_switch && arm_blocked_by == nullptr)
            {
                reset_all_pids();
                motors.arm();
                armed = true;
                disarm_reason = DISARM_NONE;
                last_control_us = now_us;
            }
        }
        else
        {
            uint8_t reason = DISARM_NONE;
            if (!link_ok)
            {
                reason = DISARM_FAILSAFE;
            }
            else if (!arm_switch)
            {
                reason = DISARM_SWITCH;
            }
            else if (!imu_ok)
            {
                reason = DISARM_IMU_TIMEOUT;
            }
            else if (fabsf(roll_deg) > CRASH_TILT_DEG || fabsf(pitch_deg) > CRASH_TILT_DEG)
            {
                reason = DISARM_CRASH_TILT;
            }

            if (reason != DISARM_NONE)
            {
                motors.disarm();
                armed = false;
                disarm_reason = reason;
                arm_switch_was_off = false; // switch has to go OFF again
                reset_all_pids();
                for (int i = 0; i < 4; i++) motor_power[i] = 0.0f;
            }
        }

        //--------------------Control-------------------------//
        // once per new attitude
        if (armed && new_attitude)
        {
            float time_step = (now_us - last_control_us) * 1e-6f;
            last_control_us = now_us;
            if (time_step < 0.0005f) time_step = 0.0005f;
            if (time_step > 0.02f) time_step = 0.02f;

            target_roll = ROLL_STICK_DIRECTION * stick_position(rc.ch[CH_ROLL]) * MAX_TILT_ANGLE_DEG;
            target_pitch = -PITCH_STICK_DIRECTION * stick_position(rc.ch[CH_PITCH]) * MAX_TILT_ANGLE_DEG; // forward = nose down
            target_yaw_speed = YAW_STICK_DIRECTION * stick_position(rc.ch[CH_YAW]) * MAX_YAW_SPEED_DEG_S;

            if (throttle < PID_START_THROTTLE)
            {
                // throttle down, all motors idle
                reset_all_pids();
                motors_at_limit = false;
                for (int i = 0; i < 4; i++) motor_power[i] = 0.0f;
            }
            else
            {
                // no I on the ground or when a motor is at its limit
                bool freeze_integral = (throttle < INTEGRAL_START_THROTTLE) || motors_at_limit;
                roll_speed_pid.hold_integral(freeze_integral);
                pitch_speed_pid.hold_integral(freeze_integral);
                yaw_speed_pid.hold_integral(freeze_integral);

                // angle -> wanted tilt speed
                float target_roll_speed = roll_angle_pid.calculate(target_roll, roll_deg, time_step);
                float target_pitch_speed = pitch_angle_pid.calculate(target_pitch, pitch_deg, time_step);

                // tilt speed -> motor correction
                float roll_correction = roll_speed_pid.calculate(target_roll_speed, roll_speed, time_step);
                float pitch_correction = pitch_speed_pid.calculate(target_pitch_speed, pitch_speed, time_step);
                float yaw_correction = yaw_speed_pid.calculate(target_yaw_speed, yaw_speed, time_step);

                float highest = -1e9f, lowest = 1e9f;
                for (int i = 0; i < 4; i++)
                {
                    motor_power[i] = throttle;
                    motor_power[i] += roll_correction * motor_mix[i].roll;
                    motor_power[i] += pitch_correction * motor_mix[i].pitch;
                    motor_power[i] += yaw_correction * motor_mix[i].yaw;
                    if (motor_power[i] > highest) highest = motor_power[i];
                    if (motor_power[i] < lowest) lowest = motor_power[i];
                }

                // move all motors together to keep the corrections, then clamp
                motors_at_limit = false;
                if (highest > 1.0f)
                {
                    for (int i = 0; i < 4; i++) motor_power[i] -= (highest - 1.0f);
                    lowest -= (highest - 1.0f);
                    motors_at_limit = true;
                }
                if (lowest < 0.0f)
                {
                    for (int i = 0; i < 4; i++) motor_power[i] -= lowest;
                    motors_at_limit = true;
                }
                for (int i = 0; i < 4; i++)
                {
                    if (motor_power[i] > 1.0f) motor_power[i] = 1.0f;
                    if (motor_power[i] < 0.0f) motor_power[i] = 0.0f;
                }
            }
            motors.setThrottles(motor_power);
        }
        else if (!armed)
        {
            motors.setAll(0.0f); // stop pulse
            target_roll = 0.0f;
            target_pitch = 0.0f;
            target_yaw_speed = 0.0f;
        }

        //--------------------Status-------------------------//
        if (now_ms - last_hz_ms >= 1000)
        {
            loop_hz = loop_count;
            loop_count = 0;
            last_hz_ms = now_ms;
        }

        if (now_ms - last_status_ms >= 20) // 50 Hz
        {
            last_status_ms = now_ms;

            FlightStatus status;
            status.armed = armed;
            status.link_ok = link_ok;
            status.disarm_reason = disarm_reason;
            status.arm_blocked_by = arm_blocked_by;
            status.throttle = throttle;
            status.roll_deg = roll_deg;
            status.pitch_deg = pitch_deg;
            status.yaw_deg = yaw_deg;
            status.roll_speed = roll_speed;
            status.pitch_speed = pitch_speed;
            status.yaw_speed = yaw_speed;
            status.target_roll = target_roll;
            status.target_pitch = target_pitch;
            status.target_yaw_speed = target_yaw_speed;
            for (int i = 0; i < 4; i++) status.motor_power[i] = armed ? motor_power[i] : 0.0f;
            status.compass_deg = compass_deg;
            status.loop_hz = (uint16_t)loop_hz;
            status.time_ms = now_ms;

            xQueueOverwrite(status_mailbox, &status);
        }
    }
}


// ========================================================
// Flight status print (core 0)
// printf on UART is slow, kept out of the flight loop
// ========================================================
void flight_telemetry_task(void *pvParameters)
{
    (void)pvParameters;

    FlightStatus status;
    bool was_armed = false;
    uint8_t last_reason = DISARM_NONE;
    uint32_t last_print_ms = 0;

    while (true)
    {
        if (xQueuePeek(status_mailbox, &status, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            if (status.armed && !was_armed)
            {
                printf(">>> ARMED\n");
            }
            else if (!status.armed && was_armed)
            {
                printf(">>> DISARMED: %s\n", disarm_reason_text(status.disarm_reason));
            }
            else if (!status.armed && status.disarm_reason != last_reason &&
                     status.disarm_reason != DISARM_NONE)
            {
                printf(">>> DISARMED: %s\n", disarm_reason_text(status.disarm_reason));
            }
            was_armed = status.armed;
            last_reason = status.disarm_reason;

            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            if (now_ms - last_print_ms >= 250) // 4 Hz
            {
                last_print_ms = now_ms;
                printf("%s link:%d thr:%.2f | R %6.1f sp %6.1f | P %6.1f sp %6.1f | "
                       "Yr %6.0f sp %6.0f | M %.2f %.2f %.2f %.2f | %u Hz",
                       status.armed ? "ARMED" : "disarmed", status.link_ok, status.throttle,
                       status.roll_deg, status.target_roll, status.pitch_deg, status.target_pitch,
                       status.yaw_speed, status.target_yaw_speed,
                       status.motor_power[0], status.motor_power[1],
                       status.motor_power[2], status.motor_power[3],
                       status.loop_hz);
                if (!status.armed && status.arm_blocked_by) printf(" | no arm: %s", status.arm_blocked_by);
                printf("\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
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
    status_mailbox = xQueueCreate(1, sizeof(FlightStatus));

    xTaskCreate(core0_system_task, "TempTask", 256, NULL, 1, &core0_task_handle);

    //xTaskCreate(core1_flight_task, "ClockTask", 256, NULL, 1, &core1_task_handle);

    xTaskCreate(core0_elrs_task, "ELRSTask", 256, NULL, 1, &elrs_task_handle);
    //xTaskCreate(bno085_task, "ImuTask", 1024, NULL, 1, &bno085_task_handle);
    //xTaskCreate(bno085_imu_task, "BnoTask", 1024, NULL, 1, &bno085_task_handle);
    //xTaskCreate(bmp581_task,"BMPTask",1024,NULL,1,&bmp581_task_handle);
    //xTaskCreate(motor_task, "MotorTest", 1024, NULL, 1, &motor_test_handle);

    // flight task uses bno085_task_handle so bno_int_isr wakes it
    xTaskCreate(flight_control_task, "Flight", 2048, NULL, 3, &bno085_task_handle);
    xTaskCreate(flight_telemetry_task, "Telemetry", 1024, NULL, 1, &core1_task_handle);

    vTaskCoreAffinitySet(elrs_task_handle, (1 << 0));
    //vTaskCoreAffinitySet(core0_task_handle, (1 << 0));
    vTaskCoreAffinitySet(core1_task_handle, (1 << 0)); // flight print
    vTaskCoreAffinitySet(bno085_task_handle, (1 << 1)); // flight control
    //vTaskCoreAffinitySet(bmp581_task_handle, (1 << 1));
    //vTaskCoreAffinitySet(motor_test_handle, (1 << 1));

    
    vTaskStartScheduler();

    while (true) {
        //tight_loop_contents();
    }
    return 0;
}


 
