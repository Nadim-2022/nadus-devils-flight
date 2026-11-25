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
#include "sensors/Mtf01p.hpp"

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

// MTF-01P optical flow, both UARTs taken so it's read with PIO
// only the sensor Tx is wired, the sensor Rx is not used
#define FLOW_SENSOR_RX_PIN 6 // sensor Tx -> GP6
#define FLOW_SENSOR_BAUD 115200


TaskHandle_t elrs_task_handle = NULL;
TaskHandle_t core0_task_handle = NULL;
TaskHandle_t core1_task_handle = NULL;
TaskHandle_t bmp581_task_handle = NULL;
TaskHandle_t bno085_task_handle = NULL;
TaskHandle_t motor_test_handle = NULL;
TaskHandle_t optical_flow_task_handle = NULL;


static constexpr float RAD2DEG = 57.2957795f;



struct RcInput {
    uint16_t ch[16];     // channel values from ELRS
    uint32_t time_ms;    // when they were received (for failsafe)
};

static QueueHandle_t rc_mailbox;
static QueueHandle_t flow_mailbox;
static volatile bool flow_print_on = false; // "flow" command, raw sensor print
static volatile bool channels_print_on = false; // "channels" command, all RC channels

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
    CMD_LOG, // "log"
    CMD_ESC, // "esc"
    CMD_LEVEL, // "level"
    CMD_FLOW, // "flow"
    CMD_CHANNELS, // "channels"
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
            if (strcmp(line, "log") == 0) return CMD_LOG;
            if (strcmp(line, "esc") == 0) return CMD_ESC;
            if (strcmp(line, "level") == 0) return CMD_LEVEL;
            if (strcmp(line, "flow") == 0) return CMD_FLOW;
            if (strcmp(line, "channels") == 0) return CMD_CHANNELS;
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
    uint32_t last_channels_print_ms = 0;

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

            // "channels": flip a switch and see which number changes
            if (channels_print_on && rc.time_ms - last_channels_print_ms >= 200)
            {
                last_channels_print_ms = rc.time_ms;
                printf("RC");
                for (int i = 0; i < 12; i++)
                {
                    printf(" CH%d %4d", i + 1, rc.ch[i]);
                }
                printf("\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}




// ========================================================
// MTF-01P optical flow + range (core 0)
// newest reading goes to flow_mailbox
// "flow" on the serial console prints the raw readings at 10 Hz
// ========================================================
void optical_flow_task(void *pvParameters)
{
    (void)pvParameters;

    Sensors::Mtf01p flow_sensor(FLOW_SENSOR_RX_PIN, FLOW_SENSOR_BAUD);

    if (!flow_sensor.begin())
    {
        DEBUG_PRINTF("MTF-01P not started, flying without it\n");
        vTaskDelete(NULL);
    }

    uint32_t last_print_ms = 0;

    while (true)
    {
        if (flow_sensor.read_frame())
        {
            xQueueOverwrite(flow_mailbox, &flow_sensor.reading);
        }

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (flow_print_on && now_ms - last_print_ms >= 100)
        {
            last_print_ms = now_ms;
            const Sensors::OpticalFlowReading &reading = flow_sensor.reading;
            uint32_t age_ms = reading.frame_count > 0 ? now_ms - reading.received_ms : 0;
            printf("FLOW bytes %lu frames %lu age %lu ms | dist %lu mm ok %d status %u strength %u precision %u | "
                   "flow x %+.2f y %+.2f ok %d status %u quality %u | bad frames %lu lost bytes %lu\n",
                   flow_sensor.get_received_bytes(), reading.frame_count, age_ms,
                   reading.distance_mm, reading.distance_ok, reading.distance_status,
                   reading.strength, reading.precision,
                   reading.flow_x, reading.flow_y, reading.flow_ok, reading.flow_status, reading.flow_quality,
                   flow_sensor.get_bad_frames(), flow_sensor.get_lost_bytes());
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
// used at power up, "level" or the stick move below measures new ones
static constexpr float ROLL_TRIM_DEG = 0.7f;
static constexpr float PITCH_TRIM_DEG = 0.5f;

// Level on the floor without a cable plugged in (cable moves the soft mount):
// disarmed, throttle down, yaw full left + pitch full back for 1 s
static constexpr uint32_t LEVEL_STICK_HOLD_MS = 1000;
static constexpr uint32_t LEVEL_MEASURE_MS = 2000;
static constexpr float LEVEL_MAX_TILT_DEG = 10.0f; // more than this = not on the floor
static constexpr float LEVEL_MAX_MOVING_DEG_S = 5.0f; // drone has to sit still

static float roll_trim_deg = ROLL_TRIM_DEG;
static float pitch_trim_deg = PITCH_TRIM_DEG;
static volatile bool level_requested = false; // set by the "level" command


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
static constexpr float MAX_TILT_ANGLE_DEG = 25.0f; // full stickSticks, disarmed
static constexpr float MAX_TILT_SPEED_DEG_S = 150.0f; // max output of the angle loop
static constexpr float MAX_YAW_SPEED_DEG_S = 120.0f; // full yaw stick
static constexpr float PID_START_THROTTLE = 0.05f; // below this motors idle and PID off
static constexpr float INTEGRAL_START_THROTTLE = 0.30f; // below this I stays frozen (on the ground)
static constexpr float YAW_INTEGRAL_START_THROTTLE = 0.10f; // yaw can't wind up on the ground, the feet stop it turning

// MTF-01P optical flow
// Auto mode: auto switch on when arming = take off, hold height (laser) and position (flow)
// auto switch middle or down when arming = manual throttle like before
// auto switch leaves up in the air = lands by itself
// find a switch channel with "channels" (CH number - 1 here)
#define CH_AUTO 6 // CH7 = index 6, 3 position switch: down 1000, middle 1503, up 2000
#define CH_AUTO_ON 1800 // above this = switch up = auto
static constexpr bool USE_FLOW_HOLD = true; // false = auto only holds height, no position hold
static constexpr uint32_t FLOW_TIMEOUT_MS = 100;
static constexpr uint8_t FLOW_MIN_QUALITY = 50; // quality range not in the docs, check on the bench
static constexpr float FLOW_MIN_HEIGHT_M = 0.50f; // reads about 2x too much at 20 cm, right at 80 cm
static constexpr float FLOW_MAX_HEIGHT_M = 5.0f;
static constexpr float FLOW_MAX_TILT_DEG = 30.0f; // flow not used when tilted more
static constexpr float FLOW_SPEED_FILTER = 0.1f; // low pass on the flow speed
static constexpr bool FLOW_REMOVE_ROTATION = true; // take the gyro out of the flow

// MTF-01P mounted under the frame looking down
// sensor X = forward, Y = right (axes test 2026-11-24)
static constexpr bool FLOW_SWAP_AXES = false; // true = sensor Y is forward
static constexpr float FLOW_FORWARD_DIRECTION = +1.0f;
static constexpr float FLOW_RIGHT_DIRECTION = +1.0f;

// Position hold
static constexpr float POSITION_GAIN = 0.7f; // m/s back per m away from the hold point (1.0 overshot after a gust)
static constexpr float MAX_HOLD_SPEED_M_S = 0.3f; // comes back slow, less overshoot
static constexpr float MAX_HOLD_TILT_DEG = 8.0f;
static constexpr float BRAKE_DONE_SPEED_M_S = 0.3f; // sticks let go, hold point set below this speed

// Auto height, laser only (no BMP581)
static constexpr float TAKEOFF_HEIGHT_M = 0.8f; // holds here, laser checked at 0.8 m
static constexpr float CLIMB_SPEED_M_S = 0.3f; // up to TAKEOFF_HEIGHT_M
static constexpr float LANDING_SPEED_M_S = 0.2f;
static constexpr float HEIGHT_GAIN = 1.0f; // m/s of climb speed per m off the target height
static constexpr float MAX_CLIMB_SPEED_M_S = 0.3f;
static constexpr float AUTO_MAX_HEIGHT_M = 2.0f; // higher than this = land
static constexpr uint32_t RANGE_TIMEOUT_MS = 200; // no good laser this long = height lost
static constexpr float RANGE_HEIGHT_FILTER = 0.3f; // low pass on the laser height
static constexpr float CLIMB_SPEED_FILTER = 0.1f; // low pass on the climb speed
static constexpr float MAX_MEASURED_CLIMB_SPEED_M_S = 1.0f; // climb speed from the laser can't be more
// laser gave 0.23 -> 0.92 -> 1.8 m in 0.1 s at lift off (flight 2026-11-24), the drone can't do that
static constexpr float RANGE_JUMP_M = 0.15f; // bigger jump in one reading = ignored
static constexpr int RANGE_JUMP_READINGS = 5; // unless this many jumps in a row agree (floor moved)

// Take off
static constexpr uint32_t SPIN_UP_MS = 1500; // props at idle after arming
static constexpr float TAKEOFF_START_THROTTLE = 0.25f;
static constexpr float HOVER_THROTTLE_GUESS = 0.40f; // hover in the flights 2026-11-24, I learns the rest
static constexpr float TAKEOFF_THROTTLE_RAMP = 0.10f; // throttle added per second until it lifts
static constexpr float TAKEOFF_MAX_THROTTLE = 0.70f;
static constexpr uint32_t NO_LIFTOFF_MS = 2000; // at max take off throttle this long = give up
static constexpr float LIFTOFF_HEIGHT_M = 0.08f; // 0.15 let the ramp go to 0.46 and it jumped
static constexpr float LIFTOFF_CLIMB_SPEED_M_S = 0.10f;
static constexpr int LIFTOFF_READINGS = 3; // laser readings in a row that say it lifted

// Thrust limits for the height control
static constexpr float MAX_THRUST_CHANGE = 0.15f; // up from the hover throttle
static constexpr float MAX_THRUST_DROP = 0.08f; // down from the learned hover throttle (hover + I), more = falls
static constexpr float LIFTOFF_THRUST_DROP = 0.02f; // first moments after lift off, so it can't sink back
static constexpr uint32_t LIFTOFF_GENTLE_MS = 1000;
static constexpr float MIN_FLYING_THROTTLE = 0.10f;
static constexpr float MAX_FLYING_THROTTLE = 0.80f;
static constexpr float MIN_TILT_FACTOR = 0.70f; // tilt boost stops at about 45 deg

// Landing
static constexpr uint32_t ON_GROUND_MS = 1000; // wants down but not moving this long = landed
static constexpr float ON_GROUND_CLIMB_SPEED_M_S = 0.10f;
static constexpr float LANDED_HEIGHT_M = 0.10f; // landing and lower than this ON_GROUND_MS = landed
static constexpr uint32_t MAX_LANDING_MS = 15000; // landing takes longer = disarm
static constexpr float BLIND_DROP = 0.04f; // under hover throttle when the laser is lost

// Safety
static constexpr float ARM_MAX_TILT_DEG = 20.0f;
static constexpr float CRASH_TILT_DEG = 60.0f;
static constexpr float ARM_MAX_THROTTLE = 0.05f;
static constexpr uint32_t IMU_TIMEOUT_US = 100000; // 100 ms
static constexpr float MAX_ATTITUDE_JUMP_DEG = 10.0f; // roll / pitch can't move this much between two readings
static constexpr float GYRO_FILTER_HZ = 40.0f; // low pass on the gyro, takes out the vibration
static constexpr uint32_t GYRO_ONLY_MAX_US = 5000000; // 5 s flying on gyro angles when the BNO angle is lost
// after that it lands slowly on the gyro (auto or manual), on the ground it disarms
static constexpr float MANUAL_FLYING_THROTTLE = 0.25f; // manual above this = in the air
static constexpr uint32_t ESC_STARTUP_MS = 4000;


// PID gains, first guesses, not tuned yet
// angle loop : in deg, out deg/s
// speed loop : in deg/s, out motor power (0..1)
static Pid::Settings angle_pid_settings()
{
    Pid::Settings settings;
    settings.proportional_gain = 6.0f;
    settings.output_limit = MAX_TILT_SPEED_DEG_S;
    return settings;
}

static Pid::Settings roll_pitch_speed_pid_settings()
{
    Pid::Settings settings;
    settings.proportional_gain = 0.0010f;
    settings.integral_gain = 0.0015f;
    settings.derivative_gain = 0.0f;//0.00002f;
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


// flow speed loop : in m/s, out tilt angle deg
static Pid::Settings flow_speed_pid_settings()
{
    Pid::Settings settings;
    settings.proportional_gain = 5.0f;
    settings.integral_gain = 0.3f;
    settings.derivative_gain = 0.0f;
    settings.integral_limit = 5.0f; // deg of steady lean it can learn, 2 was too little against a steady push
    settings.output_limit = MAX_HOLD_TILT_DEG;
    return settings;
}


// height loop : in m, out climb speed m/s
static Pid::Settings height_pid_settings()
{
    Pid::Settings settings;
    settings.proportional_gain = HEIGHT_GAIN;
    settings.output_limit = MAX_CLIMB_SPEED_M_S;
    return settings;
}

// climb speed loop : in m/s, out throttle change from hover
// I learns the real hover throttle
static Pid::Settings climb_speed_pid_settings()
{
    Pid::Settings settings;
    settings.proportional_gain = 0.10f;
    settings.integral_gain = 0.10f;
    settings.derivative_gain = 0.0f;
    settings.integral_limit = MAX_THRUST_CHANGE;
    settings.output_limit = MAX_THRUST_CHANGE;
    return settings;
}


enum AutoPhase : uint8_t
{
    AUTO_OFF = 0, // manual throttle
    AUTO_SPIN_UP,
    AUTO_TAKEOFF,
    AUTO_HOLD,
    AUTO_LANDING,
};

static const char *auto_phase_text(uint8_t phase)
{
    switch (phase)
    {
        case AUTO_SPIN_UP: return "SPIN UP";
        case AUTO_TAKEOFF: return "TAKEOFF";
        case AUTO_HOLD: return "HOLD";
        case AUTO_LANDING: return "LANDING";
        default: return "MANUAL";
    }
}

enum LandReason : uint8_t
{
    LAND_NONE = 0,
    LAND_SWITCH,
    LAND_RC_LOST,
    LAND_TOO_HIGH,
    LAND_IMU_LOST,
};

static const char *land_reason_text(uint8_t reason)
{
    switch (reason)
    {
        case LAND_SWITCH: return "auto switch off";
        case LAND_RC_LOST: return "RC link lost";
        case LAND_TOO_HIGH: return "too high";
        case LAND_IMU_LOST: return "IMU angle lost, landing on the gyro";
        default: return "-";
    }
}

enum DisarmReason : uint8_t
{
    DISARM_NONE = 0,
    DISARM_SWITCH,
    DISARM_FAILSAFE,
    DISARM_IMU_TIMEOUT,
    DISARM_CRASH_TILT,
    DISARM_IMU_LOST,
    DISARM_LANDED,
    DISARM_NO_LIFTOFF,
    DISARM_LANDING_TIMEOUT,
    DISARM_AUTO_OFF_ON_GROUND,
};

static const char *disarm_reason_text(uint8_t reason)
{
    switch (reason)
    {
        case DISARM_SWITCH: return "arm switch off";
        case DISARM_FAILSAFE: return "FAILSAFE: RC link lost";
        case DISARM_IMU_TIMEOUT: return "IMU stopped sending";
        case DISARM_CRASH_TILT: return "CRASH: tilt over limit";
        case DISARM_IMU_LOST: return "IMU angle lost too long";
        case DISARM_LANDED: return "landed";
        case DISARM_NO_LIFTOFF: return "no lift off at max take off throttle";
        case DISARM_LANDING_TIMEOUT: return "landing took too long";
        case DISARM_AUTO_OFF_ON_GROUND: return "auto switch off before lift off";
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
    float roll_trim, pitch_trim;
    bool levelling;
    uint8_t imu_resets; // BNO085 restarted by itself
    uint16_t bad_imu_reads; // rotation vectors thrown away
    bool gyro_only; // flying on gyro angles, BNO angle not trusted
    float bno_roll, bno_pitch; // what the BNO said, also when thrown away
    bool flow_ok; // flow good enough to use
    bool flow_hold; // position hold flying
    uint8_t flow_quality;
    float flow_distance; // m, straight from the sensor
    float forward_speed, right_speed; // m/s from the flow
    uint8_t auto_phase;
    uint8_t land_reason;
    bool range_ok;
    float height, target_height; // m, laser
    float climb_speed, target_climb_speed; // m/s
    float hover_throttle;
    float position_x, position_y; // m from the flow, turned with the yaw
    float hold_x, hold_y; // m, where position hold wants to be
};

static QueueHandle_t status_mailbox;


//--------------------Flight log-------------------------//
// Last 60 s of flying at 50 Hz, kept in RAM, while armed and 3 s after disarm.
// Type "log" on the serial console after landing to print it.
// Saved as int16 (x10 or x1000) so it only takes about 240 KB.
static constexpr int FLIGHT_LOG_SIZE = 3000; // 60 s at 50 Hz
static constexpr uint32_t LOG_AFTER_DISARM_MS = 3000; // to see the angles settle after landing

struct FlightLogLine
{
    uint32_t time_ms;
    int16_t throttle; // x1000
    int16_t motor_power[4]; // x1000
    int16_t yaw_speed; // deg/s x10
    int16_t target_yaw_speed; // deg/s x10
    int16_t roll_deg; // x10
    int16_t pitch_deg; // x10
    int16_t yaw_deg; // x10
    int16_t roll_speed; // deg/s x10
    int16_t pitch_speed; // deg/s x10
    uint8_t imu_resets;
    uint16_t bad_imu_reads;
    uint8_t gyro_only;
    int16_t bno_roll; // x10
    int16_t bno_pitch; // x10
    uint8_t flow_ok;
    uint8_t flow_hold;
    uint8_t flow_quality;
    int16_t flow_distance; // m x1000
    int16_t forward_speed; // m/s x1000
    int16_t right_speed; // m/s x1000
    uint8_t auto_phase;
    uint8_t range_ok;
    int16_t height; // m x1000
    int16_t target_height; // m x1000
    int16_t climb_speed; // m/s x1000
    int16_t target_climb_speed; // m/s x1000
    int16_t hover_throttle; // x1000
    uint8_t land_reason;
    int16_t target_roll; // x10
    int16_t target_pitch; // x10
    int16_t position_x; // m x1000
    int16_t position_y; // m x1000
    int16_t hold_x; // m x1000
    int16_t hold_y; // m x1000
};

static FlightLogLine flight_log[FLIGHT_LOG_SIZE];
static volatile int flight_log_next = 0; // where the next line goes
static volatile int flight_log_count = 0; // filled lines, stops at FLIGHT_LOG_SIZE

static int16_t to_log_number(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 32767.0f) scaled = 32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    return (int16_t)lroundf(scaled);
}

// called from the flight task, oldest line gets overwritten when full
static void flight_log_add(const FlightStatus &status)
{
    FlightLogLine &line = flight_log[flight_log_next];
    line.time_ms = status.time_ms;
    line.throttle = to_log_number(status.throttle, 1000.0f);
    for (int i = 0; i < 4; i++) line.motor_power[i] = to_log_number(status.motor_power[i], 1000.0f);
    line.yaw_speed = to_log_number(status.yaw_speed, 10.0f);
    line.target_yaw_speed = to_log_number(status.target_yaw_speed, 10.0f);
    line.roll_deg = to_log_number(status.roll_deg, 10.0f);
    line.pitch_deg = to_log_number(status.pitch_deg, 10.0f);
    line.yaw_deg = to_log_number(status.yaw_deg, 10.0f);
    line.roll_speed = to_log_number(status.roll_speed, 10.0f);
    line.pitch_speed = to_log_number(status.pitch_speed, 10.0f);
    line.imu_resets = status.imu_resets;
    line.bad_imu_reads = status.bad_imu_reads;
    line.gyro_only = status.gyro_only;
    line.bno_roll = to_log_number(status.bno_roll, 10.0f);
    line.bno_pitch = to_log_number(status.bno_pitch, 10.0f);
    line.flow_ok = status.flow_ok;
    line.flow_hold = status.flow_hold;
    line.flow_quality = status.flow_quality;
    line.flow_distance = to_log_number(status.flow_distance, 1000.0f);
    line.forward_speed = to_log_number(status.forward_speed, 1000.0f);
    line.right_speed = to_log_number(status.right_speed, 1000.0f);
    line.auto_phase = status.auto_phase;
    line.range_ok = status.range_ok;
    line.height = to_log_number(status.height, 1000.0f);
    line.target_height = to_log_number(status.target_height, 1000.0f);
    line.climb_speed = to_log_number(status.climb_speed, 1000.0f);
    line.target_climb_speed = to_log_number(status.target_climb_speed, 1000.0f);
    line.hover_throttle = to_log_number(status.hover_throttle, 1000.0f);
    line.land_reason = status.land_reason;
    line.target_roll = to_log_number(status.target_roll, 10.0f);
    line.target_pitch = to_log_number(status.target_pitch, 10.0f);
    line.position_x = to_log_number(status.position_x, 1000.0f);
    line.position_y = to_log_number(status.position_y, 1000.0f);
    line.hold_x = to_log_number(status.hold_x, 1000.0f);
    line.hold_y = to_log_number(status.hold_y, 1000.0f);

    flight_log_next = (flight_log_next + 1) % FLIGHT_LOG_SIZE;
    if (flight_log_count < FLIGHT_LOG_SIZE) flight_log_count++;
}

// oldest line first, time counts from the first line
// a jump in time = disarmed and armed again
static void flight_log_print()
{
    int count = flight_log_count;
    int first = (flight_log_next - count + FLIGHT_LOG_SIZE) % FLIGHT_LOG_SIZE;
    uint32_t start_ms = flight_log[first].time_ms;

    printf("=== FLIGHT LOG START (%d lines) ===\n", count);
    printf("time_s,throttle,m0_rear_right,m1_front_right,m2_rear_left,m3_front_left,"
           "yaw_speed,yaw_target,roll,pitch,yaw,roll_speed,pitch_speed,imu_resets,bad_imu_reads,gyro_only,bno_roll,bno_pitch,"
           "flow_ok,flow_hold,flow_quality,flow_distance,forward_speed,right_speed,"
           "auto_phase,range_ok,height,target_height,climb_speed,target_climb_speed,hover_throttle,land_reason,"
           "target_roll,target_pitch,position_x,position_y,hold_x,hold_y\n");
    for (int i = 0; i < count; i++)
    {
        const FlightLogLine &line = flight_log[(first + i) % FLIGHT_LOG_SIZE];
        printf("%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u,%u,%.1f,%.1f,%u,%u,%u,%.3f,%.3f,%.3f,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f\n",
               (line.time_ms - start_ms) / 1000.0f,
               line.throttle / 1000.0f,
               line.motor_power[0] / 1000.0f, line.motor_power[1] / 1000.0f,
               line.motor_power[2] / 1000.0f, line.motor_power[3] / 1000.0f,
               line.yaw_speed / 10.0f, line.target_yaw_speed / 10.0f,
               line.roll_deg / 10.0f, line.pitch_deg / 10.0f, line.yaw_deg / 10.0f,
               line.roll_speed / 10.0f, line.pitch_speed / 10.0f,
               line.imu_resets, line.bad_imu_reads, line.gyro_only,
               line.bno_roll / 10.0f, line.bno_pitch / 10.0f,
               line.flow_ok, line.flow_hold, line.flow_quality,
               line.flow_distance / 1000.0f, line.forward_speed / 1000.0f, line.right_speed / 1000.0f,
               line.auto_phase, line.range_ok,
               line.height / 1000.0f, line.target_height / 1000.0f, line.climb_speed / 1000.0f,
               line.target_climb_speed / 1000.0f, line.hover_throttle / 1000.0f, line.land_reason,
               line.target_roll / 10.0f, line.target_pitch / 10.0f,
               line.position_x / 1000.0f, line.position_y / 1000.0f, line.hold_x / 1000.0f, line.hold_y / 1000.0f);
    }
    printf("=== FLIGHT LOG END ===\n");
}


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
static Pid forward_speed_pid(flow_speed_pid_settings());
static Pid right_speed_pid(flow_speed_pid_settings());
static Pid height_pid(height_pid_settings());
static Pid climb_speed_pid(climb_speed_pid_settings());

static void reset_all_pids()
{
    roll_angle_pid.reset();
    pitch_angle_pid.reset();
    roll_speed_pid.reset();
    pitch_speed_pid.reset();
    yaw_speed_pid.reset();
    forward_speed_pid.reset();
    right_speed_pid.reset();
    height_pid.reset();
    climb_speed_pid.reset();
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
    float untrimmed_roll = 0.0f, untrimmed_pitch = 0.0f;
    uint8_t imu_resets = 0;
    uint16_t bad_imu_reads = 0;
    float bno_roll_reading = 0.0f, bno_pitch_reading = 0.0f; // untrimmed
    uint64_t last_gyro_timestamp = 0;

    // BNO angle lost, angles come from the gyro for a while
    bool gyro_only = false;
    uint32_t gyro_only_since_us = 0;

    // level measuring
    bool levelling = false;
    uint32_t level_sticks_since_ms = 0; // 0 = sticks not in the level spot
    uint32_t level_start_ms = 0;
    float level_roll_sum = 0.0f, level_pitch_sum = 0.0f;
    int level_samples = 0;

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
    uint32_t last_armed_ms = 0;
    bool was_ever_armed = false;

    // MTF-01P, position x / y along the yaw it had when it started
    Sensors::OpticalFlowReading flow = {};
    bool have_flow = false;
    uint32_t last_flow_frame = 0;
    uint32_t last_flow_received_ms = 0;
    bool flow_usable = false;
    float forward_speed = 0.0f, right_speed = 0.0f; // m/s
    float position_x = 0.0f, position_y = 0.0f;
    float hold_x = 0.0f, hold_y = 0.0f;
    bool flow_holding = false;
    bool hold_point_set = false;

    // auto mode, height from the laser
    bool auto_mode = false;
    uint8_t auto_phase = AUTO_OFF;
    uint32_t phase_start_ms = 0;
    uint8_t land_reason = LAND_NONE;
    uint8_t control_disarm = DISARM_NONE; // set by the height control, done next loop
    bool have_range = false;
    uint32_t last_range_ms = 0;
    float last_range_height = 0.0f; // last laser height taken, not filtered
    int range_jumps_in_row = 0;
    float lowest_jump_height = 0.0f, highest_jump_height = 0.0f;
    int liftoff_readings = 0;
    bool imu_landing = false; // BNO angle lost too long, landing on the gyro
    float range_ground_distance = 0.0f; // laser on the ground when it armed
    float height = 0.0f, climb_speed = 0.0f;
    float target_height = 0.0f, target_climb_speed = 0.0f;
    float base_throttle = 0.0f; // all motors before roll / pitch / yaw
    float hover_throttle = 0.0f; // throttle when it lifted off
    float takeoff_throttle = 0.0f;
    bool at_max_takeoff_throttle = false;
    uint32_t max_takeoff_throttle_since_ms = 0;
    bool maybe_on_ground = false;
    uint32_t maybe_on_ground_since_ms = 0;


    while (true)
    {
        // short wait so the safety checks still run if the IMU stops
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));

        if (bno.wasReset())
        {
            DEBUG_PRINTF("Sensor was reset\n");
            enable_flight_reports(bno);
            if (imu_resets < 255) imu_resets++;
        }

        bool new_attitude = false;
        bool new_gyro = false;
        while (bno.getSensorEvent(&value))
        {
            if (value.sensorId == SH2_GAME_ROTATION_VECTOR)
            {
                float new_qr = value.un.gameRotationVector.real;
                float new_qi = value.un.gameRotationVector.i;
                float new_qj = value.un.gameRotationVector.j;
                float new_qk = value.un.gameRotationVector.k;

                // a good rotation vector has length 1, anything far off is a bad read
                float length = new_qr * new_qr + new_qi * new_qi + new_qj * new_qj + new_qk * new_qk;
                if (length > 0.9f && length < 1.1f)
                {
                    qr = new_qr;
                    qi = new_qi;
                    qj = new_qj;
                    qk = new_qk;
                    new_attitude = true;
                }
                else if (bad_imu_reads < 65535)
                {
                    bad_imu_reads++;
                }
            }
            else if (value.sensorId == SH2_GYROSCOPE_CALIBRATED)
            {
                // low pass, the drone moves slow, the vibration is fast
                float gyro_step = (float)(value.timestamp - last_gyro_timestamp) * 1e-6f;
                last_gyro_timestamp = value.timestamp;
                if (!have_gyro || gyro_step <= 0.0f || gyro_step > 0.02f) gyro_step = 0.0025f;
                float smoothing = gyro_step / (gyro_step + 1.0f / (2.0f * 3.14159265f * GYRO_FILTER_HZ));

                gyro_x += smoothing * (value.un.gyroscope.x - gyro_x);
                gyro_y += smoothing * (value.un.gyroscope.y - gyro_y);
                gyro_z += smoothing * (value.un.gyroscope.z - gyro_z);

                // keep the angles moving with the gyro while the BNO angle is bad
                if (gyro_only)
                {
                    untrimmed_roll += IMU_ROLL_DIRECTION * value.un.gyroscope.x * RAD2DEG * gyro_step;
                    untrimmed_pitch += IMU_PITCH_DIRECTION * value.un.gyroscope.y * RAD2DEG * gyro_step;
                    yaw_deg += IMU_YAW_DIRECTION * value.un.gyroscope.z * RAD2DEG * gyro_step;
                }
                last_gyro_us = time_us_32();
                have_gyro = true;
                new_gyro = true;
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

            float new_roll = IMU_ROLL_DIRECTION * bno_roll;
            float new_pitch = IMU_PITCH_DIRECTION * bno_pitch;
            bno_roll_reading = new_roll;
            bno_pitch_reading = new_pitch;

            // BNO lost its angle (vibration), the drone can't jump like that.
            // Fly on the gyro angles until the BNO agrees again (max GYRO_ONLY_MAX_US).
            bool jumped = armed && have_attitude &&
                          (fabsf(new_roll - untrimmed_roll) > MAX_ATTITUDE_JUMP_DEG ||
                           fabsf(new_pitch - untrimmed_pitch) > MAX_ATTITUDE_JUMP_DEG);
            if (jumped)
            {
                new_attitude = false;
                if (bad_imu_reads < 65535) bad_imu_reads++;
                if (!gyro_only)
                {
                    gyro_only = true;
                    gyro_only_since_us = now_us;
                }
            }
            else
            {
                gyro_only = false;
                untrimmed_roll = new_roll;
                untrimmed_pitch = new_pitch;
                roll_deg = untrimmed_roll - roll_trim_deg;
                pitch_deg = untrimmed_pitch - pitch_trim_deg;
                yaw_deg = IMU_YAW_DIRECTION * bno_yaw;

                last_attitude_us = now_us;
                have_attitude = true;
                loop_count++;
            }
        }

        if (gyro_only)
        {
            roll_deg = untrimmed_roll - roll_trim_deg;
            pitch_deg = untrimmed_pitch - pitch_trim_deg;
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

        //--------------------Optical flow-------------------------//
        bool new_flow = false;
        if (xQueuePeek(flow_mailbox, &flow, 0) == pdTRUE && flow.frame_count != last_flow_frame)
        {
            last_flow_frame = flow.frame_count;
            new_flow = true;
            have_flow = true;
        }

        bool flow_fresh = have_flow && (now_ms - flow.received_ms) < FLOW_TIMEOUT_MS;
        bool flow_was_usable = flow_usable;
        flow_usable = flow_fresh && flow.flow_ok && flow.distance_ok &&
                      flow.flow_quality >= FLOW_MIN_QUALITY &&
                      flow.distance_m >= FLOW_MIN_HEIGHT_M && flow.distance_m <= FLOW_MAX_HEIGHT_M &&
                      fabsf(roll_deg) < FLOW_MAX_TILT_DEG && fabsf(pitch_deg) < FLOW_MAX_TILT_DEG;

        if (flow_usable && !flow_was_usable)
        {
            forward_speed = 0.0f;
            right_speed = 0.0f;
            last_flow_received_ms = flow.received_ms;
        }

        if (new_flow && flow_usable)
        {
            float flow_forward = FLOW_SWAP_AXES ? flow.flow_y : flow.flow_x;
            float flow_right = FLOW_SWAP_AXES ? flow.flow_x : flow.flow_y;
            flow_forward *= FLOW_FORWARD_DIRECTION;
            flow_right *= FLOW_RIGHT_DIRECTION;

            if (FLOW_REMOVE_ROTATION)
            {
                // nose down looks like going forward, roll right looks like going right
                flow_forward += pitch_speed / RAD2DEG;
                flow_right -= roll_speed / RAD2DEG;
            }

            // flow is per metre, times the distance = ground speed
            forward_speed += FLOW_SPEED_FILTER * (flow_forward * flow.distance_m - forward_speed);
            right_speed += FLOW_SPEED_FILTER * (flow_right * flow.distance_m - right_speed);

            float flow_time_step = (flow.received_ms - last_flow_received_ms) * 0.001f;
            last_flow_received_ms = flow.received_ms;
            if (flow_time_step <= 0.0f || flow_time_step > 0.1f) flow_time_step = 0.01f;

            // turned with the yaw so the hold point stays put when it yaws
            float yaw_rad = yaw_deg / RAD2DEG;
            position_x += (forward_speed * cosf(yaw_rad) - right_speed * sinf(yaw_rad)) * flow_time_step;
            position_y += (forward_speed * sinf(yaw_rad) + right_speed * cosf(yaw_rad)) * flow_time_step;
        }

        //--------------------Laser height-------------------------//
        // laser is along the tilted drone, straight down is shorter
        float level_factor = cosf(roll_deg / RAD2DEG) * cosf(pitch_deg / RAD2DEG);
        if (new_flow && flow.distance_ok)
        {
            float new_height = flow.distance_m * level_factor - range_ground_distance;
            float height_step = (flow.received_ms - last_range_ms) * 0.001f;
            bool take_it = true;

            // one reading jumping away = laser glitch, skip it
            // the same jump many times in a row = the floor really moved, take it
            if (have_range && fabsf(new_height - last_range_height) > RANGE_JUMP_M)
            {
                if (range_jumps_in_row == 0)
                {
                    lowest_jump_height = new_height;
                    highest_jump_height = new_height;
                }
                if (new_height < lowest_jump_height) lowest_jump_height = new_height;
                if (new_height > highest_jump_height) highest_jump_height = new_height;
                range_jumps_in_row++;
                take_it = false;

                if (range_jumps_in_row >= RANGE_JUMP_READINGS)
                {
                    if (highest_jump_height - lowest_jump_height <= RANGE_JUMP_M)
                    {
                        // start again from the new height
                        take_it = true;
                        height_step = 0.0f;
                    }
                    range_jumps_in_row = 0;
                }
            }
            else
            {
                range_jumps_in_row = 0;
            }

            if (take_it)
            {
                if (!have_range || height_step <= 0.0f || height_step > 0.1f)
                {
                    // first reading or after a gap, start again from here
                    height = new_height;
                    climb_speed = 0.0f;
                }
                else
                {
                    float previous_height = height;
                    height += RANGE_HEIGHT_FILTER * (new_height - height);
                    climb_speed += CLIMB_SPEED_FILTER * ((height - previous_height) / height_step - climb_speed);
                    if (climb_speed > MAX_MEASURED_CLIMB_SPEED_M_S) climb_speed = MAX_MEASURED_CLIMB_SPEED_M_S;
                    if (climb_speed < -MAX_MEASURED_CLIMB_SPEED_M_S) climb_speed = -MAX_MEASURED_CLIMB_SPEED_M_S;
                }
                last_range_height = new_height;
                last_range_ms = flow.received_ms;
                have_range = true;

                if (height > LIFTOFF_HEIGHT_M && climb_speed > LIFTOFF_CLIMB_SPEED_M_S)
                {
                    liftoff_readings++;
                }
                else
                {
                    liftoff_readings = 0;
                }
            }
        }
        bool range_ok = have_range && (now_ms - last_range_ms) < RANGE_TIMEOUT_MS;

        bool auto_switch = link_ok && rc.ch[CH_AUTO] > CH_AUTO_ON;
        bool in_the_air = auto_mode && (auto_phase == AUTO_HOLD || auto_phase == AUTO_LANDING);

        bool gyro_only_ok = gyro_only && (now_us - gyro_only_since_us) < GYRO_ONLY_MAX_US;

        // BNO angle lost too long while flying: don't drop it, land slowly on the gyro
        bool flying = in_the_air || (armed && !auto_mode && throttle >= MANUAL_FLYING_THROTTLE);
        if (armed && gyro_only && !gyro_only_ok && flying && !imu_landing)
        {
            imu_landing = true;
            if (!auto_mode)
            {
                // manual: take over from here, the stick throttle is about hover
                auto_mode = true;
                hover_throttle = base_throttle;
                height_pid.reset();
                climb_speed_pid.reset();
            }
            auto_phase = AUTO_LANDING;
            phase_start_ms = now_ms;
            land_reason = LAND_IMU_LOST;
            maybe_on_ground = false;
            in_the_air = true;
        }

        bool imu_ok = have_attitude && have_gyro &&
                      ((now_us - last_attitude_us) < IMU_TIMEOUT_US || gyro_only_ok || imu_landing) &&
                      (now_us - last_gyro_us) < IMU_TIMEOUT_US;

        //--------------------Level-------------------------//
        // sticks: throttle down, yaw full left, pitch full back, arm switch off
        bool level_sticks = link_ok && !arm_switch && throttle < ARM_MAX_THROTTLE &&
                            YAW_STICK_DIRECTION * stick_position(rc.ch[CH_YAW]) < -0.9f &&
                            PITCH_STICK_DIRECTION * stick_position(rc.ch[CH_PITCH]) < -0.9f;
        if (!level_sticks)
        {
            level_sticks_since_ms = 0;
        }
        else if (level_sticks_since_ms == 0)
        {
            level_sticks_since_ms = now_ms;
        }

        bool start_level = level_requested ||
                           (level_sticks_since_ms != 0 && now_ms - level_sticks_since_ms >= LEVEL_STICK_HOLD_MS);
        if (!armed && !levelling && start_level && have_attitude)
        {
            levelling = true;
            level_start_ms = now_ms;
            level_roll_sum = 0.0f;
            level_pitch_sum = 0.0f;
            level_samples = 0;
            level_sticks_since_ms = 0;
            DEBUG_PRINTF("Levelling, keep the drone still\n");
        }
        level_requested = false;

        if (levelling && new_attitude)
        {
            bool moving = fabsf(roll_speed) > LEVEL_MAX_MOVING_DEG_S || fabsf(pitch_speed) > LEVEL_MAX_MOVING_DEG_S;
            bool tilted = fabsf(untrimmed_roll) > LEVEL_MAX_TILT_DEG || fabsf(untrimmed_pitch) > LEVEL_MAX_TILT_DEG;
            if (moving || tilted)
            {
                // start again once it sits still
                level_start_ms = now_ms;
                level_roll_sum = 0.0f;
                level_pitch_sum = 0.0f;
                level_samples = 0;
            }
            else
            {
                level_roll_sum += untrimmed_roll;
                level_pitch_sum += untrimmed_pitch;
                level_samples++;
            }

            if (now_ms - level_start_ms >= LEVEL_MEASURE_MS && level_samples > 0)
            {
                roll_trim_deg = level_roll_sum / level_samples;
                pitch_trim_deg = level_pitch_sum / level_samples;
                levelling = false;
            }
        }

        //--------------------Arm / Disarm-------------------------//
        const char *arm_blocked_by = nullptr;

        if (!armed)
        {
            if (levelling)
            {
                arm_blocked_by = "levelling";
            }
            else if (now_ms - motors_start_ms < ESC_STARTUP_MS)
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
            else if (auto_switch && !range_ok)
            {
                arm_blocked_by = "auto: no laser reading";
            }

            if (arm_switch && arm_blocked_by == nullptr)
            {
                reset_all_pids();
                motors.arm();
                armed = true;
                disarm_reason = DISARM_NONE;
                last_control_us = now_us;

                // auto switch decides the mode, only at arming
                auto_mode = auto_switch;
                auto_phase = auto_mode ? AUTO_SPIN_UP : AUTO_OFF;
                phase_start_ms = now_ms;
                land_reason = LAND_NONE;
                control_disarm = DISARM_NONE;
                maybe_on_ground = false;
                at_max_takeoff_throttle = false;
                imu_landing = false;
                flow_holding = false;
                base_throttle = 0.0f;
                target_climb_speed = 0.0f;

                // height 0 = here
                range_ground_distance = flow.distance_m * level_factor;
                height = 0.0f;
                climb_speed = 0.0f;
                target_height = 0.0f;
            }
        }
        else
        {
            uint8_t reason = DISARM_NONE;
            if (!link_ok && !in_the_air)
            {
                // RC lost on the ground or in manual: stop
                reason = DISARM_FAILSAFE;
            }
            else if (!link_ok)
            {
                // RC lost in the air in auto: land, checked below
            }
            else if (!arm_switch)
            {
                reason = DISARM_SWITCH;
            }
            else if (!imu_ok)
            {
                reason = gyro_only ? DISARM_IMU_LOST : DISARM_IMU_TIMEOUT;
            }
            else if (fabsf(roll_deg) > CRASH_TILT_DEG || fabsf(pitch_deg) > CRASH_TILT_DEG)
            {
                reason = DISARM_CRASH_TILT;
            }
            else if (control_disarm != DISARM_NONE)
            {
                reason = control_disarm;
            }

            if (reason == DISARM_NONE && auto_mode)
            {
                if (!in_the_air && link_ok && !auto_switch)
                {
                    // switched off before it lifted, just stop
                    reason = DISARM_AUTO_OFF_ON_GROUND;
                }
                else if (in_the_air && auto_phase != AUTO_LANDING)
                {
                    uint8_t new_land_reason = LAND_NONE;
                    if (!link_ok)
                    {
                        new_land_reason = LAND_RC_LOST;
                    }
                    else if (!auto_switch)
                    {
                        new_land_reason = LAND_SWITCH;
                    }
                    else if (range_ok && height > AUTO_MAX_HEIGHT_M)
                    {
                        new_land_reason = LAND_TOO_HIGH;
                    }

                    if (new_land_reason != LAND_NONE)
                    {
                        auto_phase = AUTO_LANDING;
                        phase_start_ms = now_ms;
                        land_reason = new_land_reason;
                        maybe_on_ground = false;
                    }
                }
                else if (auto_phase == AUTO_LANDING && now_ms - phase_start_ms > MAX_LANDING_MS)
                {
                    reason = DISARM_LANDING_TIMEOUT;
                }
            }

            if (reason != DISARM_NONE)
            {
                motors.disarm();
                armed = false;
                disarm_reason = reason;
                arm_switch_was_off = false; // switch has to go OFF again
                reset_all_pids();
                for (int i = 0; i < 4; i++) motor_power[i] = 0.0f;
                auto_mode = false;
                auto_phase = AUTO_OFF;
                control_disarm = DISARM_NONE;
                imu_landing = false;
                base_throttle = 0.0f;
                target_climb_speed = 0.0f;
                flow_holding = false;
            }
        }

        //--------------------Control-------------------------//
        // once per new attitude, or per gyro reading when flying on gyro angles
        if (armed && (new_attitude || (gyro_only && new_gyro)))
        {
            float time_step = (now_us - last_control_us) * 1e-6f;
            last_control_us = now_us;
            if (time_step < 0.0005f) time_step = 0.0005f;
            if (time_step > 0.02f) time_step = 0.02f;

            target_roll = ROLL_STICK_DIRECTION * stick_position(rc.ch[CH_ROLL]) * MAX_TILT_ANGLE_DEG;
            target_pitch = -PITCH_STICK_DIRECTION * stick_position(rc.ch[CH_PITCH]) * MAX_TILT_ANGLE_DEG; // forward = nose down
            target_yaw_speed = YAW_STICK_DIRECTION * stick_position(rc.ch[CH_YAW]) * MAX_YAW_SPEED_DEG_S;
            if (!link_ok)
            {
                // RC lost while landing, stay level
                target_roll = 0.0f;
                target_pitch = 0.0f;
                target_yaw_speed = 0.0f;
            }

            //--------------------Height control-------------------------//
            if (!auto_mode)
            {
                // manual, straight from the stick
                base_throttle = throttle;
            }
            else if (auto_phase == AUTO_SPIN_UP)
            {
                // props at idle for a moment
                base_throttle = 0.0f;
                if (now_ms - phase_start_ms >= SPIN_UP_MS)
                {
                    auto_phase = AUTO_TAKEOFF;
                    phase_start_ms = now_ms;
                    takeoff_throttle = TAKEOFF_START_THROTTLE;
                    at_max_takeoff_throttle = false;
                    liftoff_readings = 0;
                }
            }
            else if (auto_phase == AUTO_TAKEOFF)
            {
                // throttle goes up slowly until the laser sees it lift
                takeoff_throttle += TAKEOFF_THROTTLE_RAMP * time_step;
                if (takeoff_throttle >= TAKEOFF_MAX_THROTTLE)
                {
                    takeoff_throttle = TAKEOFF_MAX_THROTTLE;
                    if (!at_max_takeoff_throttle)
                    {
                        at_max_takeoff_throttle = true;
                        max_takeoff_throttle_since_ms = now_ms;
                    }
                }
                base_throttle = takeoff_throttle;

                if (range_ok && liftoff_readings >= LIFTOFF_READINGS)
                {
                    // lifted off, lift off throttle is too low (ground effect) or too high (ramp went on), use the guess
                    hover_throttle = HOVER_THROTTLE_GUESS;
                    height_pid.reset();
                    climb_speed_pid.reset();
                    target_height = height;
                    maybe_on_ground = false;
                    auto_phase = AUTO_HOLD;
                    phase_start_ms = now_ms;
                }
                else if (at_max_takeoff_throttle && now_ms - max_takeoff_throttle_since_ms >= NO_LIFTOFF_MS)
                {
                    control_disarm = DISARM_NO_LIFTOFF;
                }
            }
            else
            {
                // AUTO_HOLD or AUTO_LANDING
                if (auto_phase == AUTO_LANDING)
                {
                    target_climb_speed = -LANDING_SPEED_M_S;
                    target_height = height;
                }
                else
                {
                    // climb slowly to TAKEOFF_HEIGHT_M, then hold it
                    if (target_height < TAKEOFF_HEIGHT_M)
                    {
                        target_height += CLIMB_SPEED_M_S * time_step;
                        if (target_height > TAKEOFF_HEIGHT_M) target_height = TAKEOFF_HEIGHT_M;
                    }
                    target_climb_speed = height_pid.calculate(target_height, height, time_step);
                }

                float thrust_change = 0.0f;
                if (range_ok)
                {
                    climb_speed_pid.hold_integral(motors_at_limit);
                    thrust_change = climb_speed_pid.calculate(target_climb_speed, climb_speed, time_step);
                    // I can still move the hover down, only the quick part is limited
                    float thrust_drop = MAX_THRUST_DROP;
                    if (auto_phase == AUTO_HOLD && now_ms - phase_start_ms < LIFTOFF_GENTLE_MS) thrust_drop = LIFTOFF_THRUST_DROP;
                    float lowest_change = climb_speed_pid.get_integral_part() - thrust_drop;
                    if (thrust_change < lowest_change) thrust_change = lowest_change;
                }
                else
                {
                    // laser lost, sink slowly on the learned hover throttle
                    thrust_change = climb_speed_pid.get_integral_part() - BLIND_DROP;
                }

                // tilted = less lift, a bit more throttle
                float tilt_factor = level_factor;
                if (tilt_factor < MIN_TILT_FACTOR) tilt_factor = MIN_TILT_FACTOR;

                base_throttle = (hover_throttle + thrust_change) / tilt_factor;
                if (base_throttle < MIN_FLYING_THROTTLE) base_throttle = MIN_FLYING_THROTTLE;
                if (base_throttle > MAX_FLYING_THROTTLE) base_throttle = MAX_FLYING_THROTTLE;

                // wants down, not moving, less than hover = on the ground
                bool wants_down = target_climb_speed < -ON_GROUND_CLIMB_SPEED_M_S;
                bool not_moving = fabsf(climb_speed) < ON_GROUND_CLIMB_SPEED_M_S;
                bool low_enough = auto_phase == AUTO_LANDING && height < LANDED_HEIGHT_M;
                if (range_ok && ((wants_down && not_moving && thrust_change < 0.0f) || low_enough))
                {
                    if (!maybe_on_ground)
                    {
                        maybe_on_ground = true;
                        maybe_on_ground_since_ms = now_ms;
                    }
                    else if (now_ms - maybe_on_ground_since_ms >= ON_GROUND_MS)
                    {
                        control_disarm = DISARM_LANDED;
                    }
                }
                else
                {
                    maybe_on_ground = false;
                }
            }

            //--------------------Position hold-------------------------//
            // auto + sticks centred + good flow = hold here, otherwise plain self level
            bool hold_wanted = USE_FLOW_HOLD && auto_mode && (auto_phase == AUTO_HOLD || auto_phase == AUTO_LANDING);
            bool sticks_centred = target_roll == 0.0f && target_pitch == 0.0f;
            bool can_hold = hold_wanted && flow_usable && sticks_centred;

            if (can_hold)
            {
                if (!flow_holding)
                {
                    flow_holding = true;
                    hold_point_set = false;
                    forward_speed_pid.reset();
                    right_speed_pid.reset();
                }

                float target_forward_speed = 0.0f;
                float target_right_speed = 0.0f;

                if (!hold_point_set)
                {
                    // sticks just let go, stop first then hold there
                    if (fabsf(forward_speed) < BRAKE_DONE_SPEED_M_S && fabsf(right_speed) < BRAKE_DONE_SPEED_M_S)
                    {
                        hold_x = position_x;
                        hold_y = position_y;
                        hold_point_set = true;
                    }
                }
                else
                {
                    // distance to the hold point, turned back to forward / right
                    float yaw_rad = yaw_deg / RAD2DEG;
                    float error_x = hold_x - position_x;
                    float error_y = hold_y - position_y;
                    float error_forward = error_x * cosf(yaw_rad) + error_y * sinf(yaw_rad);
                    float error_right = -error_x * sinf(yaw_rad) + error_y * cosf(yaw_rad);

                    target_forward_speed = POSITION_GAIN * error_forward;
                    target_right_speed = POSITION_GAIN * error_right;
                    if (target_forward_speed > MAX_HOLD_SPEED_M_S) target_forward_speed = MAX_HOLD_SPEED_M_S;
                    if (target_forward_speed < -MAX_HOLD_SPEED_M_S) target_forward_speed = -MAX_HOLD_SPEED_M_S;
                    if (target_right_speed > MAX_HOLD_SPEED_M_S) target_right_speed = MAX_HOLD_SPEED_M_S;
                    if (target_right_speed < -MAX_HOLD_SPEED_M_S) target_right_speed = -MAX_HOLD_SPEED_M_S;
                }

                // nose down to go forward, right side down to go right
                target_pitch = -forward_speed_pid.calculate(target_forward_speed, forward_speed, time_step);
                target_roll = right_speed_pid.calculate(target_right_speed, right_speed, time_step);
            }
            else
            {
                flow_holding = false;
            }

            if (base_throttle < PID_START_THROTTLE)
            {
                // throttle down, all motors idle
                reset_all_pids();
                motors_at_limit = false;
                for (int i = 0; i < 4; i++) motor_power[i] = 0.0f;
            }
            else
            {
                // no I on the ground or when a motor is at its limit
                // auto: no I until it's in the air
                bool on_ground = auto_mode ? !in_the_air : throttle < INTEGRAL_START_THROTTLE;
                bool yaw_on_ground = auto_mode ? !in_the_air : throttle < YAW_INTEGRAL_START_THROTTLE;
                bool freeze_integral = on_ground || motors_at_limit;
                roll_speed_pid.hold_integral(freeze_integral);
                pitch_speed_pid.hold_integral(freeze_integral);
                yaw_speed_pid.hold_integral(yaw_on_ground || motors_at_limit);

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
                    motor_power[i] = base_throttle;
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
            status.throttle = armed ? base_throttle : throttle;
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
            status.roll_trim = roll_trim_deg;
            status.pitch_trim = pitch_trim_deg;
            status.levelling = levelling;
            status.imu_resets = imu_resets;
            status.bad_imu_reads = bad_imu_reads;
            status.gyro_only = gyro_only;
            status.bno_roll = bno_roll_reading - roll_trim_deg;
            status.bno_pitch = bno_pitch_reading - pitch_trim_deg;
            status.flow_ok = flow_usable;
            status.flow_hold = armed && flow_holding;
            status.flow_quality = flow.flow_quality;
            status.flow_distance = flow.distance_m;
            status.forward_speed = forward_speed;
            status.right_speed = right_speed;
            status.auto_phase = auto_phase;
            status.land_reason = land_reason;
            status.range_ok = range_ok;
            status.height = height;
            status.target_height = target_height;
            status.climb_speed = climb_speed;
            status.target_climb_speed = target_climb_speed;
            status.hover_throttle = hover_throttle;
            status.position_x = position_x;
            status.position_y = position_y;
            status.hold_x = hold_x;
            status.hold_y = hold_y;

            xQueueOverwrite(status_mailbox, &status);

            if (armed)
            {
                last_armed_ms = now_ms;
                was_ever_armed = true;
            }
            if (armed || (was_ever_armed && now_ms - last_armed_ms < LOG_AFTER_DISARM_MS))
            {
                flight_log_add(status);
            }
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
    bool was_levelling = false;
    uint8_t last_reason = DISARM_NONE;
    uint8_t last_auto_phase = AUTO_OFF;
    uint8_t last_imu_resets = 0;
    uint16_t last_bad_imu_reads = 0;
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
            if (status.armed && status.auto_phase != last_auto_phase)
            {
                if (status.auto_phase == AUTO_LANDING)
                {
                    printf(">>> LANDING: %s\n", land_reason_text(status.land_reason));
                }
                else if (status.auto_phase == AUTO_HOLD)
                {
                    printf(">>> LIFT OFF at throttle %.2f\n", status.hover_throttle);
                }
                else
                {
                    printf(">>> %s\n", auto_phase_text(status.auto_phase));
                }
            }
            last_auto_phase = status.armed ? status.auto_phase : AUTO_OFF;
            was_armed = status.armed;
            last_reason = status.disarm_reason;

            if (was_levelling && !status.levelling)
            {
                printf(">>> LEVEL SET: ROLL_TRIM_DEG = %.1ff, PITCH_TRIM_DEG = %.1ff\n",
                       status.roll_trim, status.pitch_trim);
            }
            was_levelling = status.levelling;

            if (status.imu_resets != last_imu_resets || status.bad_imu_reads != last_bad_imu_reads)
            {
                printf(">>> IMU: %u resets, %u bad reads\n", status.imu_resets, status.bad_imu_reads);
                last_imu_resets = status.imu_resets;
                last_bad_imu_reads = status.bad_imu_reads;
            }

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
                printf(" | %s H %d %5.2f tg %5.2f vs %+5.2f tg %+5.2f",
                       auto_phase_text(status.auto_phase), status.range_ok, status.height, status.target_height,
                       status.climb_speed, status.target_climb_speed);
                printf(" | flow %d q %3u d %.2f fwd %+.2f right %+.2f hold %d",
                       status.flow_ok, status.flow_quality, status.flow_distance,
                       status.forward_speed, status.right_speed, status.flow_hold);
                if (!status.armed && status.arm_blocked_by) printf(" | no arm: %s", status.arm_blocked_by);
                printf("\n");
            }
        }

        SerialCmd command = serial_poll_command();
        if (command == CMD_LEVEL)
        {
            if (was_armed)
            {
                printf("Disarm first to level\n");
            }
            else
            {
                printf("Levelling for 2 s, keep the drone still on a level floor\n");
                level_requested = true;
            }
        }
        else if (command == CMD_CHANNELS)
        {
            channels_print_on = !channels_print_on;
            printf("Channels print %s\n", channels_print_on ? "on" : "off");
        }
        else if (command == CMD_FLOW)
        {
            flow_print_on = !flow_print_on;
            printf("Flow print %s\n", flow_print_on ? "on" : "off");
        }
        else if (command == CMD_LOG)
        {
            // not while flying, printing it all takes a while
            if (was_armed)
            {
                printf("Disarm first to print the log\n");
            }
            else
            {
                flight_log_print();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


// ========================================================
// ESC CALIBRATION
// Sends max pulse (2000 us), then min pulse (1000 us) so the ESCs learn the range.
// Runs instead of the flight task, set ESC_CALIBRATION_MODE to true to use it.
// PROPS OFF. Start with the battery unplugged, Pico on USB.
// ========================================================
static constexpr bool ESC_CALIBRATION_MODE = false;

static void wait_for_esc_command()
{
    while (serial_poll_command() != CMD_ESC)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void esc_calibration_task(void *pvParameters)
{
    (void)pvParameters;

    Motors::Config motor_config;
    motor_config.pins[0] = MOTOR0_PIN;
    motor_config.pins[1] = MOTOR1_PIN;
    motor_config.pins[2] = MOTOR2_PIN;
    motor_config.pins[3] = MOTOR3_PIN;
    motor_config.count = 4;
    static Motors motors(motor_config);

    if (!motors.begin())
    {
        printf("Motors error: %s\n", motors.lastError());
        vTaskDelete(NULL);
    }

    printf("\n=== ESC CALIBRATION ===\n");
    printf("PROPS OFF, battery unplugged\n");
    printf("Type esc to send max pulse\n");
    wait_for_esc_command();

    // arm so setAll can go above the stop pulse, 1.0 = max_us
    motors.arm();
    motors.setAll(1.0f);
    printf("Max pulse on all motors (%u us)\n", motors.pulseUs(0));
    printf("Plug in the battery now, wait for the beeps\n");
    printf("Then type esc to send min pulse\n");
    wait_for_esc_command();

    // disarm = min_us on all motors
    motors.disarm();
    printf("Min pulse on all motors (%u us)\n", motors.pulseUs(0));
    printf("Wait for the confirm beeps, then unplug the battery\n");
    printf("Set ESC_CALIBRATION_MODE back to false and flash again\n");

    // keep the stop pulse going
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
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
    flow_mailbox = xQueueCreate(1, sizeof(Sensors::OpticalFlowReading));

    xTaskCreate(core0_system_task, "TempTask", 256, NULL, 1, &core0_task_handle);

    //xTaskCreate(core1_flight_task, "ClockTask", 256, NULL, 1, &core1_task_handle);

    xTaskCreate(core0_elrs_task, "ELRSTask", 256, NULL, 1, &elrs_task_handle);
    xTaskCreate(optical_flow_task, "FlowTask", 1024, NULL, 1, &optical_flow_task_handle);
    //xTaskCreate(bno085_task, "ImuTask", 1024, NULL, 1, &bno085_task_handle);
    //xTaskCreate(bno085_imu_task, "BnoTask", 1024, NULL, 1, &bno085_task_handle);
    //xTaskCreate(bmp581_task,"BMPTask",1024,NULL,1,&bmp581_task_handle);
    //xTaskCreate(motor_task, "MotorTest", 1024, NULL, 1, &motor_test_handle);

    if (ESC_CALIBRATION_MODE)
    {
        // no flight task, only ESC calibration
        xTaskCreate(esc_calibration_task, "EscCal", 1024, NULL, 2, &motor_test_handle);
    }
    else
    {
        // flight task uses bno085_task_handle so bno_int_isr wakes it
        xTaskCreate(flight_control_task, "Flight", 2048, NULL, 3, &bno085_task_handle);
        xTaskCreate(flight_telemetry_task, "Telemetry", 1024, NULL, 1, &core1_task_handle);
        vTaskCoreAffinitySet(core1_task_handle, (1 << 0)); // flight print
        vTaskCoreAffinitySet(bno085_task_handle, (1 << 1)); // flight control
    }

    vTaskCoreAffinitySet(elrs_task_handle, (1 << 0));
    vTaskCoreAffinitySet(optical_flow_task_handle, (1 << 0)); // PIO interrupt on core 0 too
    //vTaskCoreAffinitySet(core0_task_handle, (1 << 0));
    //vTaskCoreAffinitySet(bmp581_task_handle, (1 << 1));
    //vTaskCoreAffinitySet(motor_test_handle, (1 << 1));

    
    vTaskStartScheduler();

    while (true) {
        //tight_loop_contents();
    }
    return 0;
}


 
