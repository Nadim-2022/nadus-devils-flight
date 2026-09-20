/*!
 *  @file BMP5xx.h
 *
 *  Native Raspberry Pi Pico (pico-sdk) SPI driver for the Bosch BMP580 /
 *  BMP581 pressure and temperature sensor.
 *
 *  No Arduino, no Adafruit_BusIO, no Adafruit_Sensor. Only the Bosch BMP5
 *  SensorAPI (bmp5.c / bmp5.h / bmp5_defs.h) and the pico-sdk.
 *
 *  Derived from the Adafruit_BMP5xx library by Limor "ladyada" Fried
 *  (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution.
 */

#ifndef BMP5XX_H
#define BMP5XX_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

extern "C" {
#include "../../external/BMP581/bmp5.h"
}

 
/*=========================================================================
    CHIP IDS
    -----------------------------------------------------------------------*/
/**! Chip ID for BMP580 */
#define BMP580_CHIP_ID (0x50)
/**! Chip ID for BMP581 */
#define BMP581_CHIP_ID (0x51)
 
/**! Default SPI clock in Hz. BMP58x tolerates up to ~10 MHz. */
#define BMP5XX_DEFAULT_SPI_BAUDRATE (1000000u)
/*=========================================================================*/
 
/**
 * @brief Oversampling settings
 */
typedef enum {
  BMP5XX_OVERSAMPLING_1X = BMP5_OVERSAMPLING_1X,     ///< 1x oversampling
  BMP5XX_OVERSAMPLING_2X = BMP5_OVERSAMPLING_2X,     ///< 2x oversampling
  BMP5XX_OVERSAMPLING_4X = BMP5_OVERSAMPLING_4X,     ///< 4x oversampling
  BMP5XX_OVERSAMPLING_8X = BMP5_OVERSAMPLING_8X,     ///< 8x oversampling
  BMP5XX_OVERSAMPLING_16X = BMP5_OVERSAMPLING_16X,   ///< 16x oversampling
  BMP5XX_OVERSAMPLING_32X = BMP5_OVERSAMPLING_32X,   ///< 32x oversampling
  BMP5XX_OVERSAMPLING_64X = BMP5_OVERSAMPLING_64X,   ///< 64x oversampling
  BMP5XX_OVERSAMPLING_128X = BMP5_OVERSAMPLING_128X, ///< 128x oversampling
} bmp5xx_oversampling_t;
 
/**
 * @brief IIR filter coefficients
 */
typedef enum {
  BMP5XX_IIR_FILTER_BYPASS = BMP5_IIR_FILTER_BYPASS,       ///< No filtering
  BMP5XX_IIR_FILTER_COEFF_1 = BMP5_IIR_FILTER_COEFF_1,     ///< Filter coeff 1
  BMP5XX_IIR_FILTER_COEFF_3 = BMP5_IIR_FILTER_COEFF_3,     ///< Filter coeff 3
  BMP5XX_IIR_FILTER_COEFF_7 = BMP5_IIR_FILTER_COEFF_7,     ///< Filter coeff 7
  BMP5XX_IIR_FILTER_COEFF_15 = BMP5_IIR_FILTER_COEFF_15,   ///< Filter coeff 15
  BMP5XX_IIR_FILTER_COEFF_31 = BMP5_IIR_FILTER_COEFF_31,   ///< Filter coeff 31
  BMP5XX_IIR_FILTER_COEFF_63 = BMP5_IIR_FILTER_COEFF_63,   ///< Filter coeff 63
  BMP5XX_IIR_FILTER_COEFF_127 = BMP5_IIR_FILTER_COEFF_127, ///< Filter coeff 127
} bmp5xx_iir_filter_t;
 
/**
 * @brief Output Data Rate settings
 */
typedef enum {
  BMP5XX_ODR_240_HZ = BMP5_ODR_240_HZ,     ///< 240 Hz
  BMP5XX_ODR_218_5_HZ = BMP5_ODR_218_5_HZ, ///< 218.5 Hz
  BMP5XX_ODR_199_1_HZ = BMP5_ODR_199_1_HZ, ///< 199.1 Hz
  BMP5XX_ODR_179_2_HZ = BMP5_ODR_179_2_HZ, ///< 179.2 Hz
  BMP5XX_ODR_160_HZ = BMP5_ODR_160_HZ,     ///< 160 Hz
  BMP5XX_ODR_149_3_HZ = BMP5_ODR_149_3_HZ, ///< 149.3 Hz
  BMP5XX_ODR_140_HZ = BMP5_ODR_140_HZ,     ///< 140 Hz
  BMP5XX_ODR_129_8_HZ = BMP5_ODR_129_8_HZ, ///< 129.8 Hz
  BMP5XX_ODR_120_HZ = BMP5_ODR_120_HZ,     ///< 120 Hz
  BMP5XX_ODR_110_1_HZ = BMP5_ODR_110_1_HZ, ///< 110.1 Hz
  BMP5XX_ODR_100_2_HZ = BMP5_ODR_100_2_HZ, ///< 100.2 Hz
  BMP5XX_ODR_89_6_HZ = BMP5_ODR_89_6_HZ,   ///< 89.6 Hz
  BMP5XX_ODR_80_HZ = BMP5_ODR_80_HZ,       ///< 80 Hz
  BMP5XX_ODR_70_HZ = BMP5_ODR_70_HZ,       ///< 70 Hz
  BMP5XX_ODR_60_HZ = BMP5_ODR_60_HZ,       ///< 60 Hz
  BMP5XX_ODR_50_HZ = BMP5_ODR_50_HZ,       ///< 50 Hz
  BMP5XX_ODR_45_HZ = BMP5_ODR_45_HZ,       ///< 45 Hz
  BMP5XX_ODR_40_HZ = BMP5_ODR_40_HZ,       ///< 40 Hz
  BMP5XX_ODR_35_HZ = BMP5_ODR_35_HZ,       ///< 35 Hz
  BMP5XX_ODR_30_HZ = BMP5_ODR_30_HZ,       ///< 30 Hz
  BMP5XX_ODR_25_HZ = BMP5_ODR_25_HZ,       ///< 25 Hz
  BMP5XX_ODR_20_HZ = BMP5_ODR_20_HZ,       ///< 20 Hz
  BMP5XX_ODR_15_HZ = BMP5_ODR_15_HZ,       ///< 15 Hz
  BMP5XX_ODR_10_HZ = BMP5_ODR_10_HZ,       ///< 10 Hz
  BMP5XX_ODR_05_HZ = BMP5_ODR_05_HZ,       ///< 5 Hz
  BMP5XX_ODR_04_HZ = BMP5_ODR_04_HZ,       ///< 4 Hz
  BMP5XX_ODR_03_HZ = BMP5_ODR_03_HZ,       ///< 3 Hz
  BMP5XX_ODR_02_HZ = BMP5_ODR_02_HZ,       ///< 2 Hz
  BMP5XX_ODR_01_HZ = BMP5_ODR_01_HZ,       ///< 1 Hz
  BMP5XX_ODR_0_5_HZ = BMP5_ODR_0_5_HZ,     ///< 0.5 Hz
  BMP5XX_ODR_0_250_HZ = BMP5_ODR_0_250_HZ, ///< 0.25 Hz
  BMP5XX_ODR_0_125_HZ = BMP5_ODR_0_125_HZ, ///< 0.125 Hz
} bmp5xx_odr_t;
 
/**
 * @brief Power mode settings
 */
typedef enum {
  BMP5XX_POWERMODE_STANDBY = BMP5_POWERMODE_STANDBY,      ///< Standby mode
  BMP5XX_POWERMODE_NORMAL = BMP5_POWERMODE_NORMAL,        ///< Normal mode
  BMP5XX_POWERMODE_FORCED = BMP5_POWERMODE_FORCED,        ///< Forced mode
  BMP5XX_POWERMODE_CONTINUOUS = BMP5_POWERMODE_CONTINOUS, ///< Continuous mode
  BMP5XX_POWERMODE_DEEP_STANDBY =
      BMP5_POWERMODE_DEEP_STANDBY, ///< Deep standby mode
} bmp5xx_powermode_t;
 
/**
 * @brief Interrupt polarity settings
 */
typedef enum {
  BMP5XX_INTERRUPT_ACTIVE_LOW = BMP5_ACTIVE_LOW,  ///< Interrupt active low
  BMP5XX_INTERRUPT_ACTIVE_HIGH = BMP5_ACTIVE_HIGH ///< Interrupt active high
} bmp5xx_interrupt_polarity_t;
 
/**
 * @brief Interrupt drive settings
 */
typedef enum {
  BMP5XX_INTERRUPT_PUSH_PULL = BMP5_INTR_PUSH_PULL,  ///< Push-pull output
  BMP5XX_INTERRUPT_OPEN_DRAIN = BMP5_INTR_OPEN_DRAIN ///< Open-drain output
} bmp5xx_interrupt_drive_t;
 
/**
 * @brief Interrupt mode settings
 */
typedef enum {
  BMP5XX_INTERRUPT_PULSED = BMP5_PULSED,  ///< Pulsed interrupt
  BMP5XX_INTERRUPT_LATCHED = BMP5_LATCHED ///< Latched interrupt
} bmp5xx_interrupt_mode_t;
 
/**
 * @brief Interrupt source settings (can be combined with bitwise OR)
 */
typedef enum {
  BMP5XX_INTERRUPT_DATA_READY = 0x01,     ///< Data ready interrupt
  BMP5XX_INTERRUPT_FIFO_FULL = 0x02,      ///< FIFO full interrupt
  BMP5XX_INTERRUPT_FIFO_THRESHOLD = 0x04, ///< FIFO threshold interrupt
  BMP5XX_INTERRUPT_PRESSURE_OUT_OF_RANGE =
      0x08 ///< Pressure out of range interrupt
} bmp5xx_interrupt_source_t;
 
/**
 * @brief Bus context handed to the Bosch API callbacks through intf_ptr.
 */
typedef struct {
  spi_inst_t* spi; /**< spi0 or spi1 */
  uint cs_pin;     /**< GPIO used as chip select (software controlled) */
} bmp5xx_spi_intf_t;
 
/**
 * @brief One complete measurement.
 */
typedef struct {
  float temperature;  /**< Temperature in degrees Celsius */
  float pressure;     /**< Pressure in hPa */
  uint32_t timestamp; /**< Milliseconds since boot */
} bmp5xx_data_t;
 
/**
 * @brief Driver for the BMP580 / BMP581 barometric pressure sensor over the
 * Pico's hardware SPI peripheral.
 */
class BMP5xx {
 public:
  BMP5xx();
  ~BMP5xx(void);
 
  bool begin(spi_inst_t* spi, uint sck_pin, uint mosi_pin, uint miso_pin,
             uint cs_pin,
             uint32_t baudrate = BMP5XX_DEFAULT_SPI_BAUDRATE);
  bool beginPreconfigured(spi_inst_t* spi, uint cs_pin);
 
  float readTemperature(void);
  float readPressure(void);
  float readAltitude(float seaLevel = 1013.25f);
 
  bool performReading(void);
  bool getData(bmp5xx_data_t* data);
 
  bool setTemperatureOversampling(bmp5xx_oversampling_t oversampling);
  bool setPressureOversampling(bmp5xx_oversampling_t oversampling);
  bool setIIRFilterCoeff(bmp5xx_iir_filter_t filtercoeff);
  bool setOutputDataRate(bmp5xx_odr_t odr);
  bool setPowerMode(bmp5xx_powermode_t powermode);
 
  bmp5xx_oversampling_t getTemperatureOversampling(void);
  bmp5xx_oversampling_t getPressureOversampling(void);
  bmp5xx_iir_filter_t getIIRFilterCoeff(void);
  bmp5xx_odr_t getOutputDataRate(void);
  bmp5xx_powermode_t getPowerMode(void);
 
  bool enablePressure(bool enable = true);
  bool dataReady(void);
 
  bool configureInterrupt(bmp5xx_interrupt_mode_t mode,
                          bmp5xx_interrupt_polarity_t polarity,
                          bmp5xx_interrupt_drive_t drive,
                          uint8_t sources = BMP5XX_INTERRUPT_DATA_READY,
                          bool enable = true);
 
  bool softReset(void);
 
  /**
   * @brief Chip ID read during begin(). 0x50 = BMP580, 0x51 = BMP581.
   * @return Chip ID byte, 0 if begin() has not succeeded yet.
   */
  uint8_t getChipID(void) const {
    return _bmp5_dev.chip_id;
  }
 
  /**
   * @brief Result code of the last Bosch API call (BMP5_OK == 0).
   * @return Last Bosch API status code
   */
  int8_t lastStatus(void) const {
    return _last_status;
  }
 
  /**! Temperature (Celsius) assigned after calling performReading() */
  float temperature;
  /**! Pressure (hPa) assigned after calling performReading() */
  float pressure;
 
 private:
  bool _init(void);
 
  /**
   * @brief Polls a register until (value & mask) == expected, or a timeout
   * elapses. Used instead of fixed delays for reset/readiness conditions
   * whose actual settling time varies boot to boot.
   * @return True if the condition was observed before the timeout.
   */
  bool poll_register(uint8_t reg, uint8_t mask, uint8_t expected,
                     uint32_t timeout_us, uint32_t poll_interval_us);
 
  /**! BMP5xx device struct from Bosch API */
  struct bmp5_dev _bmp5_dev;
 
  /**! Configuration struct for OSR/ODR settings */
  struct bmp5_osr_odr_press_config _osr_odr_config;
 
  /**! Configuration struct for IIR filter settings */
  struct bmp5_iir_config _iir_config;
 
  /**! SPI bus + chip select passed to the Bosch callbacks */
  bmp5xx_spi_intf_t _intf;
 
  /**! Result of the most recent Bosch API call */
  int8_t _last_status;
 
  // Static callbacks for the Bosch API
  static BMP5_INTF_RET_TYPE spi_read(uint8_t reg_addr, uint8_t* reg_data,
                                     uint32_t len, void* intf_ptr);
  static BMP5_INTF_RET_TYPE spi_write(uint8_t reg_addr, const uint8_t* reg_data,
                                      uint32_t len, void* intf_ptr);
  static void delay_usec(uint32_t us, void* intf_ptr);
};
 
#endif // BMP5XX_H
 