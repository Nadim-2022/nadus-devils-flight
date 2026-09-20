/*!
 *  @file BMP5xx.cpp
 *
 *  Native Raspberry Pi Pico (pico-sdk) SPI driver for the Bosch BMP580 /
 *  BMP581 pressure and temperature sensor.
 *
 *  Chip select is driven in software so the Pico's hardware SS is not needed
 *  and the bus can be shared with other devices.
 *
 *  Derived from the Adafruit_BMP5xx library by Limor "ladyada" Fried
 *  (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution.
 */

#include "sensors/BMP5xx.h"
#include "debug/debug.hpp"

#include <math.h>
#include <string.h>

#include "hardware/sync.h"
 
 
/* ---------------------------------------------------------------------- */
/* Chip select helpers                                                     */
/* ---------------------------------------------------------------------- */
 
static inline void cs_select(uint cs_pin) {
  asm volatile("nop \n nop \n nop");
  gpio_put(cs_pin, 0);
  asm volatile("nop \n nop \n nop");
}
 
static inline void cs_deselect(uint cs_pin) {
  asm volatile("nop \n nop \n nop");
  gpio_put(cs_pin, 1);
  asm volatile("nop \n nop \n nop");
}
 
/* ---------------------------------------------------------------------- */
/* Construction / destruction                                              */
/* ---------------------------------------------------------------------- */
 
/*!
 * @brief  BMP5xx constructor
 */
BMP5xx::BMP5xx(void) {
  memset(&_bmp5_dev, 0, sizeof(_bmp5_dev));
  memset(&_osr_odr_config, 0, sizeof(_osr_odr_config));
  memset(&_iir_config, 0, sizeof(_iir_config));
  _intf.spi = NULL;
  _intf.cs_pin = 0;
  _last_status = BMP5_OK;
  temperature = 0.0f;
  pressure = 0.0f;
}
 
/*!
 * @brief  BMP5xx destructor. Nothing is heap allocated, so this only puts the
 * sensor back into standby if it was initialized.
 */
BMP5xx::~BMP5xx(void) {
  if (_intf.spi) {
    bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &_bmp5_dev);
  }
}
 
/* ---------------------------------------------------------------------- */
/* Initialization                                                          */
/* ---------------------------------------------------------------------- */
 
/*!
 * @brief Initializes the SPI peripheral, the pins and the sensor.
 * @param spi SPI instance to use, spi0 or spi1
 * @param sck_pin GPIO used for SCK (must be a valid SCK pin for that instance)
 * @param mosi_pin GPIO used for MOSI / TX (sensor SDI)
 * @param miso_pin GPIO used for MISO / RX (sensor SDO)
 * @param cs_pin GPIO used for chip select, driven in software
 * @param baudrate SPI clock in Hz, default 1 MHz
 * @return True if initialization was successful, otherwise false.
 */
bool BMP5xx::begin(spi_inst_t* spi, uint sck_pin, uint mosi_pin, uint miso_pin,
                   uint cs_pin, uint32_t baudrate) {
  if (!spi) {
    return false;
  }
 
  spi_init(spi, baudrate);
  spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
 
  gpio_set_function(sck_pin, GPIO_FUNC_SPI);
  gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
  gpio_set_function(miso_pin, GPIO_FUNC_SPI);
 
  return beginPreconfigured(spi, cs_pin);
}
 
/*!
 * @brief Initializes the sensor on an SPI bus that has already been set up
 * elsewhere (spi_init + gpio_set_function already done). Useful when several
 * devices share the bus. The bus must be in mode 0 or mode 3, MSB first.
 * @param spi SPI instance the sensor is wired to
 * @param cs_pin GPIO used for chip select, driven in software
 * @return True if initialization was successful, otherwise false.
 */
bool BMP5xx::beginPreconfigured(spi_inst_t* spi, uint cs_pin) {
  if (!spi) {
    return false;
  }
 
  _intf.spi = spi;
  _intf.cs_pin = cs_pin;
 
  // Chip select idles high
  gpio_init(cs_pin);
  gpio_set_dir(cs_pin, GPIO_OUT);
  gpio_put(cs_pin, 1);
 
  // Give the part time to come out of power on reset
  sleep_ms(5);
 
  // Setup Bosch API callbacks
  _bmp5_dev.intf_ptr = &_intf;
  _bmp5_dev.intf = BMP5_SPI_INTF;
  _bmp5_dev.read = BMP5xx::spi_read;
  _bmp5_dev.write = BMP5xx::spi_write;
  _bmp5_dev.delay_us = BMP5xx::delay_usec;
 
  // Retry the entire reset-and-verify handshake from scratch, not just the
  // chip-ID read at the tail end. Observed on real hardware: the whole
  // sequence sometimes fails at its very first check (the POR-complete poll)
  // rather than partway through, so a retry has to start over at the reset
  // command itself to have a chance of succeeding.
  for (int attempt = 0; attempt < 5; attempt++) {
    if (_init()) {
      return true;
    }
    sleep_ms(10 * (attempt + 1)); // back off a bit more each attempt
  }
  return false;
}
 
/*!
 * @brief Polls a register until (value & mask) == expected, or timeout.
 * @return True if the condition was seen before the timeout elapsed.
 */
bool BMP5xx::poll_register(uint8_t reg, uint8_t mask, uint8_t expected,
                           uint32_t timeout_us, uint32_t poll_interval_us) {
  uint32_t elapsed = 0;
  uint8_t val = 0;
  while (true) {
    int8_t rslt = bmp5_get_regs(reg, &val, 1, &_bmp5_dev);
    if (rslt == BMP5_OK && (val & mask) == expected) {
      return true;
    }
    if (elapsed >= timeout_us) {
      return false;
    }
    _bmp5_dev.delay_us(poll_interval_us, _bmp5_dev.intf_ptr);
    elapsed += poll_interval_us;
  }
}
 
/*!
 * @brief Performs the actual initialization using the Bosch API
 * @return True if initialization was successful, otherwise false.
 */
bool BMP5xx::_init(void) {
  // Reset the sensor first
  int8_t rslt = bmp5_soft_reset(&_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }

  // Now initialize the sensor
  rslt = bmp5_init(&_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }

  // Set default configuration
  _osr_odr_config.osr_t = BMP5_OVERSAMPLING_2X;
  _osr_odr_config.osr_p = BMP5_OVERSAMPLING_16X;
  _osr_odr_config.odr = BMP5_ODR_50_HZ;
  _osr_odr_config.press_en = BMP5_ENABLE;

  rslt = bmp5_set_osr_odr_press_config(&_osr_odr_config, &_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }
 // Set default IIR filter
  _iir_config.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
  _iir_config.set_iir_p = BMP5_IIR_FILTER_COEFF_1;
  _iir_config.shdw_set_iir_t = BMP5_ENABLE;
  _iir_config.shdw_set_iir_p = BMP5_ENABLE;
  _iir_config.iir_flush_forced_en = BMP5_ENABLE;

  rslt = bmp5_set_iir_config(&_iir_config, &_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }

  // Set to normal mode
  rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }

  // Enable data ready interrupt for non-blocking data ready checking
  struct bmp5_int_source_select int_source_select = {0};
  int_source_select.drdy_en = BMP5_ENABLE;
  int_source_select.fifo_full_en = BMP5_DISABLE;
  int_source_select.fifo_thres_en = BMP5_DISABLE;
  int_source_select.oor_press_en = BMP5_DISABLE;

  rslt = bmp5_int_source_select(&int_source_select, &_bmp5_dev);
  if (rslt != BMP5_OK) {
    return false;
  }

  // Configure interrupt pin as push-pull, active high, latched mode
  rslt = bmp5_configure_interrupt(BMP5_LATCHED, BMP5_ACTIVE_HIGH,
                                  BMP5_INTR_PUSH_PULL, BMP5_INTR_ENABLE,
                                  &_bmp5_dev);
  return rslt == BMP5_OK;
 
}
 
/*!
 * @brief Issues a soft reset and re-applies the stored configuration.
 * @return True on success, False on failure
 */
bool BMP5xx::softReset(void) {
  if (!_intf.spi) {
    return false;
  }
  return _init();
}
 
/* ---------------------------------------------------------------------- */
/* Measurements                                                            */
/* ---------------------------------------------------------------------- */
 
/*!
 * @brief Performs a reading of both temperature and pressure and stores values
 * in class instance variables
 * @return True if the reading was successful, otherwise false.
 */
bool BMP5xx::performReading(void) {
  struct bmp5_sensor_data sensor_data;
 
  _last_status = bmp5_get_sensor_data(&sensor_data, &_osr_odr_config,
                                      &_bmp5_dev);
  if (_last_status != BMP5_OK) {
    return false;
  }
 
  temperature = (float)sensor_data.temperature;
  pressure = (float)sensor_data.pressure / 100.0f; // Convert Pa to hPa
 
  return true;
}
 
/*!
 * @brief Takes a reading and fills a data struct with a timestamp
 * @param data Struct that will receive temperature, pressure and timestamp
 * @return True if the reading was successful, otherwise false.
 */
bool BMP5xx::getData(bmp5xx_data_t* data) {
  if (!data) {
    return false;
  }
  if (!performReading()) {
    return false;
  }
  data->temperature = temperature;
  data->pressure = pressure;
  data->timestamp = to_ms_since_boot(get_absolute_time());
  return true;
}
 
/*!
 * @brief Returns the temperature from a fresh reading
 * @return Temperature in degrees Celsius
 */
float BMP5xx::readTemperature(void) {
  performReading();
  return temperature;
}
 
/*!
 * @brief Returns the pressure from a fresh reading
 * @return Pressure in hPa
 */
float BMP5xx::readPressure(void) {
  performReading();
  return pressure;
}
 
/*!
 * @brief Calculates the approximate altitude using barometric pressure
 * @param seaLevel Sea level pressure in hPa (default = 1013.25)
 * @return Altitude in meters
 */
float BMP5xx::readAltitude(float seaLevel) {
  float atmospheric = readPressure();
  return 44330.0f * (1.0f - powf(atmospheric / seaLevel, 0.1903f));
}
 
/* ---------------------------------------------------------------------- */
/* Configuration                                                           */
/* ---------------------------------------------------------------------- */
 
/*!
 * @brief Set temperature oversampling
 * @param oversampling Oversampling setting
 * @return True on success, False on failure
 */
bool BMP5xx::setTemperatureOversampling(bmp5xx_oversampling_t oversampling) {
  _osr_odr_config.osr_t = (uint8_t)oversampling;
  _last_status = bmp5_set_osr_odr_press_config(&_osr_odr_config, &_bmp5_dev);
  return _last_status == BMP5_OK;
}
 
/*!
 * @brief Set pressure oversampling
 * @param oversampling Oversampling setting
 * @return True on success, False on failure
 */
bool BMP5xx::setPressureOversampling(bmp5xx_oversampling_t oversampling) {
  _osr_odr_config.osr_p = (uint8_t)oversampling;
  _last_status = bmp5_set_osr_odr_press_config(&_osr_odr_config, &_bmp5_dev);
  return _last_status == BMP5_OK;
}
 
/*!
 * @brief Set IIR filter coefficient
 * @param filtercoeff IIR filter coefficient
 * @return True on success, False on failure
 */
bool BMP5xx::setIIRFilterCoeff(bmp5xx_iir_filter_t filtercoeff) {
  _iir_config.set_iir_t = (uint8_t)filtercoeff;
  _iir_config.set_iir_p = (uint8_t)filtercoeff;
  _last_status = bmp5_set_iir_config(&_iir_config, &_bmp5_dev);
  return _last_status == BMP5_OK;
}
 
/*!
 * @brief Set output data rate
 * @param odr Output data rate setting
 * @return True on success, False on failure
 */
bool BMP5xx::setOutputDataRate(bmp5xx_odr_t odr) {
  _osr_odr_config.odr = (uint8_t)odr;
  _last_status = bmp5_set_osr_odr_press_config(&_osr_odr_config, &_bmp5_dev);
  return _last_status == BMP5_OK;
}
 
/*!
 * @brief Set power mode
 * @param powermode Power mode setting
 * @return True on success, False on failure
 */
bool BMP5xx::setPowerMode(bmp5xx_powermode_t powermode) {
  _last_status =
      bmp5_set_power_mode((enum bmp5_powermode)powermode, &_bmp5_dev);
  return _last_status == BMP5_OK;
}
 
/*!
 * @brief Get temperature oversampling setting
 * @return Current temperature oversampling
 */
bmp5xx_oversampling_t BMP5xx::getTemperatureOversampling(void) {
  struct bmp5_osr_odr_press_config config;
  memset(&config, 0, sizeof(config));
  _last_status = bmp5_get_osr_odr_press_config(&config, &_bmp5_dev);
  return (bmp5xx_oversampling_t)config.osr_t;
}
 
/*!
 * @brief Get pressure oversampling setting
 * @return Current pressure oversampling
 */
bmp5xx_oversampling_t BMP5xx::getPressureOversampling(void) {
  struct bmp5_osr_odr_press_config config;
  memset(&config, 0, sizeof(config));
  _last_status = bmp5_get_osr_odr_press_config(&config, &_bmp5_dev);
  return (bmp5xx_oversampling_t)config.osr_p;
}
 
/*!
 * @brief Get IIR filter coefficient
 * @return Current IIR filter coefficient
 */
bmp5xx_iir_filter_t BMP5xx::getIIRFilterCoeff(void) {
  struct bmp5_iir_config config;
  memset(&config, 0, sizeof(config));
  _last_status = bmp5_get_iir_config(&config, &_bmp5_dev);
  return (bmp5xx_iir_filter_t)config.set_iir_p;
}
 
/*!
 * @brief Get output data rate
 * @return Current output data rate
 */
bmp5xx_odr_t BMP5xx::getOutputDataRate(void) {
  struct bmp5_osr_odr_press_config config;
  memset(&config, 0, sizeof(config));
  _last_status = bmp5_get_osr_odr_press_config(&config, &_bmp5_dev);
  return (bmp5xx_odr_t)config.odr;
}
 
/*!
 * @brief Get power mode
 * @return Current power mode
 */
bmp5xx_powermode_t BMP5xx::getPowerMode(void) {
  enum bmp5_powermode powermode = BMP5_POWERMODE_STANDBY;
  _last_status = bmp5_get_power_mode(&powermode, &_bmp5_dev);
  return (bmp5xx_powermode_t)powermode;
}
 
/*!
 * @brief Enable/disable pressure measurement
 * @param enable True to enable pressure, false to disable
 * @return True on success, False on failure
 */
bool BMP5xx::enablePressure(bool enable) {
  _osr_odr_config.press_en = enable ? BMP5_ENABLE : BMP5_DISABLE;
  _last_status = bmp5_set_osr_odr_press_config(&_osr_odr_config, &_bmp5_dev);
  return _last_status == BMP5_OK;
}
 
/*!
 * @brief Check if new sensor data is ready
 * @return True if new data is available, false otherwise
 */
bool BMP5xx::dataReady(void) {
  uint8_t int_status = 0;
  _last_status = bmp5_get_interrupt_status(&int_status, &_bmp5_dev);
  if (_last_status != BMP5_OK) {
    return false;
  }
  // Check if data ready interrupt is asserted
  return (int_status & BMP5_INT_ASSERTED_DRDY) != 0;
}
 
/*!
 * @brief Configure interrupt pin settings and sources
 * @param mode Interrupt mode (pulsed or latched)
 * @param polarity Interrupt polarity (active high or low)
 * @param drive Interrupt drive (push-pull or open-drain)
 * @param sources Interrupt sources (can be combined with bitwise OR)
 * @param enable Enable or disable interrupt pin
 * @return True if configuration was successful, false otherwise
 */
bool BMP5xx::configureInterrupt(bmp5xx_interrupt_mode_t mode,
                                bmp5xx_interrupt_polarity_t polarity,
                                bmp5xx_interrupt_drive_t drive,
                                uint8_t sources, bool enable) {
  // Configure interrupt pin settings first
  enum bmp5_intr_en_dis int_enable =
      enable ? BMP5_INTR_ENABLE : BMP5_INTR_DISABLE;
 
  _last_status = bmp5_configure_interrupt(
      (enum bmp5_intr_mode)mode, (enum bmp5_intr_polarity)polarity,
      (enum bmp5_intr_drive)drive, int_enable, &_bmp5_dev);
  if (_last_status != BMP5_OK) {
    return false;
  }
 
  // Configure interrupt sources after pin settings
  struct bmp5_int_source_select int_source_select;
  memset(&int_source_select, 0, sizeof(int_source_select));
  int_source_select.drdy_en =
      (sources & BMP5XX_INTERRUPT_DATA_READY) ? BMP5_ENABLE : BMP5_DISABLE;
  int_source_select.fifo_full_en =
      (sources & BMP5XX_INTERRUPT_FIFO_FULL) ? BMP5_ENABLE : BMP5_DISABLE;
  int_source_select.fifo_thres_en =
      (sources & BMP5XX_INTERRUPT_FIFO_THRESHOLD) ? BMP5_ENABLE : BMP5_DISABLE;
  int_source_select.oor_press_en =
      (sources & BMP5XX_INTERRUPT_PRESSURE_OUT_OF_RANGE) ? BMP5_ENABLE
                                                         : BMP5_DISABLE;
 
  _last_status = bmp5_int_source_select(&int_source_select, &_bmp5_dev);
 
  return _last_status == BMP5_OK;
}
 
/* ---------------------------------------------------------------------- */
/* Bosch API bus callbacks - pico-sdk hardware/spi                         */
/* ---------------------------------------------------------------------- */
 
/**************************************************************************/
/*!
    @brief  SPI read callback for the Bosch API
    @param  reg_addr Register address to read from
    @param  reg_data Buffer to store read data
    @param  len Number of bytes to read
    @param  intf_ptr Pointer to a bmp5xx_spi_intf_t
    @return BMP5_INTF_RET_SUCCESS on success, -1 on error
*/
/**************************************************************************/
BMP5_INTF_RET_TYPE BMP5xx::spi_read(uint8_t reg_addr, uint8_t* reg_data,
                                    uint32_t len, void* intf_ptr) {
  bmp5xx_spi_intf_t* intf = (bmp5xx_spi_intf_t*)intf_ptr;
  if (!intf || !intf->spi || (len && !reg_data)) {
    return -1;
  }
 
  uint8_t cmd = reg_addr | 0x80;
 
  uint32_t irq_state = save_and_disable_interrupts();
  cs_select(intf->cs_pin);
  int written = spi_write_blocking(intf->spi, &cmd, 1);
  int read = 0;
  if (written == 1 && len > 0) {
    read = spi_read_blocking(intf->spi, 0x00, reg_data, (size_t)len);
  }
  cs_deselect(intf->cs_pin);
  restore_interrupts(irq_state);
 
  if (written != 1 || read != (int)len) {
    return -1;
  }
 
  return BMP5_INTF_RET_SUCCESS;
}
 
/**************************************************************************/
/*!
    @brief  SPI write callback for the Bosch API
    @param  reg_addr Register address to write to
    @param  reg_data Buffer containing data to write
    @param  len Number of bytes to write
    @param  intf_ptr Pointer to a bmp5xx_spi_intf_t
    @return BMP5_INTF_RET_SUCCESS on success, -1 on error
*/
/**************************************************************************/
BMP5_INTF_RET_TYPE BMP5xx::spi_write(uint8_t reg_addr, const uint8_t* reg_data,
                                     uint32_t len, void* intf_ptr) {
  bmp5xx_spi_intf_t* intf = (bmp5xx_spi_intf_t*)intf_ptr;
  if (!intf || !intf->spi || (len && !reg_data)) {
    return -1;
  }
 
  uint8_t cmd = reg_addr & 0x7F; // Clear the read bit for a write
 

  uint32_t irq_state = save_and_disable_interrupts();
  cs_select(intf->cs_pin);
  int written = spi_write_blocking(intf->spi, &cmd, 1);
  int payload = 0;
  if (written == 1 && len > 0) {
    payload = spi_write_blocking(intf->spi, reg_data, (size_t)len);
  }
  cs_deselect(intf->cs_pin);
  restore_interrupts(irq_state);
 
  if (written != 1 || payload != (int)len) {
    return -1;
  }
 
  sleep_us(200);
 
  return BMP5_INTF_RET_SUCCESS;
}
 
/**************************************************************************/
/*!
    @brief  Delay callback for the Bosch API
    @param  us Microseconds to delay
    @param  intf_ptr Pointer to interface (unused)
*/
/**************************************************************************/
void BMP5xx::delay_usec(uint32_t us, void* intf_ptr) {
  (void)intf_ptr; // Unused parameter
  sleep_us(us);
}
 