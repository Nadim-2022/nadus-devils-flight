/*!
 *  @file BNO08x.cpp
 *
 *  SPI-only BNO08x driver for the Raspberry Pi Pico (Pico C/C++ SDK).
 *  Ported from the Adafruit BNO08x Arduino library (BSD license).
 */

 
#include "sensors/BNO08x.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

#include "hardware/gpio.h"
#include "debug/debug.hpp"

BNO08x::BNO08x(spi_inst_t *spi, uint sck_pin, uint mosi_pin, uint miso_pin,
               uint cs_pin, uint int_pin, int reset_pin, int wake_pin)
    : _spi(spi), _sck(sck_pin), _mosi(mosi_pin), _miso(miso_pin),
      _cs(cs_pin), _int(int_pin), _reset(reset_pin), _wake(wake_pin) {
  memset(&prodIds, 0, sizeof(prodIds));
  memset(&_hal, 0, sizeof(_hal));
}
 
/*!
 * @brief Set up SPI and GPIO, reset the sensor and open SH-2.
 */
bool BNO08x::begin(uint32_t baudrate) {
  // SPI mode 3 (CPOL=1, CPHA=1), MSB first
  spi_init(_spi, baudrate);
  spi_set_format(_spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  gpio_set_function(_sck, GPIO_FUNC_SPI);
  gpio_set_function(_mosi, GPIO_FUNC_SPI);
  gpio_set_function(_miso, GPIO_FUNC_SPI);
 
  // Chip select is driven manually so it stays low for a whole transfer
  gpio_init(_cs);
  gpio_set_dir(_cs, GPIO_OUT);
  gpio_put(_cs, 1);
 
  gpio_init(_int);
  gpio_set_dir(_int, GPIO_IN);
  gpio_pull_up(_int);
 
  if (_reset >= 0) {
    gpio_init(_reset);
    gpio_set_dir(_reset, GPIO_OUT);
    gpio_put(_reset, 1);
  }
  if (_wake >= 0) {
    gpio_init(_wake);
    gpio_set_dir(_wake, GPIO_OUT);
    gpio_put(_wake, 1); // PS0 must be high at reset for SPI mode
  }
 
  _stashLen = 0;
  _hal.owner = this;
  _hal.hal.open = halOpen;
  _hal.hal.close = halClose;
  _hal.hal.read = halRead;
  _hal.hal.write = halWrite;
  _hal.hal.getTimeUs = halGetTimeUs;
 
  hardwareReset();
 
  // Open SH-2 interface (also registers the non-sensor event handler)
  int status = sh2_open(&_hal.hal, asyncCallback, this);
  if (status != SH2_OK) {
    BNO08X_LOG("sh2_open failed: %d\n", status);
    return false;
  }
  BNO08X_LOG("sh2_open ok, INT=%d\n", (int)gpio_get(_int));
 
  // Check the connection by reading the product IDs
  memset(&prodIds, 0, sizeof(prodIds));
  status = sh2_getProdIds(&prodIds);
  if (status != SH2_OK) {
    BNO08X_LOG("sh2_getProdIds failed: %d, INT=%d\n", status,
               (int)gpio_get(_int));
    return false;
  }
 
  sh2_setSensorCallback(sensorCallback, this);
  return true;
}
 
/*!
 * @brief Reset the device using the reset pin (no-op if none).
 */
void BNO08x::hardwareReset(void) {
  if (_reset < 0) {
    return;
  }
  gpio_put(_reset, 1);
  sleep_ms(10);
  gpio_put(_reset, 0);
  sleep_ms(10);
  gpio_put(_reset, 1);
  sleep_ms(10);
}
 
/*!
 * @brief Check (and clear) whether the sensor has reset. After a reset
 *        all reports must be enabled again.
 */
bool BNO08x::wasReset(void) {
  bool x = _resetOccurred;
  _resetOccurred = false;
  return x;
}
 
/*!
 * @brief Fill `value` with a new report if one is available.
 * @return true if `value` was filled with a new report
 */
bool BNO08x::getSensorEvent(sh2_SensorValue_t *value) {
  _sensorValue = value;
  _gotEvent = false;
 
  sh2_service();
 
  _sensorValue = nullptr;
  return _gotEvent;
}


 
/*!
 * @brief Enable a report type.
 * @param sensorId    The report ID to enable
 * @param interval_us Report interval in microseconds
 */
bool BNO08x::enableReport(sh2_SensorId_t sensorId, uint32_t interval_us) {
  sh2_SensorConfig_t config;
  memset(&config, 0, sizeof(config));
 
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = 0;
  config.reportInterval_us = interval_us;
 
  return sh2_setSensorConfig(sensorId, &config) == SH2_OK;
}
 
/*!
 * @brief Calibrate accelerometer, gyroscope and magnetometer and save the
 *        result to the sensor's flash (loaded automatically at power-up).
 *
 * Move the device while it runs:
 *   gyro  - hold completely still for a few seconds
 *   accel - hold still in ~6 orientations (each face down), ~2 s each
 *   mag   - slowly rotate around all axes (figure-8), away from metal
 *
 * Saves when rotation vector, accel, gyro and mag are all at accuracy 3
 * (High) for 5 s. Blocks until saved or timeout_ms (0 = no timeout).
 * Re-enable your own reports afterwards.
 *
 * @return true if the calibration was saved
 */
bool BNO08x::calibrate(uint32_t timeout_ms) {
  static const char *names[] = {"Unreliable", "Low", "Medium", "High"};
  const uint32_t kStableMs = 5000;

  auto setup = [this]() {
    enableReport(SH2_ROTATION_VECTOR, 10000);           // 100 Hz
    enableReport(SH2_ACCELEROMETER, 50000);             // 20 Hz, accuracy only
    enableReport(SH2_GYROSCOPE_CALIBRATED, 50000);      // 20 Hz, accuracy only
    enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 20000); // 50 Hz, needed by mag cal
    sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
    sh2_setDcdAutoSave(false); // save only when we decide to
  };

  setup();

  uint8_t rv = 0, acc = 0, gyr = 0, mag = 0;
  const uint32_t start = to_ms_since_boot(get_absolute_time());
  uint32_t lastPrint = 0, lastRv = start, highSince = 0;
  bool saved = false;
  sh2_SensorValue_t v;

  while (!saved) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (timeout_ms && now - start >= timeout_ms) {
      printf("[CAL] timed out, NOT saved\n");
      break;
    }

    // A reset (or reports not arriving) wipes the configuration
    if (wasReset() || now - lastRv > 1000) {
      setup();
      lastRv = now;
      highSince = 0;
    }

    while (getSensorEvent(&v)) {
      uint8_t a = v.status & 0x03;
      switch (v.sensorId) {
      case SH2_ROTATION_VECTOR:           rv = a; lastRv = now; break;
      case SH2_ACCELEROMETER:             acc = a; break;
      case SH2_GYROSCOPE_CALIBRATED:      gyr = a; break;
      case SH2_MAGNETIC_FIELD_CALIBRATED: mag = a; break;
      default: break;
      }
    }

    bool allHigh = rv == 3 && acc == 3 && gyr == 3 && mag == 3;
    if (!allHigh) {
      highSince = 0;
    } else if (highSince == 0) {
      highSince = now ? now : 1;
    } else if (now - highSince >= kStableMs) {
      if (sh2_saveDcdNow() == SH2_OK) {
        saved = true;
        printf("[CAL] >>> calibration SAVED <<<\n");
      } else {
        highSince = now; // retry after another stable period
      }
    }

    if (now - lastPrint >= 500) {
      lastPrint = now;
      printf("[CAL] RV: %-10s Accel: %-10s Gyro: %-10s Mag: %-10s\n",
             names[rv], names[acc], names[gyr], names[mag]);
    }
    sleep_ms(1);
  }

  // Switch off the reports used only for calibration (0 = off)
  enableReport(SH2_ACCELEROMETER, 0);
  enableReport(SH2_GYROSCOPE_CALIBRATED, 0);
  enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 0);
  return saved;
}

int BNO08x::setDcdAutoSave(bool enabled) {
  return sh2_setDcdAutoSave(enabled);
}
/**************************** SPI helpers *******************************/
 
inline void BNO08x::csSelect(void) {
  asm volatile("nop \n nop \n nop");
  gpio_put(_cs, 0);
  asm volatile("nop \n nop \n nop");
}
 
inline void BNO08x::csDeselect(void) {
  asm volatile("nop \n nop \n nop");
  gpio_put(_cs, 1);
  asm volatile("nop \n nop \n nop");
}
 
/*!
 * @brief Wait for the (active low) INT line.
 */
bool BNO08x::waitForInt(uint32_t timeout_ms) {
  absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
  while (!time_reached(deadline)) {
    if (!gpio_get(_int)) {
      return true;
    }
    tight_loop_contents();
  }
  return false;
}
 
/*!
 * @brief One full-duplex SHTP transfer inside a single CS assertion.
 *
 * Sends `txLen` bytes from `tx` (may be 0) while receiving whatever the
 * sensor sends. The transfer is as long as the longer of the two.
 *
 * @return length of the received packet stored in `rx`, or 0 if none.
 */
int BNO08x::transfer(const uint8_t *tx, unsigned txLen, uint8_t *rx,
                     unsigned rxMax) {
  uint8_t txHdr[4] = {0, 0, 0, 0};
  uint8_t rxHdr[4];
  if (tx) {
    memcpy(txHdr, tx, std::min(4u, txLen));
  }
 
  csSelect();
  spi_write_read_blocking(_spi, txHdr, rxHdr, 4);
 
  // Packet length, with the "continuation" bit cleared
  unsigned rxLen = ((unsigned)rxHdr[0] | ((unsigned)rxHdr[1] << 8)) & 0x7FFF;
  bool keep = (rx != nullptr) && rxLen >= 4 && rxLen <= rxMax;
  if (keep) {
    memcpy(rx, rxHdr, 4);
  }
 
  // Clock out the rest of the longer packet (capped, in case of garbage)
  unsigned total = std::max(txLen, std::min(rxLen, kMaxTransferIn));
  uint8_t t[32], r[32];
  for (unsigned off = 4; off < total;) {
    unsigned n = std::min(32u, total - off);
    for (unsigned i = 0; i < n; i++) {
      t[i] = (tx && off + i < txLen) ? tx[off + i] : 0;
    }
    spi_write_read_blocking(_spi, t, r, n);
    if (keep && off < rxLen) {
      memcpy(rx + off, r, std::min(n, rxLen - off));
    }
    off += n;
  }
  csDeselect();
 
  return keep ? (int)rxLen : 0;
}
 
/**************************** SH-2 HAL **********************************/
 
int BNO08x::halOpen(sh2_Hal_t *self) {
  BNO08x *dev = fromHal(self);
  dev->_stashLen = 0;
  dev->_writeResets = 0;
  // After reset the sensor pulls INT low when it is ready to talk
  if (!dev->waitForInt(500)) {
    BNO08X_LOG("open: INT did not go low after reset\n");
  }
  return 0;
}
 
void BNO08x::halClose(sh2_Hal_t *self) { (void)self; }
 
int BNO08x::halRead(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                    uint32_t *t_us) {
  BNO08x *dev = fromHal(self);
 
  // First deliver anything received during a previous write
  if (dev->_stashLen) {
    unsigned n = dev->_stashLen;
    dev->_stashLen = 0;
    if (n > len) {
      return 0; // doesn't fit, drop it
    }
    memcpy(pBuffer, dev->_stash, n);
    if (t_us) {
      *t_us = dev->_stashTime;
    }
    return (int)n;
  }
 
  // Non-blocking: nothing to read unless INT is asserted
  if (gpio_get(dev->_int)) {
    return 0;
  }
 
  if (t_us) {
    *t_us = time_us_32();
  }
  return dev->transfer(nullptr, 0, pBuffer, len);
}
 
int BNO08x::halWrite(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  BNO08x *dev = fromHal(self);
 
  // Don't overwrite a received packet that hasn't been read yet;
  // returning 0 makes SH-2 service input and retry.
  if (dev->_stashLen) {
    return 0;
  }
 
  // The BNO08x only accepts an SPI transfer while it holds INT low.
  // With a wake pin we can ask it to do that; without one we have to
  // wait until it has something to send anyway.
  bool ready;
  if (dev->_wake >= 0) {
    gpio_put(dev->_wake, 0);
    ready = dev->waitForInt(200);
    gpio_put(dev->_wake, 1);
  } else {
    ready = dev->waitForInt(200);
  }
 
  if (!ready) {
    // No wake pin and the sensor is idle, so INT will never go low.
    // Like the Adafruit driver, reset it: while it boots it asserts INT
    // and the write goes through on the retry. A reset is reported by
    // wasReset(), so the application re-enables its reports.
    if (dev->_wake < 0 && dev->_reset >= 0 &&
        dev->_writeResets < kMaxWriteResets) {
      BNO08X_LOG("write: INT idle, resetting sensor so it will listen\n");
      dev->_writeResets++;
      dev->hardwareReset();
      return 0; // SHTP services the startup packets, then retries
    }
    BNO08X_LOG("write: timed out waiting for INT\n");
    dev->_writeResets = 0;
    return SH2_ERR_TIMEOUT;
  }
  dev->_writeResets = 0;
 
  // Anything the sensor sends back during the write is kept for halRead
  dev->_stashTime = time_us_32();
  int got = dev->transfer(pBuffer, len, dev->_stash, kMaxTransferIn);
  dev->_stashLen = got > 0 ? (unsigned)got : 0;
 
  return (int)len;
}
 
uint32_t BNO08x::halGetTimeUs(sh2_Hal_t *self) {
  (void)self;
  return time_us_32();
}
 
/**************************** SH-2 events *******************************/
 
void BNO08x::asyncCallback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  BNO08x *dev = static_cast<BNO08x *>(cookie);
  // On reset, flag it so the application can re-enable its reports
  if (pEvent->eventId == SH2_RESET) {
    dev->_resetOccurred = true;
  }
}
 
void BNO08x::sensorCallback(void *cookie, sh2_SensorEvent_t *event) {
  BNO08x *dev = static_cast<BNO08x *>(cookie);
  if (!dev->_sensorValue) {
    return;
  }
  if (sh2_decodeSensorEvent(dev->_sensorValue, event) != SH2_OK) {
    printf("BNO08x - Error decoding sensor event\n");
    return;
  }
  dev->_gotEvent = true;
}
 