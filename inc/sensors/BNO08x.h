#ifndef BNO08X_H
#define BNO08X_H


#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

extern "C" {
#include "../../external/sh2/sh2.h"
#include "../../external/sh2/sh2_SensorValue.h"
#include "../../external/sh2/sh2_err.h"
}


/* Additional activities not listed in the SH-2 lib */
#define PAC_ON_STAIRS 8     ///< Activity code for being on stairs
#define PAC_OPTION_COUNT 9  ///< Number of activity classifier options
 
/*!
 * @brief Driver for a BNO08x connected to the Pico over SPI.
 *
 * Note: the SH-2 library keeps global state, so only one BNO08x
 * instance can be active at a time.
 */
class BNO08x {
public:
  /*!
   * @param spi       spi0 or spi1
   * @param sck_pin   GPIO used for SCK
   * @param mosi_pin  GPIO used for MOSI (SPI TX) -> BNO DI (SA0/MOSI)
   * @param miso_pin  GPIO used for MISO (SPI RX) -> BNO SDA (MISO)
   * @param cs_pin    GPIO used for chip select (driven manually)
   * @param int_pin   GPIO connected to BNO INT (active low)
   * @param reset_pin GPIO connected to BNO RST, or -1 if not connected
   * @param wake_pin  GPIO connected to BNO P0 (PS0/WAKE), or -1 if P0 is
   *                  tied to 3V3. Recommended: without it the sensor can only
   *                  be made to accept a command while it is idle by
   *                  resetting it (which needs reset_pin).
   */
  BNO08x(spi_inst_t *spi, uint sck_pin, uint mosi_pin, uint miso_pin,
         uint cs_pin, uint int_pin, int reset_pin = -1, int wake_pin = -1);
 
  /*!
   * @brief Set up SPI/GPIO, reset the sensor and open the SH-2 interface.
   * @param baudrate SPI clock in Hz (BNO08x supports up to 3 MHz)
   * @return true on success
   */
  bool begin(uint32_t baudrate = 3000000);
 
  void hardwareReset(void);
  bool wasReset(void);
 
  bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us = 10000);
  bool getSensorEvent(sh2_SensorValue_t *value);
  bool calibrate(uint32_t timeout_ms = 0);
  int setDcdAutoSave(bool enabled);
 
  sh2_ProductIds_t prodIds; ///< Product IDs returned by the sensor
 
private:
  // sh2_Hal_t must be the first member so the C callbacks can recover
  // the owning object from the sh2_Hal_t* they are given.
  struct Hal {
    sh2_Hal_t hal;
    BNO08x *owner;
  } _hal;
 
  static constexpr unsigned kMaxTransferIn = 384; ///< Largest SHTP transfer
  static constexpr unsigned kMaxWriteResets = 3;  ///< See halWrite()
 
  // SH-2 HAL callbacks
  static int halOpen(sh2_Hal_t *self);
  static void halClose(sh2_Hal_t *self);
  static int halRead(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                     uint32_t *t_us);
  static int halWrite(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
  static uint32_t halGetTimeUs(sh2_Hal_t *self);
 
  // SH-2 event callbacks
  static void asyncCallback(void *cookie, sh2_AsyncEvent_t *pEvent);
  static void sensorCallback(void *cookie, sh2_SensorEvent_t *pEvent);
 
  static BNO08x *fromHal(sh2_Hal_t *self) {
    return reinterpret_cast<Hal *>(self)->owner;
  }
 
  bool waitForInt(uint32_t timeout_ms);
  int transfer(const uint8_t *tx, unsigned txLen, uint8_t *rx, unsigned rxMax);
  inline void csSelect(void);
  inline void csDeselect(void);
 
  spi_inst_t *_spi;
  uint _sck, _mosi, _miso, _cs, _int;
  int _reset, _wake;
 
  sh2_SensorValue_t *_sensorValue = nullptr;
  volatile bool _gotEvent = false;
  volatile bool _resetOccurred = false;
 
  // A packet the sensor sent while we were writing (SPI is full duplex).
  uint8_t _stash[kMaxTransferIn];
  unsigned _stashLen = 0;
  unsigned _writeResets = 0;
  uint32_t _stashTime = 0;
};
 
#endif