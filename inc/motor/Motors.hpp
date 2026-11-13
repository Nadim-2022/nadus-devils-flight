/*!
 * @file Motors.h
 *
 * ESC / motor output using the RP2040 / RP2350 hardware PWM.
 *
 * Generates standard servo-style ESC pulses (default 1000-2000 us) at a
 * configurable rate (default 400 Hz, supported by most multirotor ESCs).
 * Also works for OneShot125 by setting the pulse range to 125-250 us.
 *
 * Safety:
 *  - After begin() every output sends the minimum pulse (motors stopped),
 *    which is also what most ESCs need to arm.
 *  - While disarmed, every output is forced to the minimum pulse no matter
 *    what throttle is requested.
 *  - When armed, throttle 0.0 gives the idle pulse (props spinning slowly),
 *    throttle 1.0 gives the maximum pulse.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

#include "pico/stdlib.h"

class Motors {
public:
  static constexpr unsigned kMaxMotors = 8;

  struct Config {
    uint pins[kMaxMotors] = {0};
    unsigned count = 0;      ///< number of motors (1..8)
    uint32_t freq_hz = 400;  ///< ESC update rate
    uint16_t min_us = 1000;  ///< motors stopped (also the arming pulse)
    uint16_t idle_us = 1080; ///< armed, throttle 0: props spin slowly
    uint16_t max_us = 2000;  ///< full throttle
  };

  explicit Motors(const Config &cfg);

  /*!
   * @brief Set up the PWM hardware and output the minimum pulse on all
   *        motors. Call once, then wait ~2-3 s for the ESCs to initialise
   *        before arming.
   * @return false if the configuration is invalid (see lastError())
   */
  bool begin(void);

  /*! @brief Allow the motors to spin. Throttle is reset to 0 (idle). */
  void arm(void);
  /*! @brief Stop all motors immediately (minimum pulse). */
  void disarm(void);
  bool isArmed(void) const { return _armed; }

  /*!
   * @brief Set one motor's throttle.
   * @param index    motor number, 0 .. count-1
   * @param throttle 0.0 (idle) .. 1.0 (full); values outside are clamped
   */
  void setThrottle(unsigned index, float throttle);

  /*! @brief Set all motors to the same throttle (0.0 .. 1.0). */
  void setAll(float throttle);

  /*!
   * @brief Set all motors at once from an array of `count` values,
   *        e.g. the output of your mixer.
   */
  void setThrottles(const float *throttles);

  float throttle(unsigned index) const;
  uint16_t pulseUs(unsigned index) const; ///< pulse currently sent

  unsigned count(void) const { return _cfg.count; }
  const char *lastError(void) const { return _error; }

private:
  void writePulse(unsigned index, uint16_t us);

  Config _cfg;
  bool _ready = false;
  bool _armed = false;
  float _ticksPerUs = 1.0f;
  float _throttle[kMaxMotors] = {0};
  uint16_t _pulse[kMaxMotors] = {0};
  uint _slice[kMaxMotors] = {0};
  uint _channel[kMaxMotors] = {0};
  const char *_error = "";
};

#endif