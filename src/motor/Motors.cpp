/*!
 * @file Motors.cpp
 *
 * ESC / motor output using hardware PWM. See Motors.hpp.
 */

#include "motor/Motors.hpp"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include <cstdio>

Motors::Motors(const Config &cfg) : _cfg(cfg) {}

bool Motors::begin(void) {
  _ready = false;
  _armed = false;

  // ---- Check the configuration ----------------------------------------
  if (_cfg.count == 0 || _cfg.count > kMaxMotors) {
    _error = "motor count must be 1..8";
    return false;
  }
  if (!(_cfg.min_us <= _cfg.idle_us && _cfg.idle_us < _cfg.max_us)) {
    _error = "pulses must satisfy min_us <= idle_us < max_us";
    return false;
  }
  if (_cfg.freq_hz == 0 || (uint64_t)_cfg.max_us * _cfg.freq_hz >= 1000000u) {
    _error = "max_us does not fit in one PWM period (lower freq_hz)";
    return false;
  }
  for (unsigned i = 0; i < _cfg.count; i++) {
    _slice[i] = pwm_gpio_to_slice_num(_cfg.pins[i]);
    _channel[i] = pwm_gpio_to_channel(_cfg.pins[i]);
    for (unsigned j = 0; j < i; j++) {
      if (_slice[i] == _slice[j] && _channel[i] == _channel[j]) {
        // Two pins on the same PWM output would always show the same
        // signal, so they can't drive different motors.
        _error = "two motor pins share the same PWM slice/channel";
        return false;
      }
    }
  }

  // ---- Timing -------------------------------------------------------------
  // Pick the smallest clock divider that fits one period into the 16-bit
  // counter; a small divider gives the finest pulse resolution.
  const float sys_hz = (float)clock_get_hz(clk_sys);
  std::printf("System clock: %.3f MHz\n", sys_hz / 1e6f);
  const float period_ticks_div1 = sys_hz / (float)_cfg.freq_hz;
  float div = period_ticks_div1 / 65535.0f;
  if (div < 1.0f) div = 1.0f;
  // The divider has 4 fractional bits; round up to the next 1/16.
  div = (float)((uint32_t)(div * 16.0f + 0.999f)) / 16.0f;
  if (div > 255.9375f) {
    _error = "freq_hz too low for the PWM divider";
    return false;
  }
  const uint32_t wrap = (uint32_t)(period_ticks_div1 / div + 0.5f) - 1u;
  _ticksPerUs = sys_hz / div / 1e6f;

  // ---- Hardware -------------------------------------------------------------
  pwm_config pc = pwm_get_default_config();
  pwm_config_set_clkdiv(&pc, div);
  pwm_config_set_wrap(&pc, (uint16_t)wrap);

  for (unsigned i = 0; i < _cfg.count; i++) {
    gpio_set_function(_cfg.pins[i], GPIO_FUNC_PWM);
  }
  // Start with the stop pulse on every channel before enabling a slice
  for (unsigned i = 0; i < _cfg.count; i++) {
    bool configured = false;
    for (unsigned j = 0; j < i; j++) {
      if (_slice[j] == _slice[i]) configured = true;
    }
    if (!configured) {
      pwm_init(_slice[i], &pc, false);
    }
    _throttle[i] = 0.0f;
    writePulse(i, _cfg.min_us);
  }
  for (unsigned i = 0; i < _cfg.count; i++) {
    pwm_set_enabled(_slice[i], true);
  }
  

  //--------//
  for (unsigned i = 0; i < _cfg.count; i++) {
    const uint s = _slice[i];
    // Read the divider and wrap back from the PWM registers
    const uint32_t div_reg = pwm_hw->slice[s].div;
    const float hw_div = (float)((div_reg >> PWM_CH0_DIV_INT_LSB) & 0xFFu) +
                         (float)(div_reg & PWM_CH0_DIV_FRAC_BITS) / 16.0f;
    const uint32_t hw_top = pwm_hw->slice[s].top;
    const float hw_freq = sys_hz / (hw_div * (float)(hw_top + 1u));

    std::printf("Motor %u: GP%u  slice %u%c  %.2f Hz  pulse %u us\n",
                i, _cfg.pins[i], s, _channel[i] ? 'B' : 'A',
                (double)hw_freq, _pulse[i]);
  }


  _ready = true;
  _error = "";
  return true;
}

void Motors::arm(void) {
  if (!_ready) return;
  for (unsigned i = 0; i < _cfg.count; i++) {
    _throttle[i] = 0.0f;
    writePulse(i, _cfg.idle_us);
  }
  _armed = true;
}

void Motors::disarm(void) {
  _armed = false;
  if (!_ready) return;
  for (unsigned i = 0; i < _cfg.count; i++) {
    _throttle[i] = 0.0f;
    writePulse(i, _cfg.min_us);
  }
}

void Motors::setThrottle(unsigned index, float throttle) {
  if (!_ready || index >= _cfg.count) return;

  if (!(throttle >= 0.0f)) throttle = 0.0f; // also catches NaN
  if (throttle > 1.0f) throttle = 1.0f;
  _throttle[index] = throttle;

  if (!_armed) {
    writePulse(index, _cfg.min_us);
    return;
  }
  const float span = (float)(_cfg.max_us - _cfg.idle_us);
  writePulse(index, (uint16_t)((float)_cfg.idle_us + throttle * span + 0.5f));
}

void Motors::setAll(float throttle) {
  for (unsigned i = 0; i < _cfg.count; i++) setThrottle(i, throttle);
}

void Motors::setThrottles(const float *throttles) {
  if (!throttles) return;
  for (unsigned i = 0; i < _cfg.count; i++) setThrottle(i, throttles[i]);
}

float Motors::throttle(unsigned index) const {
  return index < _cfg.count ? _throttle[index] : 0.0f;
}

uint16_t Motors::pulseUs(unsigned index) const {
  return index < _cfg.count ? _pulse[index] : 0;
}

void Motors::writePulse(unsigned index, uint16_t us) {
  if (us < _cfg.min_us) us = _cfg.min_us;
  if (us > _cfg.max_us) us = _cfg.max_us;
  _pulse[index] = us;
  // The new level takes effect at the start of the next PWM period, so a
  // pulse is never cut short.
  pwm_set_chan_level(_slice[index], _channel[index],
                     (uint16_t)((float)us * _ticksPerUs + 0.5f));
}