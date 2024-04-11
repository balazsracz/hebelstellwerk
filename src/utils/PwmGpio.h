/** \copyright
 * Copyright (c) 2024, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file PwmGpio.h
 *
 * Adapter to perform a GPIO output on a PWM pin.
 *
 * @author Balazs Racz
 * @date 18 Mar 2024
 */

#ifndef _UTILS_PWMGPIO_H_
#define _UTILS_PWMGPIO_H_

#include "utils/Gpio.h"
#include "utils/Pwm.h"

/// Instantiate this class when driving an on/off object, such as a LED, using
/// a PWM pin. The duty cycle percent will be a parameter for the on and off
/// rates.
class PwmGpio : public Gpio {
 public:
  /// Constructor.
  ///
  /// @param gpio_pin pin number to use for registering this virtual GPIO pin.
  /// @param pwm_pin pin number for the PWM pin to drive for this use-case.
  /// @param pct_off percent (0..100) duty cycle to use when the GPIO is
  /// commanded to off.
  /// @param pct_on percent (0..100) duty cycle to use when the GPIO is
  /// commanded to on.
  PwmGpio(gpio_pin_t gpio_pin, pwm_pin_t pwm_pin, int8_t pct_off, int8_t pct_on)
      : pwm_pin_(pwm_pin), pct_off_(pct_off), pct_on_(pct_on) {
    GpioRegistry::instance()->register_obj(this, gpio_pin);
    pwm_ = PwmRegistry::instance()->get(pwm_pin);
  }

  void set_output(gpio_pin_t pin) const override {
    // Nothing to do, always an output.
  }

  void write(gpio_pin_t pin, bool value) const override {
    if (value != last_state_) {
      last_state_ = value;
      uint32_t count = pwm_->tick_per_period();
      if (last_state_) {
        count *= pct_on_;
      } else {
        count *= pct_off_;
      }
      count /= 100;
      pwm_->write(pwm_pin_, count);
    }
  }

  bool read(gpio_pin_t pin) const override { return last_state_; }

 private:
  /// PWM pin object from the registry.
  const Pwm* pwm_;
  /// Number of the PWM pin to drive.
  pwm_pin_t pwm_pin_;
  /// Percent duty cycle to drive the PWM pin when the gpio is commanded off.
  int8_t pct_off_;
  /// Percent duty cycle to drive the PWM pin when the gpio is commanded on.
  int8_t pct_on_;
  /// Last commanded state of the GPIO pin.
  mutable bool last_state_{false};
};

/// Instantiate this class when driving a dual-color object with two LEDs
/// connected to two PWM pins. In the ON state one LED will be driven, in the
/// OFF state the other. The duty cycle percent will be a parameter for the on
/// and off outputs.
class DualPwmGpio : public Gpio {
 public:
  /// Constructor.
  ///
  /// @param gpio_pin pin number to use for registering this virtual GPIO pin.
  /// @param pwm_pin_off pin number for the PWM pin to drive when the virtual
  /// gpio is off (zero).
  /// @param pwm_pin_on pin number for the PWM pin to drive when the virtual
  /// gpio is on (one).
  /// @param pct_off percent (0..100) duty cycle to use when the GPIO is
  /// commanded to off.
  /// @param pct_on percent (0..100) duty cycle to use when the GPIO is
  /// commanded to on.
  DualPwmGpio(gpio_pin_t gpio_pin, pwm_pin_t pwm_pin_off, pwm_pin_t pwm_pin_on,
              int8_t pct_off, int8_t pct_on)
      : pwm_pin_off_(pwm_pin_off),
        pwm_pin_on_(pwm_pin_on),
        pct_off_(pct_off),
        pct_on_(pct_on) {
    GpioRegistry::instance()->register_obj(this, gpio_pin);
    pwm_off_ = PwmRegistry::instance()->get(pwm_pin_off_);
    pwm_on_ = PwmRegistry::instance()->get(pwm_pin_on_);
  }

  void set_output(gpio_pin_t pin) const override {
    // Nothing to do, always an output.
  }

  void write(gpio_pin_t pin, bool value) const override {
    if (value != last_state_) {
      last_state_ = value;
      uint32_t count = pwm_on_->tick_per_period();
      if (last_state_) {
        count *= pct_on_;
        count /= 100;
        pwm_on_->write(pwm_pin_on_, count);
        pwm_off_->write(pwm_pin_off_, 0);
      } else {
        count *= pct_off_;
        count /= 100;
        pwm_off_->write(pwm_pin_off_, count);
        pwm_on_->write(pwm_pin_on_, 0);
      }
    }
  }

  bool read(gpio_pin_t pin) const override { return last_state_; }

 private:
  /// PWM pin object from the registry.
  const Pwm* pwm_off_;
  /// PWM pin object from the registry.
  const Pwm* pwm_on_;
  /// Number of the PWM pin to drive.
  pwm_pin_t pwm_pin_off_;
  /// Number of the PWM pin to drive.
  pwm_pin_t pwm_pin_on_;
  /// Percent duty cycle to drive the PWM pin when the gpio is commanded off.
  int8_t pct_off_;
  /// Percent duty cycle to drive the PWM pin when the gpio is commanded on.
  int8_t pct_on_;
  /// Last commanded state of the GPIO pin.
  mutable bool last_state_{false};
};

#endif  // _UTILS_PWMGPIO_H_
