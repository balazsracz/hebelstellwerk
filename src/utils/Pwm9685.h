/** \copyright
 * Copyright (c) 2024, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
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
 * \file Pwm9685.h
 *
 * Arduino driver wrapper for our Pwm API.
 *
 * @author Balazs Racz
 * @date 26 March 2024
 */

#ifndef _UTILS_PWM9685_H_
#define _UTILS_PWM9685_H_

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#include "utils/Executor.h"
#include "utils/Pwm.h"

class PWM9685 : public Pwm, public Executable {
 public:
  /// One chip has 16 channels.
  static constexpr uint16_t NUM_PINS = 16;

  /// Constructor
  ///
  /// @param start_pin_num pwm_pin to register the first channel for. The next
  /// consecutive 16 pwm_pin will mean channel 0..15 on the outputs.
  /// @param address I2C address, default is 0x40.
  /// @param w Instance of I2C bus from Arduino device.
  PWM9685(pwm_pin_t start_pin_num, uint8_t address = 0x40, TwoWire& w = Wire)
      : driver_(address, w), start_pin_(start_pin_num) {
    Executor::instance()->add(this);
    PwmRegistry::instance()->register_obj(this, start_pin_, Count{NUM_PINS});
  }

  void begin() override {
    if (!driver_.begin()) {
      LOG(LEVEL_ERROR, "Failed to start PCA9685 with pwm pin %d", start_pin_);
    } else {
      // We want ~1000 count to be 1 msec. 4096 count would then be 4 msec. This
      // turns into 250 Hz frequency.
      driver_.setPWMFreq(250/4);
    }
  }

  void set_freq(uint32_t osc_freq, uint16_t pwm_freq, uint16_t tick_per_msec) {
    driver_.setOscillatorFrequency(osc_freq);
    driver_.setPWMFreq(pwm_freq);
    tick_per_msec_ = tick_per_msec;
  }

  tick_t tick_per_msec() const override { return tick_per_msec_; }
  tick_t tick_per_period() const override { return 4096; }

  void write(pwm_pin_t pin, tick_t count_high) const override {
    driver_.setPWM(pin - start_pin_, 0, count_high);
  }

  void loop() override {}

 private:
  /// Backing driver that talks through i2c to the chip.
  mutable Adafruit_PWMServoDriver driver_;
  /// Registered first pin.
  pwm_pin_t start_pin_;
  uint16_t tick_per_msec_{1024/4};
};

#endif  // _UTILS_PWM9685_H_
