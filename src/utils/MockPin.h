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
 * \file MockPin.h
 *
 * Helper classes for writing unit tests.
 *
 * @author Balazs Racz
 * @date 18 Mar 2024
 */

#ifndef _UTILS_MOCKPIN_H_
#define _UTILS_MOCKPIN_H_

#include "gmock/gmock.h"
#include "utils/Gpio.h"
#include "utils/Pwm.h"

#ifndef GTEST
#error this class is test-only
#endif

class MockGpio : public Gpio {
 public:
  MOCK_CONST_METHOD2(write, void(gpio_pin_t, bool));
  MOCK_CONST_METHOD1(read, bool(gpio_pin_t));

  MOCK_CONST_METHOD1(set_output, void(gpio_pin_t));
  MOCK_CONST_METHOD1(set_input, void(gpio_pin_t));
};

class MockPwm : public Pwm {
 public:
  MOCK_CONST_METHOD0(tick_per_msec, tick_t());
  MOCK_CONST_METHOD0(tick_per_period, tick_t());

  MOCK_CONST_METHOD2(write, void(pwm_pin_t, tick_t));
};

class FakeGpio : public Gpio {
 public:
  FakeGpio(gpio_pin_t start_pin, uint16_t num_pins = 1)
      : start_pin_(start_pin), num_pins_(num_pins) {
    state_.reset(new bool[num_pins]);
    memset(state_.get(), 0, num_pins);
    is_output_.resize(num_pins_);
    GpioRegistry::instance()->register_obj(this, start_pin_, Count{num_pins_});
  }

  void write(gpio_pin_t pin, bool value) const override {
    check_valid_pin(pin);
    state_[pin - start_pin_] = value;
  }
  bool read(gpio_pin_t pin) const override {
    check_valid_pin(pin);
    return state_[pin - start_pin_];
  }
  void set_output(gpio_pin_t pin) const override {
    check_valid_pin(pin);
    is_output_[pin - start_pin_] = true;
  }
  void set_input(gpio_pin_t pin) const override {
    check_valid_pin(pin);
    is_output_[pin - start_pin_] = false;
  }

  // These functions only exist on the fake for testing.

  /// Check if a pin has been set to output or input.
  ///
  /// @param pin pin number (global value).
  ///
  /// @return true if the pin is output.
  ///
  bool get_output(gpio_pin_t pin) {
    check_valid_pin(pin);
    return is_output_[pin - start_pin_];
  }

  std::unique_ptr<bool[]> state_;
  mutable std::vector<bool> is_output_;

 private:
  void check_valid_pin(gpio_pin_t p) const {
    bool valid = (p >= start_pin_ && p < (start_pin_ + num_pins_));
    if (!valid) {
      DIE("not valid pin");
    }
  }
  gpio_pin_t start_pin_;
  uint16_t num_pins_;
};

#endif  // _UTILS_MOCKPIN_H_
