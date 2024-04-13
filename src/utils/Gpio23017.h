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
 * \file Gpio23017.h
 *
 * Arduino driver wrapper for our Gpio API fior the MCP23017 I2C GPIO Extender.
 *
 * @author Balazs Racz
 * @date 26 March 2024
 */

#ifndef _UTILS_GPIO23017_H_
#define _UTILS_GPIO23017_H_

#include <Adafruit_MCP23X17.h>

#include "utils/Executor.h"
#include "utils/Gpio.h"
#include "utils/Timer.h"

class Gpio23017 : public Gpio, public Executable {
 public:
  /// One chip has 16 channels.
  static constexpr uint16_t NUM_PINS = 16;

  /// Time period of polling the GPIO inputs.
  static constexpr uint16_t POLL_MSEC = 30;

  Gpio23017(gpio_pin_t start_pin_num, uint8_t address = MCP23XXX_ADDR,
            TwoWire& w = Wire)
      : start_pin_(start_pin_num), i2c_address_(address), wire_instance_(&w) {
    Executor::instance()->add(this);
    GpioRegistry::instance()->register_obj(this, start_pin_num,
                                           Count{NUM_PINS});
  }

  void begin() override {
    init_ok_ = driver_.begin_I2C(i2c_address_, wire_instance_);
    if (!init_ok_) {
      LOG(LEVEL_ERROR, "Failed to start MCP23017 with address 0x%x gpio %d",
          i2c_address_, start_pin_);
    }
    tm_.start_drifting(POLL_MSEC);
    for (unsigned p = 0; p < NUM_PINS; ++p) {
      if (dir_output_ & (1u << p)) {
        driver_.pinMode(p, OUTPUT);
      } else {
        driver_.pinMode(p, INPUT_PULLUP);
      }
    }
    begin_done_ = true;
  }

  void loop() override {
    if (tm_.check()) {
      input_states_ = driver_.readGPIOAB();
    }
  }

  void write(gpio_pin_t pin, bool value) const override {
    auto p = pin - start_pin_;
    uint16_t mask = 1u << p;
    uint16_t v = value ? mask : 0;
    if ((output_states_ & mask) != v) {
      driver_.digitalWrite(p, value ? HIGH : LOW);
    }
    output_states_ = (output_states_ & ~mask) | v;
  }
  bool read(gpio_pin_t pin) const override {
    auto p = pin - start_pin_;
    uint16_t mask = 1u << p;
    return !!(input_states_ & mask);
  }
  void set_output(gpio_pin_t pin) const override {
    auto p = pin - start_pin_;
    dir_output_ |= (1u << p);
    if (begin_done_) {
      driver_.pinMode(p, OUTPUT);
    }
  }
  void set_input(gpio_pin_t pin) const override {
    auto p = pin - start_pin_;
    dir_output_ &= ~(1u << p);
    if (begin_done_) {
      driver_.pinMode(p, INPUT_PULLUP);
    }
  }

  uint16_t input_states() {
    return input_states_;
  }
  bool ok() {
    return init_ok_;
  }
  
 private:
  /// Backing driver that talks through i2c to the chip.
  mutable Adafruit_MCP23X17 driver_;
  /// Registered first pin.
  gpio_pin_t start_pin_;
  /// I2C address of the device
  uint8_t i2c_address_;
  /// Last polled state of the input pins.
  uint16_t input_states_{0};
  /// Last set state of the output pins.
  mutable uint16_t output_states_{0};
  /// Direction bits. For each bit: bit is 0 = input, bit is 1 = output.
  mutable uint16_t dir_output_{0};
  /// true if begin was already called. THis is important due to delayed I2C
  /// initialization.
  bool begin_done_{false};
  /// True when the initialization returned success.
  bool init_ok_{false};
  /// I2C bus instance.
  TwoWire* wire_instance_;
  /// Timer for polling the inputs.
  Timer tm_;
};

#endif  // _UTILS_GPIO23017_H_
