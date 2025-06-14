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
 * \file AnalogDemux.h
 *
 * Gpio implementation for multiple buttons with one analog input sensing
 * different voltages.
 *
 * @author Balazs Racz
 * @date 11 May 2024
 */

#ifndef _UTILS_ANALOGDEMUX_H_
#define _UTILS_ANALOGDEMUX_H_

#ifdef ARDUINO

#include <Arduino.h>

#endif

#if defined(ANALOGGPIO_TEST) || defined(ARDUINO)

class AnalogDemux : public Gpio, private Executable {
 public:
  /// Constructor.
  ///
  /// @param register_pin is the gpio pin number from which the object will be
  /// registered. The number of entries to register is the same as the number
  /// of values presented.
  /// @param arduino_pin is the arduino analog pin number (e.g. A0, A1, ...)
  /// which we will be reading as input.
  /// @param centers the native (computed optimal) values from the ADC input
  /// for each single button. The highest ADC value should be first. All the
  /// rest of the values have to be decreasing ordered.
  /// @param count how many buttons
  AnalogDemux(gpio_pin_t register_pin, int arduino_pin, const int16_t* centers,
              uint8_t count)
      : count_(count),
        register_pin_(register_pin),
        arduino_pin_(arduino_pin),
        centers_(centers) {
    Executor::instance()->add(this);
    GpioRegistry::instance()->register_obj(this, register_pin, Count{count});
  }

  void begin() override {
    pinMode(arduino_pin_, INPUT);
    tm_.start_drifting(POLL_MSEC);
  }

  void loop() override {
    if (tm_.check()) {
      last_read_ = analogRead(arduino_pin_);
    }
  }

  void write(gpio_pin_t pin, bool value) const override {
    DIE("Analog Pin does not support write.");
  }

  bool read(gpio_pin_t pin) const override {
    int idx = pin - register_pin_;
    if (idx < 0 || idx >= count_) {
      return false;
    }
    if (idx == 0) {
      if (last_read_ > ((centers_[0] + ADC_MAX) / 2)) {
        return false;
      }
    } else if (last_read_ > ((centers_[idx] + centers_[idx - 1]) / 2)) {
      return false;
    }
    if (idx < (count_ - 1) &&
        last_read_ < ((centers_[idx] + centers_[idx + 1]) / 2)) {
      return false;
    }
    return true;
  }
  
  void set_output(gpio_pin_t pin) const override {
    DIE("Analog Pin does not support output.");
  }
  void set_input(gpio_pin_t pin) const override {}

 private:
  static constexpr unsigned POLL_MSEC = 30;
  static constexpr int16_t ADC_MAX = 1024;
  uint8_t count_;
  /// The pin we registered for.
  gpio_pin_t register_pin_;
  /// The analog pin to read in.
  int arduino_pin_;
  const int16_t* centers_;
  /// Last read analog value.
  int16_t last_read_ = 1024;
  /// Helper for the polling.
  Timer tm_;
};

#endif  // defined ARDUINO

#endif  // _UTILS_ANALOGDEMUX_H_
