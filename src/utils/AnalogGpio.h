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
 * \file AnalogGpio.h
 *
 * Gpio implementation for the Arduino's analog input.
 *
 * @author Balazs Racz
 * @date 6 Apr 2024
 */

#ifndef _UTILS_ANALOGGPIO_H_
#define _UTILS_ANALOGGPIO_H_

#ifdef ARDUINO

#include <Arduino.h>

#endif

#if defined(ANALOGGPIO_TEST) || defined(ARDUINO)

#include "utils/Gpio.h"
#include "utils/Timer.h"
#include "utils/Logging.h"

/// Analog input GPIO. Simulates a digital input pin by performing analog reads
/// of an arduino input. The input has to be above a given threshold of voltage
/// for a given amount of time to be considered active. Active pins read high,
/// inactive pins read low.
class AnalogGpio : public Gpio, private Executable {
 public:
  /// Constructor.
  ///
  /// @param register_pin is the gpio pin number under which the object will be
  /// registered.
  /// @param arduino_pin is the arduino analog pin number (e.g. A0, A1, ...)
  /// which we will be reading as input.
  /// @param threshold_analog is the minimum value of analogRead() output to
  /// consider the pin as active. This is usually in the range of 0..1023, but
  /// might depends on the MCU platform.
  /// @param length_msec. How many msec long the input has to be above the
  /// threshold to consider it active.
  AnalogGpio(gpio_pin_t register_pin, int arduino_pin, int16_t threshold_analog,
             int16_t length_msec)
      : arduino_pin_(arduino_pin),
        length_msec_(length_msec),
        threshold_analog_(threshold_analog)

  {
    Executor::instance()->add(this);
    GpioRegistry::instance()->register_obj(this, register_pin);
  }
  void begin() override {
    pinMode(arduino_pin_, INPUT);
    tm_.start_periodic(POLL_MSEC);
  }
  void loop() override {
    while (tm_.check()) {
      auto current = analogRead(arduino_pin_);
      if (current >= threshold_analog_) {
        msec_above_ += POLL_MSEC;
      } else {
        msec_above_ = 0;
      }
    }
  }

  void write(gpio_pin_t pin, bool value) const override {
    DIE("Analog Pin does not support write.");
  }
  bool read(gpio_pin_t pin) const override {
    LOG(LEVEL_VERBOSE, "analog_gpio read %d >= %d", msec_above_, length_msec_);
    return msec_above_ >= length_msec_;
  }
  void set_output(gpio_pin_t pin) const override {
    DIE("Analog Pin does not support output.");
  }
  void set_input(gpio_pin_t pin) const override {}

 private:
  /// How frequently we should do the analog read.
  static constexpr uint16_t POLL_MSEC = 39;

  /// Helper for the polling.
  Timer tm_;

  /// Number of analog input pin.
  const int arduino_pin_;

  /// How many msec should the threshold be exceeded to report high.
  const int16_t length_msec_;

  /// How many msec should the threshold be exceeded to report high.
  const int16_t threshold_analog_;

  /// How many msec did we measure to be above the threshold.
  uint16_t msec_above_{0};
};

#endif  // Arduino

#endif  // _UTILS_ANALOGGPIO_H_
