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
 * \file ArduinoGpio.h
 *
 * Gpio implementation for the Arduino's builtin pins and APIs.
 *
 * @author Balazs Racz
 * @date 9 Mar 2024
 */

#ifndef _UTILS_ARDUINOGPIO_H_
#define _UTILS_ARDUINOGPIO_H_

#ifdef ARDUINO

#include <Arduino.h>

#include "utils/Gpio.h"

class ArduinoGpio : public Gpio {
 public:
  ArduinoGpio() {
    GpioRegistry::instance()->register_obj(this, 0, Count{NUM_DIGITAL_PINS});
  }
  void write(gpio_pin_t pin, bool value) const override {
    digitalWrite((int)pin, value ? HIGH : LOW);
  }
  bool read(gpio_pin_t pin) const override { return digitalRead((int)pin); }
  void set_output(gpio_pin_t pin) const override { pinMode((int)pin, OUTPUT); }
  void set_input(gpio_pin_t pin) const override { pinMode((int)pin, INPUT); }
};

#endif  // Arduino

#endif  // _UTILS_ARDUINOGPIO_H_
