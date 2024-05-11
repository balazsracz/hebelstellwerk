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
 * \file PixelGpio.h
 *
 * Virtual gpio (output-only) for a single neopixel in a strip.
 *
 * @author Balazs Racz
 * @date 11 May 2024
 */

#ifndef _UTILS_PIXELGPIO_H_
#define _UTILS_PIXELGPIO_H_

#include "utils/Pixel.h"
#include "utils/Gpio.h"

class PixelGpio : public Gpio {
 public:
  /// Constructor.
  ///
  /// @param px Pixel strip object.
  /// @param pin Registered GPIO pins. This is the first.
  /// @param count How many gpio pins to register.
  /// @param params 3x count entries. each triplet is {pixel strip offset,
  /// color OFF, color ON}.
  PixelGpio(Pixel* px, gpio_pin_t pin, uint16_t count, const uint32_t* params)
      : pin_(pin), count_(count), px_(px), params_(params) {
    GpioRegistry::instance()->register_obj(this, pin_, Count{count_});
  }

  void set_input(gpio_pin_t pin) const override {
    DIE("Pixel Gpio does not support input.");
  }
  void set_output(gpio_pin_t pin) const override {}
  
  void write(gpio_pin_t pin, bool value) const override {
    int delta = pin - pin_;
    ASSERT(delta >= 0 && delta < count_);
    const uint32_t* p = params_ + delta * 3;
    if (value) {
      px_->set(p[0], p[2]);
    } else {
      px_->set(p[0], p[1]);
    }
  }

  bool read(gpio_pin_t pin) const override {
    DIE("Pixel Gpio does not support read.");
  }
 private:
  gpio_pin_t pin_;
  uint16_t count_;
  Pixel* px_;
  const uint32_t* params_;
};

#endif // _UTILS_PIXELGPIO_H_
