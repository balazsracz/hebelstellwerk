/** \copyright
 * Copyright (c) 2025, Balazs Racz
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
 * \file GpioCopier.h
 *
 * Helper class that copies values from one GPIO to another.
 *
 * @author Balazs Racz
 * @date 14 Jun 2025
 */

#ifndef _UTILS_GPIOCOPIER_H_
#define _UTILS_GPIOCOPIER_H_

#include <Arduino.h>

#include "utils/Executor.h"
#include "utils/Gpio.h"
#include "utils/Macros.h"
#include "utils/Timer.h"

/// Helper class for defining which GPIOs to copy.
struct GpioCopyInstance {
  gpio_pin_t src_pin;
  gpio_pin_t dst_pin;
  bool inverted = false;
};

class GpioCopy : public Executable {
 public:
  template <typename C>
  GpioCopy(const C& pins) {
    for (const auto& e : pins) {
      gpios_.emplace_back(e.src_pin, e.dst_pin, e.inverted);
    }
    Executor::instance()->add(this);
  }

  void begin() override {
    tm_.start_drifting(1);
  }

  void loop() override {
    if (!tm_.check()) {
      return;
    }
    for (auto& e : gpios_) {
      bool curr_value = e.src_gpio_->read(e.src_pin_);
      if (e.inverted_) {
        curr_value = !curr_value;
      }
      if (first_run_ || curr_value != e.last_value_) {
        e.last_value_ = curr_value;
        e.dst_gpio_->write(e.dst_pin_, curr_value);
      }
    }
  }

 private:
  struct Entry {
    Entry(gpio_pin_t src_pin, gpio_pin_t dst_pin, bool inverted)
        : src_pin_(src_pin),
          dst_pin_(dst_pin),
          inverted_(inverted),
          src_gpio_(GpioRegistry::instance()->get(src_pin)),
          dst_gpio_(GpioRegistry::instance()->get(dst_pin))
    {
      ASSERT(src_gpio_ != nullptr);
      ASSERT(dst_gpio_ != nullptr);
    }
    const gpio_pin_t src_pin_;
    const gpio_pin_t dst_pin_;
    const bool inverted_;
    const Gpio* const src_gpio_;
    const Gpio* const dst_gpio_;
    // This is the last read value of the src GPIO.
    bool last_value_{false};
  };
  /// Stores the gpios we care about with their last state.
  std::vector<Entry> gpios_;
  /// This timer is set to 1 msec so that we don't spend too much time in this
  /// code.
  Timer tm_;
  /// True during the first loop.
  bool first_run_ = true;
};

#endif  // _UTILS_GPIOCOPIER_H_
