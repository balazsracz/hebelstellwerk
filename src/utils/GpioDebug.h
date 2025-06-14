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
 * \file GpioDebug.h
 *
 * Helper class that prints debug messages when Gpio values change.
 *
 * @author Balazs Racz
 * @date 14 Jun 2025
 */

#ifndef _UTILS_GPIODEBUG_H_
#define _UTILS_GPIODEBUG_H_

#include <Arduino.h>

#include "utils/Executor.h"
#include "utils/Gpio.h"
#include "utils/Macros.h"
#include "utils/Timer.h"

/// Helper class for defining which GPIOs to output debug information about.
struct GpioDebugInstance {
  gpio_pin_t pin;
  const char* name;
  bool inverted = false;
};

class GpioDebug : public Executable {
 public:
  template <typename C>
  GpioDebug(const C& pins) {
    for (const auto& e : pins) {
      gpios_.emplace_back(e.pin, e.name, e.inverted);
    }
    Executor::instance()->add(this);
  }

  void begin() override {
    for (auto& e : gpios_) {
      e.last_value_ = e.gpio_->read(e.num_);
    }
    tm_.start_drifting(1);
  }

  void loop() override {
    if (!tm_.check()) {
      return;
    }
    for (auto& e : gpios_) {
      bool curr_value = e.gpio_->read(e.num_);
      if (curr_value != e.last_value_) {
        e.last_value_ = curr_value;
        if (e.inverted_) {
          curr_value = !curr_value;
        }
        if (Serial) {
          Serial.printf("Pin %s is %s\n", e.name_, curr_value ? "on" : "off");
        }
      }
    }
  }

 private:
  struct Entry {
    Entry(gpio_pin_t pin, const char* name, bool inverted)
        : name_(name),
          num_(pin),
          inverted_(inverted),
          gpio_(GpioRegistry::instance()->get(num_)) {
      ASSERT(gpio_ != nullptr);
    }
    const char* name_;
    const gpio_pin_t num_;
    const bool inverted_;
    const Gpio* const gpio_;
    bool last_value_{false};
  };
  /// Stores the gpios we care about with their last state.
  std::vector<Entry> gpios_;
  /// This timer is set to 1 msec so that we don't spend too much time in this
  /// code.
  Timer tm_;
};

#endif  // _UTILS_GPIODEBUG_H_
