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
 * \file GpioRegistry.h
 *
 * Singleton class to keep track of instances of Gpio objects.
 *
 * @author Balazs Racz
 * @date 9 Mar 2024
 */

#ifndef _UTILS_GPIOREGISTRY_H_
#define _UTILS_GPIOREGISTRY_H_

#include <algorithm>
#include <vector>

#include "utils/Atomic.h"
#include "utils/Gpio.h"
#include "utils/Singleton.h"
#include "utils/Types.h"

class GpioRegistry : public Singleton<GpioRegistry>, private Atomic {
 public:
  /// Registers a Gpio object for a given range of pin number.
  ///
  /// @param obj GPIO object that implements the given I/O.
  /// @param start first pin number that should be registered to this object.
  /// @param end (exclusive) last of the range to register.
  ///
  void register_gpio(const Gpio* obj, gpio_pin_t start, gpio_pin_t end) {
    // entries_.emplace_back({.start_ = start, .end_ = end, .obj_ = obj});
    AtomicHolder h(this);
    entries_.push_back(Entry{start, end, obj});
    std::sort(entries_.begin(), entries_.end());
    for (uint16_t i = 0; i < entries_.size() - 1; ++i) {
      if (entries_[i].end_ > entries_[i + 1].start_) {
        DIE("Overlapping GPIO registrations.");
      }
    }
  }

  /// Retrieves a registered GPIO object.
  /// @param pin the pin number. (It should be registered.)
  /// @return the Gpio object that was registered for this pin number.
  const Gpio* get(gpio_pin_t pin) {
    auto* p = get_or_null(pin);
    if (!p) {
      DIE("Requested GPIO not found");
      p = Instance<DummyGpio>::get();
    }
    return p;
  }

  /// Retrieves a registered GPIO object, if the pin number is known.
  /// @param pin the pin number.
  /// @return the Gpio object that was registered for this pin number, or
  /// nullptr if it is not known.
  const Gpio* get_or_null(gpio_pin_t pin) {
    auto it = std::upper_bound(entries_.begin(), entries_.end(), pin, Comp());
    if (it != entries_.begin()) --it;
    if (it == entries_.end()) {
      return nullptr;
    }
    if (it->start_ <= pin && it->end_ > pin) {
      return it->obj_;
    }
    return nullptr;
  }
  
 private:
  /// This structure keeps a registry entry for a Gpio object.
  struct Entry {
    gpio_pin_t start_;
    gpio_pin_t end_;
    const Gpio* obj_;
    bool operator<(const Entry& o) { return start_ < o.start_; }
  };

  /// Comparison object for std::lower_bound.
  struct Comp {
    bool operator()(gpio_pin_t a, const GpioRegistry::Entry& e) {
      return a < e.start_;
    }
  };

  /// Sorted vector of registered GPIOs.
  std::vector<Entry> entries_;
};

#endif  // _UTILS_GPIOREGISTRY_H_