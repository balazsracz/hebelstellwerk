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
 * \file Registry.h
 *
 * Abstract Singleton class to keep track of instances of some objects.
 *
 * @author Balazs Racz
 * @date 9 Mar 2024
 */

#ifndef _UTILS_REGISTRY_H_
#define _UTILS_REGISTRY_H_

#include <algorithm>
#include <vector>

#include "utils/Macros.h"
#include "utils/Atomic.h"
#include "utils/Instance.h"

template<typename reg_num_t, class Obj, class DefaultObj>
class AbstractRegistry : private Atomic {
 public:
  /// Registers an object for a given range of pin number.
  ///
  /// @param obj object that implements the given I/O.
  /// @param start first number that should be registered to this object.
  /// @param end (exclusive) last of the range to register.
  ///
  void register_obj(Obj* obj, reg_num_t start, reg_num_t end) {
    AtomicHolder h(this);
    entries_.push_back(Entry{start, end, obj});
    std::sort(entries_.begin(), entries_.end());
    for (uint16_t i = 0; i < entries_.size() - 1; ++i) {
      if (entries_[i].end_ > entries_[i + 1].start_) {
        DIE("Overlapping registrations.");
      }
    }
  }

  /// Retrieves a registered object.
  /// @param pin the pin number. (It should be registered.)
  /// @return the object that was registered for this pin number.
  Obj* get(reg_num_t pin) {
    auto* p = get_or_null(pin);
    if (!p) {
      DIE("Requested object not found");
      p = Instance<DefaultObj>::get();
    }
    return p;
  }

  /// Retrieves a registered object, if the pin number is known.
  /// @param pin the pin number.
  /// @return the object that was registered for this pin number, or
  /// nullptr if the number is not known.
  Obj* get_or_null(reg_num_t pin) {
    if (pin == (reg_num_t)-1) {
      return Instance<DefaultObj>::get();
    }
    auto it = std::upper_bound(entries_.begin(), entries_.end(), pin, Comp());
    if (it != entries_.begin())--it;
    if (it == entries_.end()) {
      return nullptr;
    }
    if (it->start_ <= pin && it->end_ > pin) {
      return it->obj_;
    }
    return nullptr;
  }
  
 private:
  /// This structure keeps a registry entry for a Obj object.
  struct Entry {
    reg_num_t start_;
    reg_num_t end_;
    Obj* obj_;
    bool operator<(const Entry& o) { return start_ < o.start_; }
  };

  /// Comparison object for std::lower_bound.
  struct Comp {
    bool operator()(reg_num_t a, const Entry& e) {
      return a < e.start_;
    }
  };

  /// Sorted vector of registered objects.
  std::vector<Entry> entries_;
};

#endif  // _UTILS_REGISTRY_H_
