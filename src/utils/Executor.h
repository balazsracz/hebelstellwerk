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
 * \file Executor.h
 *
 * Class to control execution in the polling loop.
 *
 * @author Balazs Racz
 * @date 7 Mar 2024
 */

#ifndef _STW_EXECUTOR_H_
#define _STW_EXECUTOR_H_

#include <algorithm>
#include <vector>

#include "utils/Atomic.h"
#include "utils/Executable.h"
#include "utils/Macros.h"
#include "utils/Singleton.h"
#include "utils/Types.h"

class Executor : private Atomic, public Singleton<Executor> {
 public:
  /// Registers an executable. It will first get a begin() call, then
  /// periodically the loop() call, until it is unregistered.
  void add(Executable* e) {
    AtomicHolder h(this);
    pendingStart_.push_back(e);
  }

  /// Unregisters an executable. It will not be called from any future loop()
  /// invocations.
  void remove(Executable* e) {
    AtomicHolder h(this);
    for (uint16_t i = 0; i < entries_.size(); ++i) {
      if (entries_[i] == e) {
        entries_[i] = nullptr;
      }
    }
  }

  void begin() {
    {
      AtomicHolder h(this);
      ASSERT(entries_.empty());
      std::swap(pendingStart_, entries_);
    }
    // Calls the registered handlers' begin().
    for (uint16_t i = 0; i < entries_.size(); ++i) {
      entries_[i]->begin();
    }
  }

  void loop() {
    Executable* new_entry = nullptr;
    {
      // Checks for new registered handlers.
      AtomicHolder h(this);
      if (!pendingStart_.empty()) {
        new_entry = pendingStart_.back();
        pendingStart_.pop_back();
      }
    }
    if (new_entry) {
      new_entry->begin();
      entries_.push_back(new_entry);
    }
    bool found_empty = false;
    // Calls the registered handlers' loop().
    for (uint16_t i = 0; i < entries_.size(); ++i) {
      Executable* e = entries_[i];
      if (e) {
        e->loop();
      } else {
        found_empty = true;
      }
    }
    if (found_empty) {
      // cleanup
      AtomicHolder h(this);
      entries_.erase(std::remove(entries_.begin(), entries_.end(), nullptr),
                     entries_.end());
    }
  }

#ifdef GTEST

  millis_t millis() { return mockTimeMillis_; }

  /// Mock time that can be advanced by unit tests.
  uint32_t mockTimeMillis_{0};

  /// Advances the mock time by a given number of milliseconds.
  /// @param millis how many msec to advance time.
  void advance(uint32_t millis) { mockTimeMillis_ += millis; }

#else
  /// @return the current time in milliseconds since some arbitrary starting
  /// time.
  millis_t millis() { return ::millis(); }
#endif

 private:
  /// Executables that didn't yet have their begin() invoked.
  std::vector<Executable*> pendingStart_;
  /// Executables to call during a loop.
  std::vector<Executable*> entries_;
};  // class Executor

#endif  // _STW_EXECUTOR_H_
