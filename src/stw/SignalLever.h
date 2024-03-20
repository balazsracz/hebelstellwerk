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
 * \file SignalLever.h
 *
 * Implements the state machine for a single signal lever.
 *
 * @author Balazs Racz
 * @date 20 Mar 2024
 */

#ifndef _STW_SIGNALLEVER_H__
#define _STW_SIGNALLEVER_H__

#include "utils/Types.h"
#include "utils/Registry.h"
#include "utils/Singleton.h"
#include "utils/Executor.h"
#include "utils/Gpio.h"
#include "utils/Logging.h"


enum SignalId : uint8_t;

enum SignalAspect : uint8_t {
  HP0 = 0,
  ASPECT_H = HP0,
  HP1,
  ASPECT_F1 = HP1,
  HP2,
  ASPECT_F2 = HP2
};

/// Computes a registration entry for the signal registry.
///
/// @param id signal identifier.
/// @param aspect which aspect for that signal
///
/// @return a registry entry index
static int16_t signal_registry_idx(SignalId id, SignalAspect aspect) {
  return ((int)id) * 10 + (int)aspect;
}

class SignalLever;

/// Partial template specialization to not have to specify a default turnout to
/// the registry.
template<> class Instance<SignalLever> {
 public:
  static SignalLever* get() { return nullptr; }
};


class SignalRegistry
    : public AbstractRegistry<int16_t, SignalLever, SignalLever>,
      public Singleton<SignalRegistry> {};

class SignalLever : private Executable {
 public:
  SignalLever(SignalId signal, SignalAspect aspect, gpio_pin_t lever_input,
              bool lever_invert, gpio_pin_t lock_output, bool lock_invert)
      : id_(signal),
        aspect_(aspect),
        input_invert_(lever_invert),
        output_invert_(lock_invert),
        lever_input_(lever_input),
        lock_output_(lock_output) {
    input_ = GpioRegistry::instance()->get(lever_input_);
    lock_ = GpioRegistry::instance()->get(lock_output_);
    Executor::instance()->add(this);
    auto idx = signal_registry_idx(signal, aspect);
    SignalRegistry::instance()->register_obj(this, idx, idx+1);
  }

  enum class State {
    STOP_LOCKED,
    STOP,
    PROCEED,
  };

  /// @return the currently presented aspect based on this lever. This is not
  /// necessarily the aspect that the signal shows, in case there are two
  /// levers for the signal.
  SignalAspect current_aspect() {
    switch (state_) {
      case State::STOP_LOCKED:
      case State::STOP:
        return HP0;
      case State::PROCEED:
        break;
    }
    return aspect_;
  }

  /// @return true if the lever is locked.
  bool is_locked() { return state_ == State::STOP_LOCKED; }

  /// @return true if this lever is currently set for the signal to proceed.
  bool is_proceed() { return state_ == State::PROCEED; }

  /// Called by the Route locking logic, when the Route is set and locked, and
  /// the signal lever should be released for setting the signal to proceed.
  void unlock() {
    if (state_ != State::STOP_LOCKED) {
      LOG(LEVEL_ERROR, "Signal %d/Hp%d unlock while state %d", id_, aspect_,
          (int)state_);
    }
    LOG(LEVEL_INFO, "Signal %d/Hp%d unlock", id_, aspect_);
    state_ = State::STOP;
  }

  void begin() override {
    bool input_is_proceed = get_lever_is_proceed();
    state_ = input_is_proceed ? State::PROCEED : State::STOP_LOCKED;
  }
  
  void loop() override {
    LOG(LEVEL_INFO, "state %d locked %d output_invert %d", (int)state_, is_locked(), output_invert_);
    lock_->write(lock_output_, is_locked() ? !output_invert_ : output_invert_);
    if (!is_locked()) {
      bool input_is_proceed = get_lever_is_proceed();
      if (state_ == State::STOP && input_is_proceed) {
        LOG(LEVEL_INFO, "Signal %d/Hp%d is proceed", id_, aspect_);
        state_ = State::PROCEED;
      } else if (state_ == State::PROCEED && !input_is_proceed) {
        LOG(LEVEL_INFO, "Signal %d/Hp%d is stop+lock", id_, aspect_);
        state_ = State::STOP_LOCKED;
      }
    }
  }

 private:
  /// @return true if the lever is set to proceed aspect.
  bool get_lever_is_proceed() {
    bool input_is_proceed = input_->read(lever_input_);
    if (input_invert_) input_is_proceed = !input_is_proceed;
    return input_is_proceed;
  }

  /// Number of the signal we represent.
  SignalId id_;
  /// Which aspect of this signal we represent (Hp1 or Hp2).
  SignalAspect aspect_;

  /// true: input low is signal ON (proceed), input high is signal OFF (stop).
  /// false: input low is signal OFF (stop), input high is signal ON (proceed).
  bool input_invert_;

  /// false: output high is locked. true: output low is locked.
  bool output_invert_;

  /// Gpio pin for the input from the signal lever.
  gpio_pin_t lever_input_;

  /// Gpio pin for the output controlling the lock.
  gpio_pin_t lock_output_;

  /// Internal signal state.
  State state_;

  /// Gpio object for the lever input.
  const Gpio* input_;
  /// Gpio object for the lock output.
  const Gpio* lock_;
};

#endif  // _STW_SIGNALLEVER_H__
