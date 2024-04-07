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

#include "stw/Types.h"
#include "utils/Executor.h"
#include "utils/Gpio.h"
#include "utils/Logging.h"
#include "utils/Registry.h"
#include "utils/Singleton.h"
#include "utils/Types.h"
#include "utils/Timer.h"

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
template <>
class Instance<SignalLever> {
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
        lock_wait_(false),
        lever_(lever_input, lever_invert, GPIO_INPUT),
        lock_(lock_output, lock_invert, GPIO_OUTPUT) {
    Executor::instance()->add(this);
    auto idx = signal_registry_idx(signal, aspect);
    SignalRegistry::instance()->register_obj(this, idx);
  }

  enum class State : uint8_t {
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
    lock_wait_ = false;
  }

  /// Called by the Route locking logic, when the Route is released while the
  /// signal is in stop but unlocked. It will lock the signal in stop.
  void lock() {
    if (state_ != State::STOP) {
      LOG(LEVEL_ERROR, "Signal %d/Hp%d tried to lock while state %d", id_,
          aspect_, (int)state_);
      return;
    }
    LOG(LEVEL_INFO, "Signal %d/Hp%d lock", id_, aspect_);
    wait_and_lock();
  }

  /// Called by the executor once at startup. Sets up the initial state of the
  /// state machine. If the state says lock, then waits one second before
  /// engaging the lock.
  void begin() override {
    lock_.write(false);
    if (get_lever_is_proceed()) {
      state_ = State::PROCEED;
    } else {
      wait_and_lock();
    }
  }

  void loop() override {
    if (lock_wait_) {
      // Waiting one second before engaging the lock. During this wait period
      // we will be watching for the lever to flip back to proceed.
      if (!is_locked()) {
        // We were unlocked, typically by the route lever.
        lock_wait_ = false;
      } else if (get_lever_is_proceed()) {
        // Restarts the timer.
        wait_and_lock();
      } else if (lock_timer_.check()) {
        // Lock timer expired.
        lock_wait_ = false;
        LOG(LEVEL_INFO, "Signal %d/Hp%d lock after timer", id_, aspect_);
      }
    }
    lock_.write(is_locked() && !lock_wait_);
    if (!is_locked()) {
      bool input_is_proceed = get_lever_is_proceed();
      if (state_ == State::STOP && input_is_proceed) {
        LOG(LEVEL_INFO, "Signal %d/Hp%d is proceed", id_, aspect_);
        state_ = State::PROCEED;
      } else if (state_ == State::PROCEED && !input_is_proceed) {
        LOG(LEVEL_INFO, "Signal %d/Hp%d is stop+lock", id_, aspect_);
        wait_and_lock();
      }
    }
  }

 private:
  /// 1 second after we enter the locked state we engage the lock servo.
  static constexpr uint16_t LOCK_ENGAGE_DELAY_MSEC = 1000;
  
  /// @return true if the lever is set to proceed aspect.
  bool get_lever_is_proceed() {
    bool input_is_proceed = lever_.read();
    return input_is_proceed;
  }

  /// Sets the state to Stop+Locked and starts the timer after which the lock
  /// GPIO will be engaged.
  void wait_and_lock() {
    state_ = State::STOP_LOCKED;
    lock_timer_.start_oneshot(LOCK_ENGAGE_DELAY_MSEC);
    lock_wait_ = true;
  }

  /// Number of the signal we represent.
  SignalId id_;
  /// Which aspect of this signal we represent (Hp1 or Hp2).
  SignalAspect aspect_;

  /// True when we just entered stop+locked state and we are waiting for the
  /// timer.
  bool lock_wait_ : 1;
  /// We've seen a flip in the Hebel during the last second of the locking
  /// process. Will cause the locking process to restart.
  bool lock_cancel_ : 1;

  /// Times 1 second wait between entering the lock state and actually moving
  /// the lock output.
  Timer lock_timer_;

  /// Helper object for the input Gpio.  When inverted is true: input low is
  /// signal ON (proceed), input high is signal OFF (stop).  false: input low
  /// is signal OFF (stop), input high is signal ON (proceed).
  GpioAccessor lever_;

  /// Helper object for the output Gpio. When inverted is false: output high is
  /// locked. true: output low is locked.
  GpioAccessor lock_;

  /// Internal signal state.
  State state_;
};

#endif  // _STW_SIGNALLEVER_H__
