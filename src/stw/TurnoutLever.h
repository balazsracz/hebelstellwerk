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
 * \file TurnoutLever.h
 *
 * Implements the state machine for a single turnout lever.
 *
 * @author Balazs Racz
 * @date 19 Mar 2024
 */

#ifndef _STW_TURNOUTLEVER_H_
#define _STW_TURNOUTLEVER_H_

#include "utils/Executor.h"
#include "utils/Gpio.h"
#include "utils/Logging.h"
#include "utils/Registry.h"
#include "utils/Types.h"

enum TurnoutId : uint8_t;

class TurnoutLever;

/// Partial template specialization to not have to specify a default turnout to
/// the registry.
template<> class Instance<TurnoutLever> {
 public:
  static TurnoutLever* get() { return nullptr; }
};

class TurnoutRegistry
    : public AbstractRegistry<TurnoutId, TurnoutLever, TurnoutLever>,
      public Singleton<TurnoutRegistry> {};

class TurnoutLever : private Executable {
 public:
  TurnoutLever(TurnoutId turnout, gpio_pin_t lever_input, bool lever_invert,
               gpio_pin_t lock_output, bool lock_invert)
      : id_(turnout),
        input_invert_(lever_invert),
        output_invert_(lever_invert),
        lever_input_(lever_input),
        lock_output_(lock_output) {
    input_ = GpioRegistry::instance()->get(lever_input_);
    lock_ = GpioRegistry::instance()->get(lock_output_);
    Executor::instance()->add(this);
    TurnoutRegistry::instance()->register_obj(this, turnout,
                                              (TurnoutId)(turnout + 1));
  }

  /// Defines turnout directions.
  enum Direction {
    /// Default state (normal), usually closed.
    PLUS,
    /// Reversed state, usually thrown.
    MINUS
  };

  /// @return which direction the turnout is currently set to.
  Direction get_direction() {
    switch (state_) {
      case State::PLUS:
      case State::PLUS_LOCKED:
        return PLUS;
      case State::MINUS:
      case State::MINUS_LOCKED:
        return MINUS;
    }
    return PLUS;
  }

  /// @return true if the turnout is currently locked.
  bool is_locked() {
    switch (state_) {
      case State::PLUS_LOCKED:
      case State::MINUS_LOCKED:
        return true;
      case State::PLUS:
      case State::MINUS:
        return false;
    }
    return false;
  }

  /// To be called by a Route lever state machine. Adds a lock to this
  /// turnout. It is possible for multiple route levers to lock a given
  /// turnout, provided that they are all aligned on which state the turnout
  /// has to be locked in. The caller has to first verify that the curent
  /// direction of the turnout is appropriate for their route to be set.
  void add_lock() {
    if (0 == lock_count_) {
      LOG(LEVEL_INFO, "Locking Turnout %d", (int)id_);
      switch (state_) {
        case State::PLUS:
        case State::PLUS_LOCKED:
          state_ = State::PLUS_LOCKED;
          break;
        case State::MINUS:
        case State::MINUS_LOCKED:
          state_ = State::MINUS_LOCKED;
          break;
      }
    }
    ++lock_count_;
  }

  /// Removes a lock. Called by a route lever state machine, when the lever is
  /// returned to the neutral position.
  void remove_lock() {
    if (!lock_count_) {
      LOG(LEVEL_ERROR, "Removing lock when count == 0. Turnout %d", (int)id_);
      return;
    }
    if (--lock_count_) {
      return;
    }
    LOG(LEVEL_INFO, "Unlocking Turnout %d", (int)id_);
    switch (state_) {
      case State::PLUS:
      case State::PLUS_LOCKED:
        state_ = State::PLUS;
        break;
      case State::MINUS:
      case State::MINUS_LOCKED:
        state_ = State::MINUS;
        break;
    }
  }

  void begin() override {
    bool input_is_plus = get_lever_is_plus();
    state_ = input_is_plus ? State::PLUS : State::MINUS;
  }

  void loop() override {
    lock_->write(lock_output_, is_locked() ? !output_invert_ : output_invert_);
    if (!is_locked()) {
      bool input_is_plus = get_lever_is_plus();
      if ((get_direction() == PLUS) != input_is_plus) {
        // change input
        state_ = input_is_plus ? State::PLUS : State::MINUS;
        LOG(LEVEL_INFO, "Turnout %d is %c", (int)id_,
            input_is_plus ? '+' : '-');
      }
    }
  }

 private:
  bool get_lever_is_plus() {
    bool input_is_plus = input_->read(lever_input_);
    if (input_invert_) input_is_plus = !input_is_plus;
    return input_is_plus;
  }

  enum class State { PLUS, PLUS_LOCKED, MINUS, MINUS_LOCKED };

  TurnoutId id_;

  /// true: input low is Plus, input high is Minus. false: input low is Minus,
  /// input high is Plus.
  bool input_invert_;

  /// false: output high is locked. true: output low is locked.
  bool output_invert_;

  /// Gpio pin for the input from the turnout lever.
  gpio_pin_t lever_input_;

  /// Gpio pin for the input from the turnout lever.
  gpio_pin_t lock_output_;

  /// Internal turnout state.
  State state_;

  /// How many locks do we have right now.
  uint8_t lock_count_{0};

  /// Gpio object for the lever input.
  const Gpio* input_;
  /// Gpio object for the lock output.
  const Gpio* lock_;
};  // class TurnoutLever

#endif  // _STW_TURNOUTLEVER_H_
