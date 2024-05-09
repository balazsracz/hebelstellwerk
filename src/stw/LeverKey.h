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
 * \file LeverKey.h
 *
 * Simulates a key that can be taken out when a lever is in a specific
 * position.
 *
 * @author Balazs Racz
 * @date 9 May 2024
 */

#ifndef _STW_LEVERKEY_H_
#define _STW_LEVERKEY_H_

#include "stw/LockTable.h"
#include "stw/TurnoutLever.h"
#include "stw/Types.h"

/// LeverKey represents a lock which fixes a given Lever in one of the end
/// positions. The key can be removed from the lock only if the lever is in the
/// given endposition. If the key is removed, the lock keeps the lever in said
/// endposition.
///
/// The removal of a key is commanded by a gpio input.
///
/// Currently only keys on turnout levers are supported.
class LeverKey : public Executable {
 public:
  /// @param end_position is like WeichePlus(id) or WeicheMinus(id).
  /// @param key_in a GPIO representing when the key is removed.
  /// @param key_in_inverted if key_in_inverted is false, then HIGH (true)
  /// return of the Gpio means that the key was removed and the lever shall be
  /// locked.
  LeverKey(LockTableEntry end_position, gpio_pin_t key_in, bool key_in_inverted)
      : key_in_(key_in, key_in_inverted, GPIO_INPUT) {
    switch (end_position.type_) {
      case TURNOUT_PLUS:
        dir_ = TurnoutLever::PLUS;
        break;
      case TURNOUT_MINUS:
        dir_ = TurnoutLever::MINUS;
        break;
      default:
        DIE("Lever Key is only supported for turnout plus or minus endpoints.");
    }
    turnout_ = TurnoutRegistry::instance()->get((TurnoutId)end_position.arg_);

    Executor::instance()->add(this);
  }

 private:
  enum State : uint8_t {
    /// The lever is not in the right position.
    PRECONDITION_MISSED,
    /// The lever is in the right position, it is possible to take the key.
    UNLOCKED,
    /// The key is gone, the lever should be locked.
    LOCKED,
    /// The key is gone but the lever is not in the right position.
    UNEXPECTED_KEY
  };

  void begin() override {}

  void loop() override {
    switch (state_) {
      case PRECONDITION_MISSED: {
        if (turnout_->get_direction() == dir_) {
          state_ = UNLOCKED;
          LOG(LEVEL_INFO, "Lever key is free.");
          return;
        }
        if (key_in_.read()) {
          state_ = UNEXPECTED_KEY;
          LOG(LEVEL_INFO, "Lever key is removed unexpectedly.");
        }
        break;
      }
      case UNEXPECTED_KEY: {
        if (!key_in_.read()) {
          state_ = PRECONDITION_MISSED;
          LOG(LEVEL_INFO, "Lost lever key is retuened.");
        }
        break;
      }
      case UNLOCKED: {
        if (key_in_.read()) {
          turnout_->add_lock();
          state_ = LOCKED;
          LOG(LEVEL_INFO, "Lever key is removed, locking lever.");
        } else if (turnout_->get_direction() != dir_) {
          state_ = PRECONDITION_MISSED;
          LOG(LEVEL_INFO, "Lever key is not free anymore.");
        }
        break;
      }
      case LOCKED: {
        if (!key_in_.read()) {
          turnout_->remove_lock();
          state_ = UNLOCKED;
          LOG(LEVEL_INFO, "Lever key is returned.");
        }
        break;
      }
    }
  }

  TurnoutLever* turnout_{nullptr};
  TurnoutLever::Direction dir_;
  GpioAccessor key_in_;
  State state_{PRECONDITION_MISSED};
};

#endif  // _STW_LEVERKEY_H_
