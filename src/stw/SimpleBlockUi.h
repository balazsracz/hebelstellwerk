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
 * \file SimpleBlockUi.h
 *
 * Block user interface that does not do a lot of checking.
 *
 * @author Balazs Racz
 * @date 12 May 2024
 */

#ifndef _STW_SIMPLEBLOCKUI_H_
#define _STW_SIMPLEBLOCKUI_H_

#include "stw/GlobalCommand.h"
#include "stw/I2CBlock.h"
#include "utils/Executor.h"
#include "utils/Gpio.h"
#include "utils/Timer.h"

class SimpleBlockUi : public Executable {
 public:
  SimpleBlockUi(I2CBlockInterface* block) : block_(block) {
    Executor::instance()->add(this);
  }

  uint16_t set_vorblock_taste(gpio_pin_t pin, bool inverted) {
    vorblock_taste_.setup(pin, inverted, GPIO_INPUT);
    return 1u << 0;
  }
  uint16_t set_ruckblock_taste(gpio_pin_t pin, bool inverted) {
    ruckblock_taste_.setup(pin, inverted, GPIO_INPUT);
    return 1u << 1;
  }
  uint16_t set_abgabe_taste(gpio_pin_t pin, bool inverted) {
    abgabe_taste_.setup(pin, inverted, GPIO_INPUT);
    return 1u << 2;
  }

  uint16_t set_anfangsfeld(gpio_pin_t pin, bool inverted) {
    anfangsfeld_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u << 3;
  }
  uint16_t set_endfeld(gpio_pin_t pin, bool inverted) {
    endfeld_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u << 4;
  }
  uint16_t set_erlaubnisfeld(gpio_pin_t pin, bool inverted) {
    erlaubnisfeld_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u << 5;
  }

  void begin() override {}

  void loop() override {
    auto m = Executor::instance()->millis();
    if (m != last_millis_) {
      last_millis_ = m;
    } else {
      return;
    }
    // Updates output fields.
    auto state = block_->get_status();
    if (state & BlockBits::ERROR) {
      erlaubnisfeld_.write(RED);
      anfangsfeld_.write(RED);
      endfeld_.write(RED);
    } else {
      if (state & BlockBits::TRACK_OUT) {
        erlaubnisfeld_.write(WHITE);
      } else {
        erlaubnisfeld_.write(RED);
      }
      if (state & BlockBits::OUT_BUSY) {
        anfangsfeld_.write(RED);
      } else {
        anfangsfeld_.write(WHITE);
      }
      if (state & BlockBits::IN_BUSY) {
        endfeld_.write(RED);
      } else {
        endfeld_.write(WHITE);
      }
    }
    // Handles unknown startup status.
    if ((state & BlockBits::STARTUP) && !(state & BlockBits::TRACK_OUT) &&
        !(state & BlockBits::HANDOFF)) {
      block_->reset_out();
    }

    // Checks which button is pressed.
    Button current = BTN_NONE;
    if (abgabe_taste_.read()) {
      current = BTN_ABGABE;
    } else if (vorblock_taste_.read()) {
      current = BTN_VORBLOCK;
    } else if (ruckblock_taste_.read()) {
      current = BTN_RUCKBLOCK;
    }

    // Runs debouncing state machine.
    switch (state_) {
      case IDLE: {
        if (current != BTN_NONE) {
          state_ = BUTTON_PRESS_WAITING;
          btn_pressed_ = current;
          tm_.start_oneshot(BTN_REACT_MSEC);
        }
        break;
      }
      case BUTTON_PRESS_WAITING: {
        if (current != btn_pressed_) {
          state_ = IDLE;
        } else if (tm_.check()) {
          perform_action();
          state_ = BUTTON_RELEASE_WAITING;
        }
        break;
      }
      case BUTTON_RELEASE_WAITING: {
        // The user has to release all buttons for a certain period of time for
        // us to arm again.
        if (current != BTN_NONE) {
          tm_.start_oneshot(BTN_CLEAR_MSEC);
        } else if (tm_.check()) {
          state_ = IDLE;
        }
        break;
      }
    }
  }

  /// These bits should be set after all the setup is done.
  static constexpr uint16_t EXPECTED_SETUP = (1u << 6) - 1;

 private:
  /// Performs the action denoted by the current button press.
  void perform_action() {
    switch (btn_pressed_) {
      case BTN_VORBLOCK:
        if (global_is_unlocked() ||
            (block_->get_status() & BlockBits::TRACK_OUT)) {
          // We didn't check the busy bit, because it does not matter. We
          // should be able to repeat a Vorblocken even if we just sent one.
          block_->vorblocken();
        }
        break;
      case BTN_RUCKBLOCK:
        if (global_is_unlocked() ||
            (block_->get_status() & BlockBits::HANDOFF)) {
          // We didn't check the busy bit, because it does not matter. We
          // should be able to repeat a Ruckblocken even if we just sent one.
          block_->ruckblocken();
        }
        break;
      case BTN_ABGABE: {
        auto status = block_->get_status();
        bool okay = false;
        if (global_is_unlocked()) okay = true;
        // Regular handoff, when we have the track and it is free.
        if ((status & BlockBits::TRACK_OUT) &&
            !(status & BlockBits::OUT_BUSY)) {
          okay = true;
        }
        // Repeat handoff, when we just handed off and the track is still free.
        if ((status & BlockBits::HANDOFF) &&
            !(status & BlockBits::IN_BUSY)) {
          okay = true;
        }
        if (okay) {
          block_->abgabe();
        }
        break;
      }
      default:
        // no nothing
        break;
    }
  }

  static constexpr bool RED = false;
  static constexpr bool WHITE = true;

  /// How long does the user need to hold a button to perform the respective
  /// action.
  static constexpr unsigned BTN_REACT_MSEC = 700;
  /// How long the user has to release all buttons for us to take use input
  /// again.
  static constexpr unsigned BTN_CLEAR_MSEC = 300;

  enum State : uint8_t { IDLE, BUTTON_PRESS_WAITING, BUTTON_RELEASE_WAITING };

  enum Button : uint8_t { BTN_VORBLOCK, BTN_ABGABE, BTN_RUCKBLOCK, BTN_NONE };

  /// Button to signal an outgoing train to the next station, i.e., set the
  /// track to busy with an outgoing train. True when the user presses the
  /// button.
  DelayedGpioAccessor vorblock_taste_;
  /// Button to signal that an incoming train from the next station has
  /// arrived, i.e., to set a busy incoming track to clear.  True when the user
  /// presses the button.
  DelayedGpioAccessor ruckblock_taste_;
  /// Button to hand off the permission to the next station, i.e., set an
  /// outgoing track to an incoming track.  True when the user presses the
  /// button.
  DelayedGpioAccessor abgabe_taste_;

  /// Output for a red/white field showing that an outgoing track is
  /// clear. Value is 1 for white, 0 for red.
  DelayedGpioAccessor anfangsfeld_;
  /// Output for a red/white field showing that an incoming track is
  /// clear. Value is 1 for white, 0 for red.
  DelayedGpioAccessor endfeld_;
  /// Output for a red/white field showing that we have the permission to send
  /// trains to the track. Value is 1 for white, 0 for red.
  DelayedGpioAccessor erlaubnisfeld_;

  /// Block accecssor interface.
  I2CBlockInterface* block_;
  /// Helper for timed operations.
  Timer tm_;
  /// Simple timer that looks for doing an operation once per millisecond.
  uint32_t last_millis_{0};
  /// Which button is pressed currently.
  Button btn_pressed_{BTN_NONE};
  /// State machine for debouncing.
  State state_{IDLE};
};

#endif  // _STW_SIMPLEBLOCKUI_H_
