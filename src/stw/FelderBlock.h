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
 * \file FelderBlock.h
 *
 * Class implementing a Block that has the Felderblock user interface elements.
 *
 * @author Balazs Racz
 * @date 6 Apr 2024
 */

#ifndef _STW_FELDERBLOCK_H_
#define _STW_FELDERBLOCK_H_

#include "stw/Block.h"
#include "stw/I2CBlock.h"
#include "utils/Timer.h"

class FelderBlock : public Block {
 public:
  FelderBlock(I2CBlockInterface* iface, BlockId id, gpio_pin_t track_detector_pin,
              bool track_detector_inverted, gpio_pin_t route_lock_button_pin,
              bool route_lock_button_inverted, gpio_pin_t route_locked_lamp_pin,
              bool route_locked_lamp_inverted)
      : Block(id, track_detector_pin, track_detector_inverted,
              route_lock_button_pin, route_lock_button_inverted,
              route_locked_lamp_pin, route_locked_lamp_inverted) {}

  // Setup functions. Allows defining the GPIO mapping for this given block in
  // a way that is a bit more readable, than just having a super long list of
  // numbers in the constructor. The return values should be combined with |,
  // and at the end the resulting bitmask should equal the constant
  // EXPECTED_SETUP.

  
  uint16_t set_vorblock_taste(gpio_pin_t pin, bool inverted) {
    vorblock_taste_.setup(pin, inverted, GPIO_INPUT);
    return 1u<<0;
  }
  uint16_t set_ruckblock_taste(gpio_pin_t pin, bool inverted) {
    ruckblock_taste_.setup(pin, inverted, GPIO_INPUT);
    return 1u<<1;
  }
  uint16_t set_abgabe_taste(gpio_pin_t pin, bool inverted) {
    abgabe_taste_.setup(pin, inverted, GPIO_INPUT);
    return 1u<<2;
  }
  uint16_t set_kurbel(gpio_pin_t pin, bool inverted) {
    kurbel_.setup(pin, inverted, GPIO_INPUT);
    return 1u<<3;
  }

  uint16_t set_anfangsfeld(gpio_pin_t pin, bool inverted) {
    anfangsfeld_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u<<4;
  }
  uint16_t set_endfeld(gpio_pin_t pin, bool inverted) {
    endfeld_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u<<5;
  }
  uint16_t set_erlaubnisfeld(gpio_pin_t pin, bool inverted) {
    erlaubnisfeld_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u<<6;
  }

  uint16_t set_signalhaltmelder(gpio_pin_t pin, bool inverted) {
    signalhaltmelder_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u<<7;
  }
  uint16_t set_storungsmelder(gpio_pin_t pin, bool inverted) {
    storungsmelder_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u<<8;
  }
  
  /// These bits should be set after all the setup is done.
  static constexpr uint16_t EXPECTED_SETUP = (1u<<9) - 1;

  enum class State : uint8_t {
    /// No permission, track is free. 
    IN_FREE,
    /// No permission, track is occupied. 
    IN_OCC,
    /// Have permission, track is free. 
    OUT_FREE,
    /// Have permission, track is occupied. 
    OUT_OCC
  };
  
private:
  void begin() override {
    Block::begin();
    tm_.start_drifting(13);
  }

  void loop() override {
    Block::loop();
    // Sets the output GPIOs.
    switch(state_) {
      case State::IN_FREE: {
        /// @todo is the Anfangsfeld red when the Erlaubnisfeld is red?
        anfangsfeld_.write(WHITE);
        endfeld_.write(WHITE);
        erlaubnisfeld_.write(RED);
        break;
      }
      case State::IN_OCC: {
        /// @todo is the Anfangsfeld red when the Erlaubnisfeld is red?
        anfangsfeld_.write(WHITE);
        endfeld_.write(RED);
        erlaubnisfeld_.write(RED);
        break;
      }
      case State::OUT_FREE: {
        anfangsfeld_.write(WHITE);
        endfeld_.write(WHITE);
        erlaubnisfeld_.write(WHITE);
        break;
      }
      case State::OUT_OCC: {
        anfangsfeld_.write(RED);
        endfeld_.write(WHITE);
        erlaubnisfeld_.write(WHITE);
        break;
      }
    }
    // The rest of the logic we only do occasionally.
    if (!tm_.check()) {
      return;
    }
    
  }

  
  /// Hardware connection for the actual block PCB.
  I2CBlockInterface* iface_;

  /// Button to signal an outgoing train to the next station, i.e., set the
  /// track to busy with an outgoing train.
  DelayedGpioAccessor vorblock_taste_;
  /// Button to signal that an incoming train from the next station has
  /// arrived, i.e., to set a busy incoming track to clear.
  DelayedGpioAccessor ruckblock_taste_;
  /// Button to hand off the permission to the next station, i.e., set an
  /// outgoing track to an incoming track.
  DelayedGpioAccessor abgabe_taste_;

  /// Input for the crank. For an analog crank, this should be a virtual Gpio
  /// using the AnalogGpio class.
  DelayedGpioAccessor kurbel_;

  static constexpr bool RED = false;
  static constexpr bool WHITE = true;
  
  /// Output for a red/white field showing that an outgoing track is
  /// clear. Value is 1 for white, 0 for red.
  DelayedGpioAccessor anfangsfeld_;
  /// Output for a red/white field showing that an incoming track is
  /// clear. Value is 1 for white, 0 for red.
  DelayedGpioAccessor endfeld_;
  /// Output for a red/white field showing that we have the permission to send
  /// trains to the track. Value is 1 for white, 0 for red.
  DelayedGpioAccessor erlaubnisfeld_;

  /// Output for a red field showing that the incoming signal is showing the
  /// stop aspect. 1 for red, 0 for dark.
  DelayedGpioAccessor signalhaltmelder_;
  /// Output for a field showing that there is an error. Value is 1 for error,
  /// 0 for no error.
  DelayedGpioAccessor storungsmelder_;

  Timer tm_;
  
  /// Current state of the block.
  State state_ {State::IN_OCC};

  
};

#endif // _STW_FELDERBLOCK_H_
