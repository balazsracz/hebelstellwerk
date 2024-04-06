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

class FelderBlock : public Block {
 public:
  FelderBlock(I2CBlockInterface* iface, BlockId id, gpio_pin_t track_detector_pin,
              bool track_detector_inverted, gpio_pin_t route_lock_button_pin,
              bool route_lock_button_inverted, gpio_pin_t route_locked_lamp_pin,
              bool route_locked_lamp_inverted)
      : Block(id, track_detector_pin, track_detector_inverted,
              route_lock_button_pin, route_lock_button_inverted,
              route_locked_lamp_pin, route_locked_lamp_inverted) {}

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


  /// These bits should be set after all the setup is done.
  static constexpr uint16_t EXPECTED_SETUP = (1u<<5) - 1;

private:
  /// Hardware connection for the actual block PCB.
  I2CBlockInterface* iface_;

  DelayedGpioAccessor vorblock_taste_;
  DelayedGpioAccessor ruckblock_taste_;
  DelayedGpioAccessor abgabe_taste_;

  DelayedGpioAccessor kurbel_;
  
};

#endif // _STW_FELDERBLOCK_H_
