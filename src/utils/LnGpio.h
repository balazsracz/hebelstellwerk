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
 * \file LnGpio.h
 *
 * Gpio adapters for loconet messages.
 *
 * @author Balazs Racz
 * @date 16 May 2024
 */

#ifndef _UTILS_LNGPIO_H_
#define _UTILS_LNGPIO_H_

enum LnGpioType : uint8_t {
  LNGPIO_SWITCH,
  LNGPIO_SENSOR,
};

struct LnGpioDefn {
  LnGpioType type;
  uint16_t address;
};

class LnGpio : public Gpio, public Executable, public Singleton<LnGpio> {
 public:
  /// Constructor
  ///
  /// @param pin First GPIO number that we export.
  /// @param ln loconet bus object 
  /// @param defs pointer to constant array which declares what loconet
  /// addresses to use.
  /// @param count how many pins we have
  LnGpio(gpio_pin_t pin, LocoNetClass* ln, const LnGpioDefn* defs,
         unsigned count)
      : pin_(pin), count_(count), ln_(ln), defs_(defs) {
    last_state_ = new uint32_t[(2 * count + 31) / 32];
    memset(last_state_, 0, (2 * count + 7) / 8);
    Exeuctor::instance()->add(this);
  }

  ~LnGpio() { delete[] last_state_; }

  void write(gpio_pin_t pin, bool value) const override {
    uint16_t ofs = (pin - pin_);
    auto pos = get_pos(ofs);
    if (set_bit(pos, bit, value)) {
      // changed: set dirty
      set_bit(pos, bit << 1, true);  // dirty
    }
  }
  bool read(gpio_pin_t pin) const override {
    uint16_t ofs = (pin - pin_);
    auto pos = get_pos(ofs);
    return *(pos->first) & pos->second;
  }
  void set_output(gpio_pin_t pin) const override {
    // noop really. Loconet stuff is always input-output.
  }
  void set_input(gpio_pin_t pin) const override {
    // noop really. Loconet stuff is always input-output.
  }

 private:
  void set_state(unsigned ofs, bool value) {}

  std::pair<uint32_t*, uint32_t> get_pos(unsigned ofs) {
    ofs <<= 1;
    uint32_t* p = state_ + (ofs >> 5);
    uint32_t bit = ofs & 31;
    return {p, bit};
  }

  inline static bool set_bit(uint32_t* pos, uint32_t bit, bool value) {
    bool ret = value != ((*pos & bit) != 0);
    if (value) {
      *pos |= bit;
    } else {
      *pos &= ~bit;
    }
    return ret;
  }

  /// First GPIO pin number.
  gpio_pin_t pin_;
  /// How many pins we are simulating.
  uint16_t count_;

  /// Pointer to the loconet instance.
  LocoNetClass* ln_;
  /// Array with gpio definitions.
  const LnGpioDefn* defs_;

  /// Array allocated on the heap with two bits for each gpio pin we have. The
  /// first bit is the state. The second bit is 1 when we need to send out an
  /// update for this output.
  uint32_t* state_;
};

#endif  // _UTILS_LNGPIO_H_
