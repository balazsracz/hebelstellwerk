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

#include <LocoNet.h>
#include <utility>

#include "utils/Gpio.h"
#include "utils/Executor.h"
#include "utils/Logging.h"
#include "utils/Timer.h"

enum LnGpioType : uint8_t {
  LNGPIO_NONE,
  /// Listens to or generates switch requests on closed/thrown. Value is true
  /// (high) sends Closed / GREEN, value of False (low) sends Thrown /
  /// RED. Sends or detects only Output ON messages.
  LNGPIO_SWITCH,
  /// Sends on/off messages for Switch RED button (thrown).
  LNGPIO_SWITCH_RED,
  /// Sends on/off messages for Switch GREEN button (closed).
  LNGPIO_SWITCH_GREEN,
  LNGPIO_SENSOR,

  /// This event is set to high when a sensor low message comes. Setting it to
  /// low does nothing on the bus. Setting it to high sends the given message
  /// to the bus.
  LNGPIO_SENSOR_LOW_EVENT,
  /// This event is set to high when a sensor high message comes. Setting it to
  /// low does nothing on the bus. Setting it to high sends the given message
  /// to the bus.
  LNGPIO_SENSOR_HIGH_EVENT,

  /// Raw button from an uhlenbrock stellwerk. Address = XXXXN, where XXXX is
  /// the module address.
  LNGPIO_UB_BUTTON_1,
  LNGPIO_UB_BUTTON_2,
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
    GpioRegistry::instance()->register_obj(this, pin_, Count{count});
    unsigned slen = (2 * count + 31) / 32;
    state_ = new uint32_t[slen];
    memset(state_, 0, slen * 4);
    Executor::instance()->add(this);
  }

  ~LnGpio() { delete[] state_; }

  void begin() override {clear_tm_.start_oneshot(10);}
  void loop() override;
  
  void write(gpio_pin_t pin, bool value) const override {
    uint16_t ofs = (pin - pin_);
    auto pos = get_pos(ofs);
    if (set_bit(pos.first, pos.second, value)) {
      LOG(LEVEL_INFO, "Ln write %d to %d", ofs, value);
      // changed: set dirty
      set_bit(pos.first, pos.second << 1, true);
      any_dirty_ = true;
    }
  }
  bool read(gpio_pin_t pin) const override {
    uint16_t ofs = (pin - pin_);
    auto pos = get_pos(ofs);
    return *(pos.first) & pos.second;
  }
  void set_output(gpio_pin_t pin) const override {
    // noop really. Loconet stuff is always input-output.
  }
  void set_input(gpio_pin_t pin) const override {
    // noop really. Loconet stuff is always input-output.
  }

 private:
  void set_state(unsigned ofs, bool value) {}

  std::pair<uint32_t*, uint32_t> get_pos(unsigned ofs) const {
    ofs *= 2; // always two bits per ofs, one value, one dirty
    uint32_t* p = state_ + (ofs / 32);
    uint32_t bit = 1u << (ofs & 31);
    return {p, bit};
  }

  inline static bool set_bit(uint32_t* pos, uint32_t bit, bool value) {
    bool old_value = (*pos) & bit;
    if (value == old_value) return false;
    if (value) {
      *pos |= bit;
    } else {
      *pos &= ~bit;
    }
    return true;
  }

  inline static bool get_bit(uint32_t* pos, uint32_t bit) {
    return *pos & bit;
  }

  /// Sends out an update to the loconet bus.
  /// @param def is the definition of what to send out
  /// @param value is the new state of the bit.
  /// @return true if the message was successfully sent, false if there was a
  /// collision etc.
  bool send_ln_update(const LnGpioDefn* def, bool value);

  /// First GPIO pin number.
  gpio_pin_t pin_;
  /// How many pins we are simulating.
  uint16_t count_;

  /// True if any dirty bit was set.
  mutable bool any_dirty_{false};

  /// Pointer to the loconet instance.
  LocoNetClass* ln_;
  /// Array with gpio definitions.
  const LnGpioDefn* defs_;

  /// Array allocated on the heap with two bits for each gpio pin we have. The
  /// first bit is the state. The second bit is 1 when we need to send out an
  /// update for this output.
  uint32_t* state_;

  /// Timer used to clear event sensors.
  Timer clear_tm_;
};

#endif  // _UTILS_LNGPIO_H_
