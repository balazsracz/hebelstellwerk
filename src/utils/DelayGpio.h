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
 * \file DelayGpio.h
 *
 * Gpio Wrapper that activates after a certain delay. Can be used for
 * debouncing, but is intended for the Kurbel of the Stellwerk.
 *
 * @author Balazs Racz
 * @date 6 Jul 2025
 */

#ifndef _UTILS_DELAYGPIO_H_
#define _UTILS_DELAYGPIO_H_

#include "utils/Gpio.h"
#include "utils/Timer.h"
#include "utils/Executor.h"
#include "utils/Logging.h"

/// DelayGpio is an input-only virtual Gpio object. It reads an input
/// periodically, and returns TRUE if the input was TRUE for a given amount of
/// time.
class DelayGpio : public Gpio, private Executable {
 public:
  /// Constructor with variable number of arguments.
  /// @param self is the gpio number to register this object for.
  /// @param pin, inverted: one input to put into the OR.
  /// @param length_msec. How many msec long the input has to be high before
  /// the output is set to high.
  DelayGpio(gpio_pin_t self, gpio_pin_t input, bool inverted,
            int16_t length_msec)
      : input_(input, inverted, GPIO_INPUT),  //
        length_msec_(length_msec) {
    Executor::instance()->add(this);
    GpioRegistry::instance()->register_obj(this, self);
  }

  void begin() override {
    tm_.start_periodic(POLL_MSEC);
  }
  void loop() override {
    while (tm_.check()) {
      bool active = input_.read();
      if (active) {
        if (msec_above_ < length_msec_ &&
            msec_above_ + POLL_MSEC >= length_msec_) {
          LOG(LEVEL_INFO, "Kurbel on");
        }
        msec_above_ += POLL_MSEC;
      } else {
        if (msec_above_ > 0) {
          LOG(LEVEL_INFO, "Kurbel off");
        }
        msec_above_ = 0;
      }
    }
  }
  
  bool read(gpio_pin_t pin) const override {
    return msec_above_ >= length_msec_;
  }

  void write(gpio_pin_t pin, bool value) const override {
    // Does nothing.
  }

  void set_input(gpio_pin_t pin) const override {
  }
  void set_output(gpio_pin_t pin) const override {
    DIE("DelayGpio does not support output.");
  }

 private:
  /// How frequently we should check the iput.
  static constexpr uint16_t POLL_MSEC = 7;

  /// Helper for the polling.
  Timer tm_;

  /// Raw input pin.
  GpioAccessor input_;
  /// How many msec should input be high to report high output.
  const int16_t length_msec_;
  /// How many msec did we measure to be high input.
  uint16_t msec_above_{0};
};

#endif  // _UTILS_DELAYGPIO_H_
