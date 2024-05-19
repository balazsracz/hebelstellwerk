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
 * \file EisenbachBelegtmelder.h
 *
 * Special virtual GPIO class to work around a super weird block occupancy
 * detector built by Heiko Herholz for Abzweig Eisenbach.
 *
 * @author Balazs Racz
 * @date 16 May 2024
 */

#ifndef _STW_EISENBAHNBELEGTMELDER_H_
#define _STW_EISENBAHNBELEGTMELDER_H_

#include "utils/Executor.h"
#include "utils/Gpio.h"


class EisenbachBelegtmelder : public Gpio, public Executable {
 public:
  EisenbachBelegtmelder(gpio_pin_t pin) {
    GpioRegistry::instance()->register_obj(this, pin);
    Executor::instance()->add(this);
  }

  void begin() override {

  }

  void loop() override {
    for (unsigned i = 0; i < 4; ++i) {
      if (sen_[i].read()) {
        tm_.start_oneshot(DELAY_RESET_MSEC);
        sen_[i].write(false);
        seen_ |= 1u<<i;
      }
    }
    if (tm_.check()) {
      // Time is up, reset all bits to zero.
      seen_ = 0;
    }
  }

  void write(gpio_pin_t pin, bool value) const override {
    // do nothing
  }
  bool read(gpio_pin_t pin) const override {
    // Have we seen all four sensors?
    return seen_ == 0b1111;
  }
  
  void set_input(gpio_pin_t pin) const override {
    // noop really. Loconet stuff is always input-output.
  }
  
  
 private:
  static const unsigned DELAY_RESET_MSEC = 2000;
  GpioAccessor sen_[4] = {{LN_SEN_1020_LOW, false, GPIO_INPUT},
                          {LN_SEN_1030_LOW, false, GPIO_INPUT},
                          {LN_SEN_2010_LOW, false, GPIO_INPUT},
                          {LN_SEN_3010_LOW, false, GPIO_INPUT}};
  /// Bit 0..3 are whether we've seen sensor 0..3 since the last reset.
  uint8_t seen_{0};
  Timer tm_;
};



#endif // _STW_EISENBAHNBELEGTMELDER_H_
