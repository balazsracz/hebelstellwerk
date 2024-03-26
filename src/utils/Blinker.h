/** \copyright
 * Copyright (c) 2024, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
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
 * \file Blinker.h
 *
 * Helper class that blinks a Gpio. Used for debugging mostly.
 *
 * @author Balazs Racz
 * @date 26 March 2024
 */

#ifndef _UTILS_BLINKER_H_
#define _UTILS_BLINKER_H_

#include "utils/Gpio.h"
#include "utils/Executor.h"
#include "utils/Timer.h"

class Blinker : public Executable {
 public:
  Blinker(gpio_pin_t gpio) : output_(gpio) { ex.add(this); }

  void begin() override {}

  void loop() override {
    if (tm_.check()) {
      gpio_->write(output_, state_);
      state_ = !state_;
    }
  }

 private:
  gpio_pin_t output_;
  const Gpio* gpio_{GpioRegistry::instance()->get(output_)};
  bool state_{true};
  Timer tm_{Timer::Periodic{600}};
};

#endif // _UTILS_BLINKER_H_
