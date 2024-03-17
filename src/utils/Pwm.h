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
 * \file Pwm.h
 *
 * Abstract class for output pins that can do PWM output.
 *
 * @author Balazs Racz
 * @date 17 Mar 2024
 */

#ifndef _UTILS_PWM_H_
#define _UTILS_PWM_H_

#include "utils/Types.h"

/// A nonexistent PWM pin. Will be translated to a dummy PWM object by the
/// registry.
static constexpr pwm_pin_t NO_PWM_PIN = -1;

class Pwm {
 public:
  typedef uint16_t tick_t;

  /// @return how many ticks this object has for a one millisecond pulse width.
  virtual tick_t tick_per_msec() const = 0;

  /// @return period length in tick count.
  virtual tick_t tick_per_period() const = 0;

  /// Update the output pulse width.
  /// @param pin the output pin to act upon.
  /// @param count_high how many counts of the period should the output be high.
  virtual void write(pwm_pin_t pin, tick_t count_high) = 0;
};


/// Empty implementation of a PWM pin.
class DummyPwm : public Pwm {

};

#endif // _UTILS_PWM_H_
