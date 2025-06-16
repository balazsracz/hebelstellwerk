/** \copyright
 * Copyright (c) 2025, Balazs Racz
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
 * \file ArduinoStm32DmaPwm.h
 *
 * Drives output pins using DMA-based PWM, one at a time. Intended for servos.
 *
 * @author Balazs Racz
 * @date 12 Jun 2025
 */

#ifndef _UTILS_ARDUINOSTM32DMAPWM_H_
#define _UTILS_ARDUINOSTM32DMAPWM_H_

#ifndef ARDUINO
#error "Won't work"
#endif

#include "utils/Pwm.h"

class DmaPwmImpl;

class DmaPwm {
 public:
  DmaPwm(pwm_pin_t pwm_pin_start, std::initializer_list<int> pins) {
    create_impl(pin_start, pins);
  }

 private:
  // We use the pImpl pattern to hide the details and dependencies on low level
  // DMA and Timer drivers.
  DmaPwmImpl* impl_;
  void create_impl(pwm_pin_t pin_start, std::initializer_list<int> pins);
};

#endif // _UTILS_ARDUINOSTM32DMAPWM_H_
