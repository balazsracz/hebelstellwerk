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
 * \file MockPin.h
 *
 * Helper classes for writing unit tests. 
 *
 * @author Balazs Racz
 * @date 18 Mar 2024
 */

#ifndef _UTILS_MOCKPIN_H_
#define _UTILS_MOCKPIN_H_

#include "utils/Gpio.h"
#include "utils/Pwm.h"
#include "gmock/gmock.h"

#ifndef GTEST
#error this class is test-only
#endif

class MockGpio : public Gpio {
 public:
  MOCK_CONST_METHOD2(write, void(gpio_pin_t, bool));
  MOCK_CONST_METHOD1(read, bool(gpio_pin_t));

  MOCK_CONST_METHOD1(set_output, void(gpio_pin_t));
  MOCK_CONST_METHOD1(set_input, void(gpio_pin_t));
};

class MockPwm : public Pwm {
 public:
  MOCK_CONST_METHOD0(tick_per_msec, tick_t());
  MOCK_CONST_METHOD0(tick_per_period, tick_t());

  MOCK_CONST_METHOD2(write, void(pwm_pin_t, tick_t));
};

#endif // _UTILS_MOCKPIN_H_
