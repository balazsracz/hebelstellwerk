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
 * \file GlobalUnlocker.h
 *
 * Implements a GPIO button that triggersthe global unlock.
 *
 * @author Balazs Racz
 * @date 12 May 2024
 */

#ifndef _STW_GLOBALUNLOCKER_H_
#define _STW_GLOBALUNLOCKER_H_

#include "stw/GlobalCommand.h"
#include "utils/Executor.h"

class GlobalUnlocker : public Executable {
 public:
  GlobalUnlocker(gpio_pin_t pin, bool inverted)
      : global_unlock(pin, inverted, GPIO_INPUT) {
    Executor::instance()->add(this);
    tm_.start_periodic(10);
  }

  void begin() override {}
  void loop() override {
    if (!tm_.check()) return;
    if (global_unlock.read() != GlobalState::instance()->is_unlocked_) {
      GlobalState::instance()->is_unlocked_ = global_unlock.read();
      LOG(LEVEL_INFO, "Global unlock %s",
          GlobalState::instance()->is_unlocked_ ? "true" : "false");
    }
  }
  
 private:
  GpioAccessor global_unlock;
  Timer tm_;
};

#endif // _STW_GLOBALUNLOCKER_H_
