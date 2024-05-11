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
 * \file ArduinoArmPixel.h
 *
 * Class for driving a strip of neopixels.
 *
 * @author Balazs Racz
 * @date 11 May 2024
 */

#ifndef _UTILS_ARDUINOARMPIXEL_H_
#define _UTILS_ARDUINOARMPIXEL_H_

#ifndef ARDUINO
#error "Won't work"
#endif

#include "utils/Executor.h"
#include "utils/Macros.h"
#include "utils/Pixel.h"

class PixelStrip : public Pixel, public Executable {
 public:
  PixelStrip(int num_pixels, int pin) : Pixel(new uint8_t[num_pixels * 3], num_pixels), pin_(pin) {
    Executor::instance()->add(this);
  }

  ~PixelStrip() {
    delete[] data_;
  }
  
  void begin() override {
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW);
    clock_reset_ = nsec_to_clock(100e3);
    clock_1hi_ = nsec_to_clock(800);
    clock_1low_ = nsec_to_clock(450);
    clock_0hi_ = nsec_to_clock(400);
    clock_0low_ = nsec_to_clock(850);
  }

  void loop() override {
    if (!valid_) {
      flush();
    }
  }

  static constexpr uint32_t SETHIGH = (1u<<7);
  static constexpr uint32_t SETLOW = (1u<<7)<<16;
  
  void __attribute__((optimize("O3"))) flush() override {
    noInterrupts();
    digitalWrite(pin_, LOW);
    clear_clock();
    wait_clock(clock_reset_);
    
    for (unsigned idx = 0; idx < num_pixels_ * 3; ++idx) {
      for (uint8_t bit = 0x80; bit != 0; bit >>= 1) {
        if (data_[idx] & bit) {
          GPIOA->BSRR = SETHIGH;
          wait_clock(clock_1hi_);
          GPIOA->BSRR = SETLOW;
          wait_clock(clock_1low_);
        } else {
          GPIOA->BSRR = SETHIGH;
          wait_clock(clock_0hi_);
          GPIOA->BSRR = SETLOW;
          wait_clock(clock_0low_);
        }
      }
    }
    //digitalWrite(pin_, LOW);
    valid_ = true;
    interrupts();
  }

 private:
  /// Systick counter reset value.
  static constexpr uintptr_t STRVR = 0xE000E014;
  /// Systick counter current value.
  static constexpr uintptr_t STCVR = 0xE000E018;

  int32_t get_clock() { return *(volatile int32_t*)STCVR; }
  int32_t get_reload() { return *(volatile int32_t*)STRVR; }
  
  void clear_clock() { last_clock_  = get_clock(); }

  void __attribute__((always_inline, optimize("O3")))
      wait_clock(int delta_clock) {
    int32_t next_clock = last_clock_;
    next_clock -= delta_clock;
    if (next_clock < 0) {
      next_clock += get_reload();
      //ASSERT(next_clock > 0);
      // Wait with rollover.
      while (true) {
        int32_t c = get_clock();
        if (c > last_clock_ && c <= next_clock) break;
      }
    } else {
      // Wait without rollover.
      while (true) {
        int32_t c = get_clock();
        if (c > last_clock_ || c <= next_clock) break;
      }
    }
    last_clock_ = next_clock;
  }

  uint32_t nsec_to_clock(uint32_t nsec) {
    uint64_t f = nsec;
    f *= F_CPU;
    f /= 1e9;
    ASSERT(f < get_reload() / 2);
    return f;
  }

  bool rollover_needed_;
  int pin_;
  uint8_t* data_;
  /// Stores the END clock value from the last wait operation.
  int32_t last_clock_;
  /// How many clocks it takes to send a reset pulse.
  uint32_t clock_reset_;
  /// How many clocks it takes to send the high side of a 1 bit.
  uint32_t clock_1hi_;
  /// How many clocks it takes to send the low side of a 1 bit.
  uint32_t clock_1low_;
  /// How many clocks it takes to send the high side of a 0 bit.
  uint32_t clock_0hi_;
  /// How many clocks it takes to send the low side of a 0 bit.
  uint32_t clock_0low_;
};

#endif  // _UTILS_ARDUINOARMPIXEL_H_
