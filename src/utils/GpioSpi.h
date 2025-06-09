/** \copyright
 * Copyright (c) 2025, Balazs Racz
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
 * \file GpioSpi.h
 *
 * Arduino driver wrapper for our Gpio API for SPI connected shift registers
 * (166 and 595).
 *
 * @author Balazs Racz
 * @date 26 March 2024
 */

#ifndef _UTILS_GPIOSPI_H_
#define _UTILS_GPIOSPI_H_

#include "utils/Executor.h"
#include "utils/Gpio.h"
#include "utils/Timer.h"
#include "utils/Macros.h"

#include <SPI.h>

/// NUM_IN is the number of input bytes, NUM_OUT is the number of output
/// bytes. Each byte is 8 gpio numbers. The first NUM_OUT * 8 numbers will be
/// the outputs, the next NUM_IN * 8 numbers will be the inputs.
template<int NUM_IN, int NUM_OUT>
class GpioSpi : public Gpio, public Executable {
 public:
  static constexpr uint16_t NUM_PINS = NUM_IN * 8 + NUM_OUT * 8;
  
  /// Time period of polling the GPIO inputs.
  static constexpr uint16_t POLL_MSEC = 2;

  /// Debouncing parameter. How many times in a row we have to read the same
  /// value as input to persist it.
  static constexpr uint8_t QUIESCE_COUNT = 4;


  static_assert(NUM_OUT > 0, "Must have some outputs.");
  
  GpioSpi(gpio_pin_t start_pin_num, int latch_pin, decltype(SPI)& spi = SPI)
      : spi_(spi), start_pin_(start_pin_num), latch_pin_(latch_pin)  {
    Executor::instance()->add(this);
    GpioRegistry::instance()->register_obj(this, start_pin_num,
                                           Count{NUM_PINS});
  }

  void loop() override {
    if (tm_.check()) {
      update();
    }
  }

  void begin() override {
    tm_.start_drifting(POLL_MSEC);
    digitalWrite(latch_pin_, LOW);
    pinMode(latch_pin_, OUTPUT);
    digitalWrite(latch_pin_, LOW);
    update();
  }

  void update() {
    spi_.beginTransaction(kSpiSettings);
    digitalWrite(latch_pin_, LOW);
    for (unsigned i = NUM_OUT; i > 0;) {
      --i;
      spi_.transfer(output_bytes_[i]);
    }
    digitalWrite(latch_pin_, HIGH);
    for (unsigned i = 0; i < NUM_IN; ++i) {
      temp_input_bytes_[i] = spi_.transfer(0);
    }
    spi_.endTransaction();
    for (unsigned i = 0; i < NUM_IN * 8; ++i) {
      unsigned idx = i / 8;
      uint8_t mask = get_mask(i);
      if ((temp_input_bytes_[idx] & mask) == (input_bytes_[idx] & mask)) {
        // input unchanged.
        input_debounce_[i] = QUIESCE_COUNT;
      } else if (!input_debounce_[i]) {
        // countdown for quiesce is done. Take over temp input.
        input_bytes_[idx] =
            (input_bytes_[idx] & ~mask) | (temp_input_bytes_[idx] & mask);
        input_debounce_[i] = QUIESCE_COUNT;
      } else {
        --input_debounce_[i];
      }
    }
  }

  uint8_t get_input_byte(unsigned i) {
    ASSERT(i < NUM_IN);
    return temp_input_bytes_[i];
  }
  
  void write(gpio_pin_t pin, bool value) const override {
    auto p = pin - start_pin_;
    unsigned idx = p / 8;
    unsigned mask = get_mask(p);
    ASSERT(idx < NUM_OUT);
    uint16_t v = value ? mask : 0;
    output_bytes_[idx] = (output_bytes_[idx] & ~mask) | v;
  }
  bool read(gpio_pin_t pin) const override {
    auto p = pin - start_pin_;
    unsigned idx = p / 8;
    unsigned mask = get_mask(p);
    ASSERT(idx >= NUM_OUT);
    idx -= NUM_OUT;
    ASSERT(idx < NUM_IN);
    return !!(input_bytes_[idx] & mask);
  }
  void set_output(gpio_pin_t pin) const override {
    auto p = pin - start_pin_;
    unsigned idx = p / 8;
    ASSERT(idx < NUM_OUT);
    // nothing to do, outputs and inputs are fixed.
  }
  void set_input(gpio_pin_t pin) const override {
    auto p = pin - start_pin_;
    unsigned idx = p / 8;
    ASSERT(idx >= NUM_OUT);
    idx -= NUM_OUT;
    ASSERT(idx < NUM_IN);
    // nothing to do, outputs and inputs are fixed.
  }
  
 private:
  static constexpr SPISettings kSpiSettings{1000000, MSBFIRST, SPI_MODE0};

  static uint8_t get_mask(unsigned p) {
    /// @todo: this determines with MSBFIRST whether we have the right bit
    /// first or not.
    return 1u << (p & 7);
  }
  
  /// Instance of the SPI port to use.
  decltype(SPI)& spi_;

  /// Timer for polling the inputs.
  Timer tm_;

  /// Which (arduino) pin has the latch output.
  int latch_pin_;
  
  /// Registered first pin.
  gpio_pin_t start_pin_;
  
  /// Last set state of the output pins.
  mutable uint8_t output_bytes_[NUM_OUT] = {0};

  /// Debounced state of the input pins.
  uint8_t input_bytes_[NUM_IN] = {0};

  /// Last polled state of the input pins.
  uint8_t temp_input_bytes_[NUM_IN] = {0};

  /// Debounce variables for polling inputs.
  uint8_t input_debounce_[NUM_IN * 8] = {0};  

}; // class GpioSpi




#endif // _UTILS_GPIOSPI_H_

