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
 * \file ArduinoStm32SpiPixel.h
 *
 * Class for driving a strip of neopixels using an SPI peripheral.
 *
 * @author Balazs Racz
 * @date 11 May 2024
 */

#ifndef _UTILS_ARDUINOSTM32SPIPIXEL_H_
#define _UTILS_ARDUINOSTM32SPIPIXEL_H_

#include <SPI.h>

#include "stm32yyxx_ll_spi.h"

#ifndef ARDUINO
#error "Won't work"
#endif

#include "utils/Executor.h"
#include "utils/Macros.h"
#include "utils/Pixel.h"

class SpiPixelStrip : public Pixel, public Executable {
 public:
  SpiPixelStrip(int num_pixels, int mosi, int miso, int sclk)
      : Pixel(new uint8_t[num_pixels * 3], num_pixels),
        pin_(mosi),
        pin_name_(digitalPinToPinName(mosi)),
        spi_(mosi, miso, sclk) {
    Executor::instance()->add(this);
  }

  ~SpiPixelStrip() { delete[] data_; }

#if defined(STM32F0xx)
  static constexpr uint32_t SPI_FREQ = 48000000 / 16;
#elif defined(STM32F3xx)
  static constexpr uint32_t SPI_FREQ = 72000000 / 32;
#else
  #error set the SPI clock frequency needed
#endif  
  
  static constexpr auto MODE = SPI_MODE1;

  void begin() override {
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW);
    spi_.begin();
    spi_.beginTransaction(SPISettings{SPI_FREQ, LSBFIRST, MODE});
    ASSERT(spi_.getHandle()->Instance != nullptr);
  }

  void loop() override {
    if (!valid_) {
      flush();
    }
  }

  void __attribute__((optimize("O3"))) flush() override {
    // Set pin to SPI output.
    pinmap_pinout(pin_name_, PinMap_SPI_MOSI);

    spi_.beginTransaction(SPISettings{SPI_FREQ, LSBFIRST, MODE});
    auto* inst = spi_.getHandle()->Instance;
    clear_iteration();
    delayMicroseconds(200);
    noInterrupts();
    // LL_SPI_Enable(inst);
    while (!eof()) {
      uint16_t next_word = 0;
      uint16_t ofs = 1u;
      // Computes 16 SPI bits with 5 pulses.
      while (!eof() && ofs < (1u << 13)) {
        // Appends an 110 or 100 depending opn what the next bit should be.
        next_word |= ofs;
        ofs <<= 1;
        if (next_bit()) next_word |= ofs;
        ofs <<= 2;
      }
      while (!LL_SPI_IsActiveFlag_TXE(inst))
        ;
      // We leave an extra bit as zero at the end of the word. We hope that
      // this will not confuse the pixel. They care mostly about the length of
      // the HIGH pulse.
      LL_SPI_TransmitData16(inst, next_word);
      // Note that there is no wait here for the transfer to complete. This is
      // super important, because we want to be computing the next word while
      // the previous word is emitted by SPI.
    }
    valid_ = true;
    interrupts();
    // Wait for transfer to complete.
    while (LL_SPI_IsActiveFlag_BSY(inst))
      ;

    // Reset pin to GPIO and drive low.
    digitalWrite(pin_name_, LOW);
    pinMode(pin_name_, OUTPUT);
    digitalWrite(pin_name_, LOW);
  }

 private:
  /// Starts a new iteration over the strip. Call next_bit() repeatedly to get
  /// the bits in transmission order.
  void clear_iteration() {
    current_byte_ = 0;
    next_bit_ = 0x80;
  }
  /// @return true if the next bit in the iteration should be 1.
  bool next_bit() {
    bool ret = data_[current_byte_] & next_bit_;
    next_bit_ >>= 1;
    if (!next_bit_) {
      next_bit_ = 0x80;
      current_byte_++;
    }
    return ret;
  }
  /// @return true when the iteration is at eof of the string.
  bool eof() { return current_byte_ >= num_pixels_ * 3; }

  int pin_;
  PinName pin_name_;
  SPIClass spi_;
  /// Controls iteration over the data sequence when producing the output. This
  /// is the index if the current byte.
  unsigned current_byte_;
  /// Controls iteration over the data sequence when producing the output. This
  /// is the mask of the next bit to use.
  uint8_t next_bit_;
};

#endif  // _UTILS_ARDUINOSTM32SPIPIXEL_H_
