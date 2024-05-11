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
 * \file Pixel.h
 *
 * Base class for neopixel drivers.
 *
 * @author Balazs Racz
 * @date 11 May 2024
 */

#ifndef _UTILS_PIXEL_H_
#define _UTILS_PIXEL_H_


#include <stdint.h>

class Pixel {
 protected:
  Pixel(uint8_t* data, uint16_t count) : num_pixels_(count), data_(data) {
    memset(data_, 0, num_pixels_ * 3);
  }
 public:
  /// @param px the index of the pixel to change.
  /// @param dim 0 to 2, which channel of the pixel to change
  /// @param value new value for this channel.
  void set(int px, int dim, uint8_t value) {
    ASSERT(px < num_pixels_);
    ASSERT(dim < 3);
    uint8_t& p = data_[px * 3 + dim];
    int16_t vv = value;
    vv *= brightness_;
    value = vv >> 8;
    if (p != value) {
      p = value;
      invalidate();
    }
  }

  /// Sets all three channels of a pixel.
  /// @param px the pixel offset, 0 to count - 1.
  /// @param color 24-bit color value. LSB is blue, second byte is green, third
  /// byte is red.
  void set(int px, uint32_t color) {
    set(px, 0, (color & GREEN) >> 8);
    set(px, 1, (color & RED) >> 16);
    set(px, 2, (color & BLUE) >> 0);
  }

  /// Sets the overall brightness of the strip. All values will be multiplied
  /// with this.
  void set_brightness(uint8_t brightness)  {
    brightness_ = brightness;
  }

  void invalidate() { valid_ = false; }

  virtual void flush() = 0;
  
  static constexpr uint32_t RED = 0xff0000;
  static constexpr uint32_t GREEN = 0x00ff00;
  static constexpr uint32_t BLUE = 0x0000ff;
  static constexpr uint32_t WHITE = 0xffffff;
  static constexpr uint32_t BLACK = 0x000000;
 protected:
  uint16_t num_pixels_;
  uint8_t brightness_ = 255;
  bool valid_{false};
  uint8_t* data_;
};


#endif //  _UTILS_PIXEL_H_
