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
 * \file OrGpio.h
 *
 * Gpio Wrapper that ORs two (or more) separate GPIO inputs.
 *
 * @author Balazs Racz
 * @date 9 May 2024
 */

#ifndef _STW_BLOCKDETECTOROVERRIDE_H_
#define _STW_BLOCKDETECTOROVERRIDE_H_

/// BlockDetectorOverride is an input-only virtual Gpio object. It reads
/// a block detector and a button to override it.
///
/// When the block detector does not have a CT coil connected, then it reports
/// always busy. Therefore this class watches the real detector and if it has
/// never seen it free, then overrides it to free. Then only the button's state
/// is relevant.
class BlockDetectorOverride : public Gpio {
 public:
  /// Constructor
  ///
  /// @param self GPIO pin number to export.
  /// @param btn_gpio the gpio number for the override button
  /// @param btn_inverted true if the override button should be inverted. If
  /// inverted is false, the button HIGH means the result will be HIGH
  /// (occupied).
  /// @param real_gpio the gpio number for the track detector.
  /// @param real_inverted true if the real detector should be inverted. If
  /// inverted is false, the detector reading HIGH means the result will be
  /// HIGH (occupied).
  BlockDetectorOverride(gpio_pin_t self, gpio_pin_t btn_gpio, bool btn_inverted,
                        gpio_pin_t real_gpio, bool real_inverted)
      : real_detector_(real_gpio, real_inverted, GPIO_INPUT),
        btn_detector_(btn_gpio, btn_inverted, GPIO_INPUT) {
    GpioRegistry::instance()->register_obj(this, self);
  }

  bool read(gpio_pin_t pin) const override {
    bool real = real_detector_.read();
    bool btn = btn_detector_.read();
    if (!seen_real_empty_ && !real) {
      seen_real_empty_ = true;
    }
    if (!seen_real_empty_) {
      real = false;
    }
    return real || btn;
  }

  void write(gpio_pin_t pin, bool value) const override {
    // Does nothing.
  }

 private:
  GpioAccessor real_detector_;
  GpioAccessor btn_detector_;
  /// Set to true the first time we see the real detector as empty. The real
  /// detector has a property that when the CT coil is not attached, it reports
  /// "occupied" at all times.
  mutable bool seen_real_empty_{false};
};

#endif  // _STW_BLOCKDETECTOROVERRIDE_H_
