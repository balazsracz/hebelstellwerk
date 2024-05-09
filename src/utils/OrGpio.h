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

#ifndef _UTILS_GPIOOR_H_
#define _UTILS_GPIOOR_H_

/// OrGpio is an input-only virtual Gpio object. It reads multiple input pins,
/// and returns TRUE if any of the inputs are TRUE.
///
/// Note that due to the inversion feature, this can also express AND.
class OrGpio : public Gpio {
 public:
  /// Constructor with variable number of arguments.
  /// @param self is the gpio number to register this object for.
  /// @param pin, inverted: one input to put into the OR.
  /// pin and inverted pairs can be repeated arbitarily.
  template <typename... Args>
  OrGpio(gpio_pin_t self, gpio_pin_t pin, bool inverted, Args... args) {
    add(pin, inverted, args...);
    GpioRegistry::instance()->register_obj(this, self);
  }

  ~OrGpio() {
    while(first_) {
      auto* p = first_;
      first_ = first_->next;
      delete p;
    }
  }
  
  bool read() const override {
    for (const Entry* e = first_; e; e = e->next) {
      if (e->obj->read(e->pin)) {
        return true;
      }
    }
    return false;
  }

  void write(bool value) const override {
    // Does nothing.
  }

  void set_input(gpio_pin_t pin) const override {
    for (const Entry* e = first_; e; e = e->next) {
      e->obj->set_input(e->pin);
    }
  }
  
 private:
  void add_one(gpio_pin_t pin, bool inverted) {
    auto* e = new Entry;
    e->next = first_;
    first_ = e;
    e->pin = pin;
    e->inverted = inverted;
    e->obj = GpioRegistry::instance()->get(pin);
  }

  void add(gpio_pin_t pin, bool inverted, Args... args) {
    add(args...);
    add_one(pin, inverted);
  }

  void add() {}

  /// Keeps track of an entry to be OR'ed.
  struct Entry {
    Entry* next{nullptr};
    Gpio* obj{nullptr};
    gpio_pin_t pin{NO_PIN};
    bool inverted{false};
  };

  Entry* first_{nullptr};
};

#endif  // _UTILS_GPIOOR_H_
