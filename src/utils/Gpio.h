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
 * \file Gpio.h
 *
 * Abstract class for dealing with digital I/O pins.
 *
 * @author Balazs Racz
 * @date 8 Mar 2024
 */

#ifndef _UTILS_GPIO_H_
#define _UTILS_GPIO_H_

#include "utils/Macros.h"
#include "utils/Registry.h"
#include "utils/Singleton.h"
#include "utils/Types.h"

/// A nonexistent GPIO pin. Will be translated to a dummy GPIO object by the
/// GPIO registry.
static constexpr gpio_pin_t NO_PIN = -1;

class Gpio {
 public:
  /// Sets a GPIO output to a given value.
  /// @param pin the GPIO pin number. This is a global number, not specific to
  /// the GPIO object.
  /// @param value true for setting the output to high, false for setting the
  /// output to low.
  virtual void write(gpio_pin_t pin, bool value) const = 0;

  /// Return the current value of a GPIO object.
  /// @param pin the GPIO pin number. This is a global number, not specific to
  /// the GPIO object.
  /// @return true for high, false for low.
  virtual bool read(gpio_pin_t pin) const = 0;

  /// Sets the GPIO pin to output. Sufficient to call this once.
  virtual void set_output(gpio_pin_t pin) const {
    DIE("GPIO output not supported");
  }

  /// Sets the GPIO pin to input. Sufficient to call this once.
  virtual void set_input(gpio_pin_t pin) const {
    DIE("GPIO input not supported");
  }
};  // Class Gpio

class DummyGpio : public Gpio {
  void write(gpio_pin_t pin, bool value) const override {}
  bool read(gpio_pin_t pin) const override { return false; }
  void set_output(gpio_pin_t pin) const override {}
  void set_input(gpio_pin_t pin) const override {}
};

class GpioRegistry : public AbstractRegistry<gpio_pin_t, const Gpio, DummyGpio>,
                     public Singleton<GpioRegistry> {};

enum GpioDirection { GPIO_INPUT, GPIO_OUTPUT };

/// Helper class to keep a gpio object, query it and support inversion.
class GpioAccessor {
 public:
  GpioAccessor(gpio_pin_t pin, bool inverted, GpioDirection dir)
      : pin_num_(pin),
        inverted_(inverted),
        gpio_(GpioRegistry::instance()->get(pin)) {
    switch (dir) {
      case GPIO_INPUT:
        gpio_->set_input(pin_num_);
        break;
      case GPIO_OUTPUT:
        gpio_->set_output(pin_num_);
        break;
    }
  }

  bool read() const {
    bool value = gpio_->read(pin_num_);
    if (inverted_) {
      return !value;
    } else {
      return value;
    }
  }

  void write(bool value) const {
    if (inverted_) {
      value = !value;
    }
    gpio_->write(pin_num_, value);
  }

 protected:
  /// Empty constructor to be used only by descendants.
  GpioAccessor() : pin_num_(0), inverted_(false), gpio_(nullptr) {}

  const gpio_pin_t pin_num_;
  const bool inverted_;
  const Gpio* gpio_;
};

/// Alternate to the GpioAccessor that allows getting the
class DelayedGpioAccessor : public GpioAccessor {
 public:
  DelayedGpioAccessor() {}

  void setup(gpio_pin_t pin, bool inverted, GpioDirection dir) {
    // We call a placement constructor of the base class to set up the const
    // arguments that are local to this class.
    new (this) GpioAccessor(pin, inverted, dir);
  }
};

#endif  // _UTILS_GPIO_H_
