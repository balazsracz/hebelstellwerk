/** \copyright
 * Copyright (c) 2024, Balazs Racz
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
 * \file GpioInfra.ino
 *
 * Example Arduino sketch for the Hebelstellwerk library showing various GPIO
 * infrastructure including using GPIO extenders, PWM extenders and driving
 * Servos.
 *
 * @author Balazs Racz
 * @date 26 March 2024
 */

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Hebelstellwerk.h>
#include <Wire.h>

// called this way, it uses the default address 0x40
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

PwmRegistry pwm_reg;

class PWM9685 : public Pwm, public Executable {
 public:
  /// One chip has 16 channels.
  static constexpr uint16_t NUM_PINS = 16;

  /// Constructor
  ///
  /// @param start_address pwm_pin to register the first channel for. The next
  /// consecutive 16 pwm_pin will mean channel 0..15 on the outputs.
  /// @param address I2C address, default is 0x40.
  /// @param w Instance of I2C bus from Arduino device.
  PWM9685(pwm_pin_t start_address, int address = 0x40, TwoWire& w = Wire)
      : driver_(address, w), start_pin_(start_address) {
    ex.add(this);
    PwmRegistry::instance()->register_obj(this, start_address, Count{NUM_PINS});
  }

  void begin() override {
    driver_.begin();
    // We want ~1000 count to be 1 msec. 4096 count would then be 4 msec. This
    // turns into 250 Hz frequency.
    driver_.setPWMFreq(250);
  }

  tick_t tick_per_msec() const override { return 1024; }
  tick_t tick_per_period() const override { return 4096; }

  void write(pwm_pin_t pin, tick_t count_high) const override {
    driver_.setPWM(pin - start_pin_, 0, count_high);
  }

  void loop() override {}

 private:
  /// Backing driver that talks through i2c to the chip.
  mutable Adafruit_PWMServoDriver driver_;
  /// Registered first pin.
  pwm_pin_t start_pin_;
};

PWM9685 pwm_chip(16);
PwmGpio gpio_1(101, 18 /*pwm channel 2*/, 10, 60);
ServoGpio gpio_2(102, 20 /*pwm channel 4*/, 10, 30, 400);

class Blinker : public Executable {
 public:
  Blinker(gpio_pin_t gpio) : output_(gpio) { ex.add(this); }

  void begin() override {}

  void loop() override {
    if (tm_.check()) {
      gpio_->write(output_, state_);
      state_ = !state_;
    }
  }

 private:
  gpio_pin_t output_;
  const Gpio* gpio_{GpioRegistry::instance()->get(output_)};
  bool state_{true};
  Timer tm_{Timer::Periodic{600}};
};

Blinker blinker{101};
Blinker blinker2{LED_BUILTIN};
Blinker blinker3{102};

/// Arduino setup routine.
void setup() {
  Serial.begin(9600);
  Serial.println("hello world");
  // Calls the executor to do begin for all registered objects.
  ex.begin();
}

/// Arduino loop routine.
void loop() {
  // Calls the executor to do loop for all registered objects.
  ex.loop();
}
