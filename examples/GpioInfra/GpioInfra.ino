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

#include <Arduino.h>
#include <Hebelstellwerk.h>

#include "utils/AnalogGpio.h"
#include "utils/Blinker.h"

enum GpioPin : gpio_pin_t {
  /// @todo these are wrong. Check the wiring instead.

  ARDUINO_MIN = 99,
  GPIO_VERRIEGELUNG_FH_D1C1,
  GPIO_VERRIEGELUNG_FH_D3C3,
  GPIO_VERRIEGELUNG_FH_A3B3,
  GPIO_VERRIEGELUNG_FH_A1B1,
  GPIO_VERRIEGELUNG_D,
  GPIO_VERRIEGELUNG_C,
  GPIO_VERRIEGELUNG_B,
  GPIO_VERRIEGELUNG_A,
  GPIO_VERRIEGELUNG_10,
  GPIO_VERRIEGELUNG_9,
  GPIO_VERRIEGELUNG_7,
  GPIO_VERRIEGELUNG_2,
  GPIO_VERRIEGELUNG_1A,
  GPIO_VERRIEGELUNG_1B,
  
  GPIO_SERVO_CAL,

};

enum PwmPin : pwm_pin_t {
  PWM_CHIP_LOCKSERVO = 50,
  PWM_VERRIEGELUNG_FH_D1C1 = PWM_CHIP_LOCKSERVO + 0,
  PWM_VERRIEGELUNG_FH_D3C3,
  PWM_VERRIEGELUNG_FH_A3B3,
  PWM_VERRIEGELUNG_FH_A1B1,
  PWM_UNUSED_41_4,
  PWM_UNUSED_41_5,
  PWM_VERRIEGELUNG_D,
  PWM_VERRIEGELUNG_C,
  PWM_VERRIEGELUNG_10,
  PWM_VERRIEGELUNG_9,
  PWM_VERRIEGELUNG_7,
  PWM_VERRIEGELUNG_2,
  PWM_VERRIEGELUNG_1B,
  PWM_VERRIEGELUNG_1A,
  PWM_VERRIEGELUNG_B,
  PWM_VERRIEGELUNG_A,
};

static_assert(PWM_CHIP_LOCKSERVO + 15 == PWM_VERRIEGELUNG_A,
              "misaligned pwm lockservo");

PWM9685 pwm_chip_servo(PWM_CHIP_LOCKSERVO, 0x41);

// PWM9685 pwm_chip(16);
// PwmGpio gpio_1(101, 18 /*pwm channel 2*/, 10, 60);
// ServoGpio gpio_2(102, 20 /*pwm channel 4*/, 10, 30, 400);

// Gpio23017 ext_chip(128);
// GpioAccessor acco(128, false, GPIO_OUTPUT);
// GpioAccessor acci(128+8, false, GPIO_INPUT);

static constexpr int LCK_TIME_MSEC = 1000;

ServoGpio gpio_verr_fh_d1c1(GPIO_VERRIEGELUNG_FH_D1C1, PWM_VERRIEGELUNG_FH_D1C1,
                            usec(2430), usec(2520), 0);
ServoGpio gpio_verr_fh_d3c3(GPIO_VERRIEGELUNG_FH_D3C3, PWM_VERRIEGELUNG_FH_D3C3,
                            usec(2150), usec(2360), LCK_TIME_MSEC);
ServoGpio gpio_verr_fh_a3b3(GPIO_VERRIEGELUNG_FH_A3B3, PWM_VERRIEGELUNG_FH_A3B3,
                            usec(1057), usec(1367), LCK_TIME_MSEC);
ServoGpio gpio_verr_fh_a1b1(GPIO_VERRIEGELUNG_FH_A1B1, PWM_VERRIEGELUNG_FH_A1B1,
                            usec(1583), usec(1967), LCK_TIME_MSEC);

ServoGpio gpio_verr_d(GPIO_VERRIEGELUNG_D, PWM_VERRIEGELUNG_D,  //
                      usec(1280), usec(1650), LCK_TIME_MSEC);
ServoGpio gpio_verr_c(GPIO_VERRIEGELUNG_C, PWM_VERRIEGELUNG_C,  //
                      usec(1073), usec(1413), LCK_TIME_MSEC);
ServoGpio gpio_verr_b(GPIO_VERRIEGELUNG_B, PWM_VERRIEGELUNG_B,  //
                      usec(1570), usec(1900), LCK_TIME_MSEC);
ServoGpio gpio_verr_a(GPIO_VERRIEGELUNG_A, PWM_VERRIEGELUNG_A,  //
                      usec(1210), usec(1580), LCK_TIME_MSEC);

ServoGpio gpio_verr_10(GPIO_VERRIEGELUNG_10, PWM_VERRIEGELUNG_10,  //
                       usec(1553), usec(1850), LCK_TIME_MSEC);
ServoGpio gpio_verr_9(GPIO_VERRIEGELUNG_9, PWM_VERRIEGELUNG_9,  //
                      usec(1600), usec(1983), LCK_TIME_MSEC);
ServoGpio gpio_verr_7(GPIO_VERRIEGELUNG_7, PWM_VERRIEGELUNG_7,  //
                      usec(1153), usec(1530), LCK_TIME_MSEC);
ServoGpio gpio_verr_2(GPIO_VERRIEGELUNG_2, PWM_VERRIEGELUNG_2,  //
                      usec(933), usec(1353), LCK_TIME_MSEC);
ServoGpio gpio_verr_1a(GPIO_VERRIEGELUNG_1A, PWM_VERRIEGELUNG_1A,  //
                       usec(1500), usec(1946), LCK_TIME_MSEC);
ServoGpio gpio_verr_1b(GPIO_VERRIEGELUNG_1B, PWM_VERRIEGELUNG_1B,  //
                       usec(1453), usec(1840), LCK_TIME_MSEC);

ServoGpio gpio_measure(GPIO_SERVO_CAL, PWM_UNUSED_41_4,  //
                       usec(1000), usec(1100), 0);


// Blinker blinker{101};
Blinker blinker2{LED_BUILTIN, 2000};
// Blinker blinker3{102};
// Blinker blinker4{128};

GpioAccessor led(LED_BUILTIN, false, GPIO_OUTPUT);

/// Arduino setup routine.
void setup() {
  Serial.begin(9600);
  delay(50);
  Serial.println("hello world");
  // Calls the executor to do begin for all registered objects.
  ex.begin();
  pwm_chip_servo.set_freq(23500000, 111, 458);
}

/// Arduino loop routine.
void loop() {
  // Calls the executor to do loop for all registered objects.
  ex.loop();
  //  if (false) {
  static bool last = false;
  bool current = led.read();
  if (current != last) {
    last = current;
    if (current) {
      for (auto i :
           {GPIO_VERRIEGELUNG_FH_D1C1, GPIO_VERRIEGELUNG_FH_D3C3,
            GPIO_VERRIEGELUNG_FH_A3B3, GPIO_VERRIEGELUNG_FH_A1B1,
            GPIO_VERRIEGELUNG_D, GPIO_VERRIEGELUNG_C, GPIO_VERRIEGELUNG_B,
            GPIO_VERRIEGELUNG_A, GPIO_VERRIEGELUNG_10, GPIO_VERRIEGELUNG_9,
            GPIO_VERRIEGELUNG_7, GPIO_VERRIEGELUNG_2, GPIO_VERRIEGELUNG_1A,
            GPIO_VERRIEGELUNG_1B, GPIO_SERVO_CAL}) {
        GpioRegistry::instance()->get(i)->write(0, true);
      }
    } else {
      for (auto i :
           {GPIO_VERRIEGELUNG_FH_D1C1, GPIO_VERRIEGELUNG_FH_D3C3,
            GPIO_VERRIEGELUNG_FH_A3B3, GPIO_VERRIEGELUNG_FH_A1B1,
            GPIO_VERRIEGELUNG_D, GPIO_VERRIEGELUNG_C, GPIO_VERRIEGELUNG_B,
            GPIO_VERRIEGELUNG_A, GPIO_VERRIEGELUNG_10, GPIO_VERRIEGELUNG_9,
            GPIO_VERRIEGELUNG_7, GPIO_VERRIEGELUNG_2, GPIO_VERRIEGELUNG_1A,
            GPIO_VERRIEGELUNG_1B, GPIO_SERVO_CAL}) {
        GpioRegistry::instance()->get(i)->write(0, false);
      }
    }
  }
  // acco.write(acci.read());
}
