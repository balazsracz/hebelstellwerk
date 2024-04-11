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
 * \file UWB.ino
 *
 * Unterwaldmichelbach Bahnhof
 *
 * @author Balazs Racz
 * @date 11 April 2024
 */

#include <Arduino.h>
#include <Hebelstellwerk.h>

#include "utils/Blinker.h"

enum GpioPin : gpio_pin_t {
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

static constexpr int LOCKSERVO_FH_MIN = 10;
static constexpr int LOCKSERVO_FH_MAX = 60;

static constexpr int LCK_TIME_MSEC = 300;


PWM9685 pwm_chip(PWM_CHIP_LOCKSERVO, 0x41);
ServoGpio gpio_verr_fh_d1c1(GPIO_VERRIEGELUNG_FH_D1C1, PWM_VERRIEGELUNG_FH_D1C1,
                            LOCKSERVO_FH_MIN, LOCKSERVO_FH_MAX, LCK_TIME_MSEC);
ServoGpio gpio_verr_fh_d3c3(GPIO_VERRIEGELUNG_FH_D3C3, PWM_VERRIEGELUNG_FH_D3C3,
                          LOCKSERVO_FH_MIN, LOCKSERVO_FH_MAX, LCK_TIME_MSEC);
ServoGpio gpio_verr_fh_a3b3(GPIO_VERRIEGELUNG_FH_A3B3, PWM_VERRIEGELUNG_FH_A3B3,
                          LOCKSERVO_FH_MIN, LOCKSERVO_FH_MAX, LCK_TIME_MSEC);
ServoGpio gpio_verr_fh_a1b1(GPIO_VERRIEGELUNG_FH_A1B1, PWM_VERRIEGELUNG_FH_A1B1,
                          LOCKSERVO_FH_MIN, LOCKSERVO_FH_MAX, LCK_TIME_MSEC);

static constexpr int LOCKSERVO_SIG_MIN = 10;
static constexpr int LOCKSERVO_SIG_MAX = 60;

ServoGpio gpio_verr_d(GPIO_VERRIEGELUNG_D, PWM_VERRIEGELUNG_D,  //
                    LOCKSERVO_SIG_MIN, LOCKSERVO_SIG_MAX, LCK_TIME_MSEC);
ServoGpio gpio_verr_c(GPIO_VERRIEGELUNG_C, PWM_VERRIEGELUNG_C,  //
                    LOCKSERVO_SIG_MIN, LOCKSERVO_SIG_MAX, LCK_TIME_MSEC);
ServoGpio gpio_verr_b(GPIO_VERRIEGELUNG_B, PWM_VERRIEGELUNG_B,  //
                    LOCKSERVO_SIG_MIN, LOCKSERVO_SIG_MAX, LCK_TIME_MSEC);
ServoGpio gpio_verr_a(GPIO_VERRIEGELUNG_A, PWM_VERRIEGELUNG_A,  //
                    LOCKSERVO_SIG_MIN, LOCKSERVO_SIG_MAX, LCK_TIME_MSEC);

static constexpr int LOCKSERVO_W_MIN = 10;
static constexpr int LOCKSERVO_W_MAX = 60;

ServoGpio gpio_verr_10(GPIO_VERRIEGELUNG_10, PWM_VERRIEGELUNG_10,  //
                     LOCKSERVO_W_MIN, LOCKSERVO_W_MAX, LCK_TIME_MSEC);
ServoGpio gpio_verr_9(GPIO_VERRIEGELUNG_9, PWM_VERRIEGELUNG_9,  //
                    LOCKSERVO_W_MIN, LOCKSERVO_W_MAX, LCK_TIME_MSEC);
ServoGpio gpio_verr_7(GPIO_VERRIEGELUNG_7, PWM_VERRIEGELUNG_7,  //
                    LOCKSERVO_W_MIN, LOCKSERVO_W_MAX, LCK_TIME_MSEC);
ServoGpio gpio_verr_2(GPIO_VERRIEGELUNG_2, PWM_VERRIEGELUNG_2,  //
                    LOCKSERVO_W_MIN, LOCKSERVO_W_MAX, LCK_TIME_MSEC);
ServoGpio gpio_verr_1a(GPIO_VERRIEGELUNG_1A, PWM_VERRIEGELUNG_1A,  //
                     LOCKSERVO_W_MIN, LOCKSERVO_W_MAX, LCK_TIME_MSEC);
ServoGpio gpio_verr_1b(GPIO_VERRIEGELUNG_1B, PWM_VERRIEGELUNG_1B,  //
                     LOCKSERVO_W_MIN, LOCKSERVO_W_MAX, LCK_TIME_MSEC);




PwmGpio gpio_1(101, 18 /*pwm channel 2*/, 10, 60);
ServoGpio gpio_2(102, 20 /*pwm channel 4*/, 10, 30, 400);

Gpio23017 ext_chip(128);
GpioAccessor acco(128, false, GPIO_OUTPUT);
GpioAccessor acci(128 + 8, false, GPIO_INPUT);

Blinker blinker{101};
Blinker blinker2{LED_BUILTIN};
Blinker blinker3{102};
// Blinker blinker4{128};

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
  acco.write(acci.read());
}
