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

  GPIO_ENDFELD_D,
  GPIO_ERLAUBNISFELD_C,
  GPIO_ANFANGSFELD_C,
  GPIO_FSTRFEST_CD,
  GPIO_FSTRFEST_AB,
  GPIO_ANFANGSFELD_B,
  GPIO_ERLAUBNISFELD_B,
  GPIO_ENDFELD_A,
  
  GPIO_EXT_HEBEL_SIG_W,
  GPIO_GSP_IV = GPIO_EXT_HEBEL_SIG_W + 2,
  GPIO_GSP_III,
  GPIO_GSP_II,
  GPIO_GSP_I,
  GPIO_HEBEL_B,
  GPIO_HEBEL_A,
  GPIO_HEBEL_1A,
  GPIO_HEBEL_1B,
  GPIO_HEBEL_2,
  GPIO_HEBEL_7,
  GPIO_HEBEL_9,
  GPIO_HEBEL_10,
  GPIO_HEBEL_C,
  GPIO_HEBEL_D,

  GPIO_EXT_HEBEL_FSTR,
  GPIO_HEBEL_C1 = GPIO_EXT_HEBEL_FSTR + 8,
  GPIO_HEBEL_D1,
  GPIO_HEBEL_C3,
  GPIO_HEBEL_D3,
  GPIO_HEBEL_B3,
  GPIO_HEBEL_A3,
  GPIO_HEBEL_B1,
  GPIO_HEBEL_A1,

  GPIO_EXT_TASTER,
  GPIO_BTN_ENDFELD_D = GPIO_EXT_TASTER + 8,
  GPIO_BTN_ERLAUBNIS_C,
  GPIO_BTN_ANFANG_C,
  GPIO_BTN_FESTLEGE_CD,
  GPIO_BTN_FESTLEGE_AB,
  GPIO_BTN_ANFANG_B,
  GPIO_BTN_ERLAUBNIS_B,
  GPIO_BTN_ENDFELD_B,
  
};

static_assert(GPIO_EXT_HEBEL_SIG_W + 15 == GPIO_HEBEL_D,
              "misaligned ext hebel sig w");

static_assert(GPIO_EXT_HEBEL_FSTR + 15 == GPIO_HEBEL_A1,
              "misaligned ext hebel fstr");

static_assert(GPIO_EXT_TASTER + 15 == GPIO_BTN_ENDFELD_B,
              "misaligned ext taster");

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

  PWM_CHIP_FELDER = 70,
  PWM_END_D_ROT = PWM_CHIP_FELDER,
  PWM_END_D_WEISS,
  PWM_ERLAUBNIS_C_ROT,
  PWM_ERLAUBNIS_C_WEISS,
  PWM_ANFANG_C_ROT,
  PWM_ANFANG_C_WEISS,
  PWM_FESTLEGE_CD_ROT,
  PWM_FESTLEGE_CD_WEISS,
  PWM_FESTLEGE_AB_ROT,
  PWM_FESTLEGE_AB_WEISS,
  PWM_ANFANG_B_ROT,
  PWM_ANFANG_B_WEISS,
  PWM_ERLAUBNIS_B_ROT,
  PWM_ERLAUBNIS_B_WEISS,
  PWM_END_A_ROT,
  PWM_END_A_WEISS,
};

static_assert(PWM_CHIP_LOCKSERVO + 15 == PWM_VERRIEGELUNG_A,
              "misaligned pwm lockservo");

static_assert(PWM_CHIP_FELDER + 15 == PWM_END_A_WEISS,
              "misaligned pwm felder");

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


Gpio23017 ext_hebel_sig_w(GPIO_EXT_HEBEL_SIG_W, 0x25);
Gpio23017 ext_hebel_fstr(GPIO_EXT_HEBEL_FSTR, 0x26);
/// @todo is this I2C address correct?
Gpio23017 ext_hebel_taster(GPIO_EXT_TASTER, 0x27);



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
