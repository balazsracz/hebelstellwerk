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

// ============== Declarations related to the phycial pinout =================

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

  GPIO_ENDFELD_D,
  GPIO_ERLAUBNIS_C,
  GPIO_ANFANGS_C,
  GPIO_FESTLEGE_CD,
  GPIO_FESTLEGE_AB,
  GPIO_ANFANGS_B,
  GPIO_ERLAUBNIS_B,
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

  GPIO_EXT_DETECTOR, 
  GPIO_AB_DETECTOR = GPIO_EXT_DETECTOR + 3,
  GPIO_CD_DETECTOR = GPIO_EXT_DETECTOR + 1,
  GPIO_EXT_DETECTOR_END = GPIO_EXT_DETECTOR + 15, 


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

static_assert(PWM_CHIP_FELDER + 15 == PWM_END_A_WEISS, "misaligned pwm felder");

static constexpr int LOCKSERVO_FH_MIN = 10;
static constexpr int LOCKSERVO_FH_MAX = 60;

static constexpr int LCK_TIME_MSEC = 300;

PWM9685 pwm_chip_servo(PWM_CHIP_LOCKSERVO, 0x41);
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

PWM9685 pwm_chip_felder(PWM_CHIP_FELDER, 0x40);

// These are laid out for true = white false = red.
DualPwmGpio gpio_endfeld_d(GPIO_ENDFELD_D, PWM_END_D_ROT, PWM_END_D_WEISS);
DualPwmGpio gpio_erlaubnis_c(GPIO_ERLAUBNIS_C, PWM_ERLAUBNIS_C_ROT,
                             PWM_ERLAUBNIS_C_WEISS);
DualPwmGpio gpio_anfang_c(GPIO_ANFANGS_C, PWM_ANFANG_C_ROT, PWM_ANFANG_C_WEISS);
DualPwmGpio gpio_festlege_cd(GPIO_FESTLEGE_CD, PWM_FESTLEGE_CD_ROT,
                             PWM_FESTLEGE_CD_WEISS);
DualPwmGpio gpio_festlege_ab(GPIO_FESTLEGE_AB, PWM_FESTLEGE_AB_ROT,
                             PWM_FESTLEGE_AB_WEISS);
DualPwmGpio gpio_anfang_b(GPIO_ANFANGS_B, PWM_ANFANG_B_ROT, PWM_ANFANG_B_WEISS);
DualPwmGpio gpio_erlaubnis_b(GPIO_ERLAUBNIS_B, PWM_ERLAUBNIS_B_ROT,
                             PWM_ERLAUBNIS_B_WEISS);
DualPwmGpio gpio_endfeld_a(GPIO_ENDFELD_A, PWM_END_A_ROT, PWM_END_A_WEISS);

Gpio23017 ext_hebel_sig_w(GPIO_EXT_HEBEL_SIG_W, 0x26);
Gpio23017 ext_hebel_fstr(GPIO_EXT_HEBEL_FSTR, 0x25);
Gpio23017 ext_hebel_taster(GPIO_EXT_TASTER, 0x27);
/// @todo is this I2C address correct?
Gpio23017 ext_detector(GPIO_EXT_DETECTOR, 0x20);


class Report : public Executable {
public:
  Report() {
    Executor::instance()->add(this);
    tm_.start_periodic(1000);
  }

  void begin() override {}
  void loop() override {
    if (!tm_.check()) return;
    ++i;
    //Serial.print("Hello ");
    //Serial.println(i);
    //LOG(LEVEL_INFO, "hello2 1ok %d 2ok %d 3ok %d 4ok %d", ext_hebel_sig_w.ok(), ext_hebel_fstr.ok(), ext_hebel_taster.ok(), ext_detector.ok());
    LOG(LEVEL_INFO, "hebel %04x fstr %04x taster %04x det %04x", ext_hebel_sig_w.input_states(), ext_hebel_fstr.input_states(), ext_hebel_taster.input_states(), ext_detector.input_states());
  }

private:
  Timer tm_;
  int i = 0;
} reporter;

// ======================== Logical devices =========================

enum SignalId : uint8_t { SIGNAL_A, SIGNAL_B, SIGNAL_C, SIGNAL_D };

enum TurnoutId : uint8_t { W1a, W1b, W2, W7, W9, W10 };

enum RouteId : uint8_t { a1, b1, a3, b3, c3, d3, c1, d1 };

enum AuxId : uint8_t {
  Bue10_750,
  Gspr1,
  Gspr2,
};

enum BlockId : uint8_t {
  BLOCK_AB,
  BLOCK_CD,
};

static constexpr bool WHEB_INV = false;
static constexpr bool WLCK_INV = false;

TurnoutLever TW1a{W1a, GPIO_HEBEL_1A, WHEB_INV, GPIO_VERRIEGELUNG_1A, WLCK_INV};
TurnoutLever TW1b{W1b, GPIO_HEBEL_1B, WHEB_INV, GPIO_VERRIEGELUNG_1B, WLCK_INV};
TurnoutLever TW2{W2, GPIO_HEBEL_2, WHEB_INV, GPIO_VERRIEGELUNG_2, WLCK_INV};
TurnoutLever TW7{W7, GPIO_HEBEL_7, WHEB_INV, GPIO_VERRIEGELUNG_7, WLCK_INV};
TurnoutLever TW9{W9, GPIO_HEBEL_9, WHEB_INV, GPIO_VERRIEGELUNG_9, WLCK_INV};
TurnoutLever TW10{W10, GPIO_HEBEL_10, WHEB_INV, GPIO_VERRIEGELUNG_10, WLCK_INV};

static constexpr bool SHEB_INV = false;
static constexpr bool SLCK_INV = false;

SignalLever SA{SIGNAL_A, HP2, GPIO_HEBEL_A, SHEB_INV, GPIO_VERRIEGELUNG_A,
               SLCK_INV};
SignalLever SB{SIGNAL_B, HP2, GPIO_HEBEL_B, SHEB_INV, GPIO_VERRIEGELUNG_B,
               SLCK_INV};
SignalLever SC{SIGNAL_C, HP2, GPIO_HEBEL_C, SHEB_INV, GPIO_VERRIEGELUNG_C,
               SLCK_INV};
SignalLever SD{SIGNAL_D, HP2, GPIO_HEBEL_D, SHEB_INV, GPIO_VERRIEGELUNG_D,
               SLCK_INV};

static constexpr bool RHEB_INV = false;
static constexpr bool RLCK_INV = false;

RouteLever Rab1(a1, b1, GPIO_HEBEL_A1, RHEB_INV, GPIO_HEBEL_B1, RHEB_INV,
                GPIO_VERRIEGELUNG_FH_A1B1, RLCK_INV);
RouteLever Rab3(a3, b3, GPIO_HEBEL_A3, RHEB_INV, GPIO_HEBEL_B3, RHEB_INV,
                GPIO_VERRIEGELUNG_FH_A3B3, RLCK_INV);
RouteLever Rcd3(c3, d3, GPIO_HEBEL_C3, RHEB_INV, GPIO_HEBEL_D3, RHEB_INV,
                GPIO_VERRIEGELUNG_FH_D3C3, RLCK_INV);
RouteLever Rcd1(c1, d1, GPIO_HEBEL_C1, RHEB_INV, GPIO_HEBEL_D1, RHEB_INV,
                GPIO_VERRIEGELUNG_FH_D1C1, RLCK_INV);

/// @todo check the actual I2C addresses.
I2CBlock i2c_ab(0x52);
I2CBlock i2c_cd(0x53);

FelderBlock BlkAB(&i2c_ab, BLOCK_AB, GPIO_AB_DETECTOR, false,
                  GPIO_BTN_FESTLEGE_AB, false, GPIO_FESTLEGE_AB, false);
FelderBlock BlkCD(&i2c_cd, BLOCK_CD, GPIO_CD_DETECTOR, false,
                  GPIO_BTN_FESTLEGE_CD, false, GPIO_FESTLEGE_CD, false);

// ================ Verschlusstabelle ===============

LockTable ltbl({
    Route(a1), WeichePlus(W1a), WeichePlus(W1b), WeichePlus(W2), WeichePlus(W9),
    WeichePlus(W10), Hp2(SIGNAL_A), Aux(Bue10_750), BlockIn(BLOCK_AB),
    RouteExc(c1),  // Durchfahrt sollte in umgekehrter Reihenfolge gestellt
                   // sein
    RouteExc(d1),  // Einfahrt in dasselbe Gleis
    Route(a3), WeichePlus(W1a), WeicheMinus(W1b), WeichePlus(W7),
    WeicheMinus(W9), WeichePlus(W10), Hp2(SIGNAL_A), Aux(Bue10_750),
    BlockIn(BLOCK_AB), RouteExc(c3), RouteExc(d3),  //
    Route(b1), WeichePlus(W1a), WeichePlus(W1b), WeichePlus(W2), Hp2(SIGNAL_B),
    Aux(Bue10_750), BlockOut(BLOCK_AB),  //
    Route(b3), WeichePlus(W1a), WeicheMinus(W1b), Hp2(SIGNAL_B), Aux(Bue10_750),
    BlockOut(BLOCK_AB),  //
    Route(c1), WeichePlus(W9), WeichePlus(W10), Hp2(SIGNAL_C),
    BlockOut(BLOCK_CD), RouteExc(b1),  //
    Route(c3), WeichePlus(W7), WeicheMinus(W9), WeichePlus(W10),
    BlockOut(BLOCK_CD), Hp2(SIGNAL_C),  //
    Route(d1), WeichePlus(W1a), WeichePlus(W1b), WeichePlus(W2), WeichePlus(W9),
    WeichePlus(W10), BlockIn(BLOCK_CD), RouteExc(a1), RouteExc(b1),
    Hp2(SIGNAL_D),  //
    Route(d3), WeicheMinus(W1b), WeichePlus(W7), WeicheMinus(W9),
    WeichePlus(W10), Hp2(SIGNAL_D), BlockIn(BLOCK_CD)  //
});

Blinker blinker2{LED_BUILTIN};

/// Arduino setup routine.
void setup() {
  Serial.begin(9600);
  // wait for serial to be up.
  delay(50);
  Serial.println("hello world");
  // Calls the executor to do begin for all registered objects.
  ex.begin();
}

/// Arduino loop routine.
void loop() {
  // Calls the executor to do loop for all registered objects.
  ex.loop();
}
