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

  GPIO_SERVO_CAL,
  
  GPIO_KURBEL,

  GPIO_DUMMY,

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
  GPIO_HEBEL_D3,
  GPIO_HEBEL_C3,
  GPIO_HEBEL_B3,
  GPIO_HEBEL_A3,
  GPIO_HEBEL_B1,
  GPIO_HEBEL_A1,

  GPIO_EXT_TASTER,
  GPIO_AB_STORUNSMELDER = GPIO_EXT_TASTER + 0,
  GPIO_AB_HALTSTELLMELDER,
  GPIO_CD_HALTSTELLMELDER,
  GPIO_CD_STORUNSMELDER,
  GPIO_BTN_ENDFELD_D = GPIO_EXT_TASTER + 8,
  GPIO_BTN_ERLAUBNIS_C,
  GPIO_BTN_ANFANG_C,
  GPIO_BTN_FESTLEGE_CD,
  GPIO_BTN_FESTLEGE_AB,
  GPIO_BTN_ANFANG_B,
  GPIO_BTN_ERLAUBNIS_B,
  GPIO_BTN_ENDFELD_A,

  GPIO_EXT_DETECTOR,
  GPIO_AB_DETECTOR = GPIO_EXT_DETECTOR + 3,
  GPIO_CD_DETECTOR = GPIO_EXT_DETECTOR + 1,
  GPIO_EXT_DETECTOR_END = GPIO_EXT_DETECTOR + 15,

};

static_assert(GPIO_EXT_HEBEL_SIG_W + 15 == GPIO_HEBEL_D,
              "misaligned ext hebel sig w");

static_assert(GPIO_EXT_HEBEL_FSTR + 15 == GPIO_HEBEL_A1,
              "misaligned ext hebel fstr");

static_assert(GPIO_EXT_TASTER + 15 == GPIO_BTN_ENDFELD_A,
              "misaligned ext taster");

///@todo add kubel analog gpio
DummyGpio kurbel(GPIO_KURBEL);

DummyGpio dummy(GPIO_DUMMY);

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
                            usec(2430), usec(2550), 0);
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

GlobalState global_state;

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

static constexpr bool SHEB_INV = true;
static constexpr bool SLCK_INV = false;

SignalLever SA{SIGNAL_A, HP2, GPIO_HEBEL_A, SHEB_INV, GPIO_VERRIEGELUNG_A,
               SLCK_INV};
SignalLever SB{SIGNAL_B, HP2, GPIO_HEBEL_B, SHEB_INV, GPIO_VERRIEGELUNG_B,
               SLCK_INV};
SignalLever SC{SIGNAL_C, HP2, GPIO_HEBEL_C, SHEB_INV, GPIO_VERRIEGELUNG_C,
               SLCK_INV};
SignalLever SD{SIGNAL_D, HP2, GPIO_HEBEL_D, SHEB_INV, GPIO_VERRIEGELUNG_D,
               SLCK_INV};

#if 1

static constexpr bool RHEB_INV = true;
static constexpr bool RLCK_INV = true;

RouteLever Rab1(a1, b1, GPIO_HEBEL_A1, RHEB_INV, GPIO_HEBEL_B1, RHEB_INV,
                GPIO_VERRIEGELUNG_FH_A1B1, RLCK_INV);
RouteLever Rab3(a3, b3, GPIO_HEBEL_A3, RHEB_INV, GPIO_HEBEL_B3, RHEB_INV,
                GPIO_VERRIEGELUNG_FH_A3B3, RLCK_INV);
RouteLever Rcd3(c3, d3, GPIO_HEBEL_C3, RHEB_INV, GPIO_HEBEL_D3, RHEB_INV,
                GPIO_VERRIEGELUNG_FH_D3C3, RLCK_INV);
RouteLever Rcd1(c1, d1, GPIO_HEBEL_C1, RHEB_INV, GPIO_HEBEL_D1, RHEB_INV,
                GPIO_VERRIEGELUNG_FH_D1C1, RLCK_INV);

#endif

#if 0

  Block BlkAB(BLOCK_AB, GPIO_AB_DETECTOR, false,
                  GPIO_BTN_FESTLEGE_AB, true, GPIO_FESTLEGE_AB, false);
  Block BlkCD(BLOCK_CD, GPIO_CD_DETECTOR, false,
                  GPIO_BTN_FESTLEGE_CD, true, GPIO_FESTLEGE_CD, false);

#endif

#if 1

/// @todo check the actual I2C addresses.
I2CBlock i2c_ab(0x28);
I2CBlock i2c_cd(0x29);

FelderBlock blk_ab(&i2c_ab, BLOCK_AB, GPIO_AB_DETECTOR, false,
                   GPIO_BTN_FESTLEGE_AB, true, GPIO_FESTLEGE_AB, false);
const auto abrdy = blk_ab.set_vorblock_taste(GPIO_BTN_ANFANG_B, true) |
                   blk_ab.set_ruckblock_taste(GPIO_BTN_ENDFELD_A, true) |
                   blk_ab.set_abgabe_taste(GPIO_BTN_ERLAUBNIS_B, true) |
                   blk_ab.set_kurbel(GPIO_KURBEL, false) |
                   blk_ab.set_anfangsfeld(GPIO_ANFANGS_B, false) |
                   blk_ab.set_endfeld(GPIO_ENDFELD_A, false) |
                   blk_ab.set_erlaubnisfeld(GPIO_ERLAUBNIS_B, false) |
                   /// @todo figure out these GPIO assignments
                   blk_ab.set_signalhaltmelder(GPIO_AB_HALTSTELLMELDER, false) |
                   blk_ab.set_storungsmelder(GPIO_AB_STORUNSMELDER, false) |
                   blk_ab.set_streckentastensperre(GPIO_DUMMY, false);

FelderBlock blk_cd(&i2c_cd, BLOCK_CD, GPIO_CD_DETECTOR, false,
                   GPIO_BTN_FESTLEGE_CD, true, GPIO_FESTLEGE_CD, false);
const auto cdrdy = blk_cd.set_vorblock_taste(GPIO_BTN_ANFANG_C, true) |
                   blk_cd.set_ruckblock_taste(GPIO_BTN_ENDFELD_D, true) |
                   blk_cd.set_abgabe_taste(GPIO_BTN_ERLAUBNIS_C, true) |
                   blk_cd.set_kurbel(GPIO_KURBEL, false) |
                   blk_cd.set_anfangsfeld(GPIO_ANFANGS_C, false) |
                   blk_cd.set_endfeld(GPIO_ENDFELD_D, false) |
                   blk_cd.set_erlaubnisfeld(GPIO_ERLAUBNIS_C, false) |
                   /// @todo figure out these GPIO assignments
                   blk_cd.set_signalhaltmelder(GPIO_CD_HALTSTELLMELDER, false) |
                   blk_cd.set_storungsmelder(GPIO_CD_STORUNSMELDER, false) |
                   blk_cd.set_streckentastensperre(GPIO_DUMMY, false);

#endif

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

std::string block_to_string(uint16_t blk) {
  std::string ret;
  if (blk & BlockBits::STARTUP) ret += "start,";
  if (blk & BlockBits::NEWOUTPUT) ret += "newout,";
  if (blk & BlockBits::NEWINPUT) ret += "newin,";
  if (blk & BlockBits::ERROR) ret += "err,";  
  if (blk & BlockBits::TRACK_OUT) ret += "erlaubnis,";  
  if (blk & BlockBits::HANDOFF) ret += "handoff,";  
  if (blk & BlockBits::IN_BUSY) ret += "in-vorbl,";  
  if (blk & BlockBits::OUT_BUSY) ret += "out-vorbl,";
  if (!ret.empty()) ret.pop_back();
  return ret;
}

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
    if (entsp_a.read() && entsp_b.read() && (i & 1)) {
      GlobalState::instance()->is_unlocked_ ^= 1;
      LOG(LEVEL_INFO, "Global unlock %s",
          GlobalState::instance()->is_unlocked_ ? "true" : "false");
    }
    // Serial.print("Hello ");
    // Serial.println(i);
    // LOG(LEVEL_INFO, "hello2 1ok %d 2ok %d 3ok %d 4ok %d",
    // ext_hebel_sig_w.ok(), ext_hebel_fstr.ok(), ext_hebel_taster.ok(),
    // ext_detector.ok());
    LOG(LEVEL_INFO,
        "hebel %04x fstr %04x taster %04x det %04x blkab %d %02x %s blkcd %d "
        "%02x %s",
        ext_hebel_sig_w.input_states(), ext_hebel_fstr.input_states(),
        ext_hebel_taster.input_states(), ext_detector.input_states(),
        blk_ab.state(), i2c_ab.get_status(),
        block_to_string(i2c_ab.get_status()).c_str(), blk_cd.state(),
        i2c_cd.get_status(), block_to_string(i2c_cd.get_status()).c_str());
  }

 private:
  GpioAccessor entsp_a{GPIO_BTN_ANFANG_C, true, GPIO_INPUT};
  GpioAccessor entsp_b{GPIO_BTN_ENDFELD_D, true, GPIO_INPUT};
  Timer tm_;
  int i = 0;
} reporter;


/// Arduino setup routine.
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, true);
  Serial.begin(9600);
  // wait for serial to be up, up to 1.5 seconds.
  //for (int i = 0; i < 30; ++i) {
  //  if (Serial) break;
  //  delay(50);
  //}
  digitalWrite(LED_BUILTIN, false);
}

/// Arduino loop routine.
void loop() {
  static bool started = false;
  static unsigned next = 50;
  auto m = millis();
  if (m < 3000) {
    if (m > next) {
      Serial.println(m);
      next = m + 50;
    }
    return;
  }
  if (!started) {
    started = true;
    Serial.println("hello world");
    // Scans all i2c devices
    for (unsigned a = 0; a < 128; ++a) {
      Adafruit_I2CDevice dev(a);
      if (dev.begin(true)) {
        LOG(LEVEL_INFO, "Found i2c %02x", a);
      }
      delay(10);
    }

    // Calls the executor to do begin for all registered objects.
    ex.begin();
    blk_ab.check_setup(abrdy);
    blk_cd.check_setup(cdrdy);
    // pwm_chip_servo.set_freq(23500000, 111, 458);
    pwm_chip_servo.set_freq(23500000, 250 / 4, 1024 / 4);
  }
  // Calls the executor to do loop for all registered objects.
  ex.loop();
}
