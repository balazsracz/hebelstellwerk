
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
 * \file BlockTest.ino
 *
 * Test for FREMO-Block Schnittstelle
 *
 * @author Balazs Racz
 * @date 28 April 2024
 */

#define USE_LNGPIO

#include <Arduino.h>
#include <Hebelstellwerk.h>
#include <LocoNet.h>

#include "stw/DirectBlock.h"
#include "stw/ErbertBlockUi.h"
#include "stw/SimpleBlockUi.h"
#include "utils/AnalogDemux.h"
#include "utils/ArduinoArmPixel.h"
#include "utils/ArduinoStm32SpiPixel.h"
#include "utils/Blinker.h"
#include "utils/LnGpio.h"
#include "utils/PixelGpio.h"

#ifndef ARDUINO
#error baaa
#endif

enum GpioPin : gpio_pin_t {
  ONBOARD_LED = 13,
  ONBOARD_BTN = USER_BTN,

  BLOCK_DETECTOR = 100,
  
  ANALOG_BTN_START = 110,
  BTN_C_VORBLOCK = ANALOG_BTN_START,
  BTN_C_ABGABE,
  BTN_C_RUCKBLOCK,
  BTN_A_RUCKBLOCK,
  BTN_A_ABGABE,
  BTN_A_VORBLOCK,
  BTN_B_VORBLOCK,
  BTN_B_ABGABE,
  BTN_B_RUCKBLOCK,

  PIXEL_START = 120,
  PX_C_ANFANGSFELD = PIXEL_START,
  PX_C_ERLAUBNISFELD,
  PX_C_ENDFELD,
  PX_A_ENDFELD,
  PX_A_ERLAUBNISFELD,
  PX_A_ANFANGSFELD,
  PX_B_ANFANGSFELD,
  PX_B_ERLAUBNISFELD,
  PX_B_ENDFELD,

  LN_GPIO_START = 140,
  LN_DETECTOR_SEN = LN_GPIO_START,
  LN_BLGT_HMAG,
  
  LN_A_ERLAUBNIS_MAG,
  LN_A_ERLAUBNIS_SEN,
  LN_A_VORGEBLOCKT_SEN,
  LN_A_ERLAUBNIS_BLOCK_MAG,
  LN_A_RT_IN_MAG,
  LN_A_RT_IN_MAG2,
  LN_A_RT_OUT_MAG,
  LN_A_RT_OUT_MAG2,
  LN_A_RT_OUT_BLOCKED_SEN,
  LN_A_RBT_HMAG,

  LN_B_ERLAUBNIS_MAG,
  LN_B_ERLAUBNIS_SEN,
  LN_B_VORGEBLOCKT_SEN,
  LN_B_ERLAUBNIS_BLOCK_MAG,
  LN_B_RT_IN_MAG,
  LN_B_RT_OUT_MAG,
  LN_B_RT_OUT_BLOCKED_SEN,
  LN_B_RBT_HMAG,

  LN_C_ERLAUBNIS_MAG,
  LN_C_ERLAUBNIS_SEN,
  LN_C_VORGEBLOCKT_SEN,
  LN_C_ERLAUBNIS_BLOCK_MAG,
  LN_C_RT_IN_MAG,
  LN_C_RT_OUT_MAG,
  LN_C_RT_OUT_BLOCKED_SEN,
  LN_C_RBT_HMAG,

  LN_SEN_1020_LOW,
  LN_SEN_1030_LOW,
  LN_SEN_2010_LOW,
  LN_SEN_3010_LOW,
  
  LN_GPIO_END,
};

#include "stw/EisenbachBelegtmelder.h"

#define LED_TO_USE 13

Blinker blinker2{LED_TO_USE, 750};

GlobalState st;
GlobalUnlocker unlocker{ONBOARD_BTN, true};

static const int16_t kCenters[] = {913, 786, 683, 559, 461, 346, 254, 171, 93};
AnalogDemux gpio_an{110, PB0, kCenters, sizeof(kCenters) / sizeof(kCenters[0])};
HardwareSerial BlockASerial(PC11 /*rx*/, PC10 /*tx*/);
HardwareSerial BlockBSerial(PD2 /*rx*/, PC12 /*tx*/);
HardwareSerial BlockCSerial(PB11 /*rx*/, PB10 /*tx*/);

// HardwareSerial LnSerial(PA10 /*rx*/, PA9 /*tx*/);

// This is the built-in stirp
SpiPixelStrip strip(9, PA7, PB4, PB3);
// This is the strip on the external SPI line
// SpiPixelStrip strip(9, PB5, PB4, PB3);

const uint32_t kOutputParams[] = {
    0, Pixel::RED, Pixel::WHITE,  //
    1, Pixel::RED, Pixel::WHITE,  //
    2, Pixel::RED, Pixel::WHITE,  //
    3, Pixel::RED, Pixel::WHITE,  //
    4, Pixel::RED, Pixel::WHITE,  //
    5, Pixel::RED, Pixel::WHITE,  //
    6, Pixel::RED, Pixel::WHITE,  //
    7, Pixel::RED, Pixel::WHITE,  //
    8, Pixel::RED, Pixel::WHITE,  //
};

PixelGpio px_gpio{&strip, 120, 9, kOutputParams};

DirectBlock block_a{"A", &BlockASerial, PC11, PC1};
SimpleBlockUi a_ui{&block_a};
ErbertBlockUi a_eui{"A", &block_a};
DirectBlock block_b{"B", &BlockBSerial, PD2, PA1};
SimpleBlockUi b_ui{&block_b};
ErbertBlockUi b_eui{"B", &block_b};
DirectBlock block_c{"C", &BlockCSerial, PB11, PA15};
SimpleBlockUi c_ui{&block_c};
ErbertBlockUi c_eui{"C", &block_c};

const uint16_t a_ui_rdy = a_ui.set_vorblock_taste(BTN_A_VORBLOCK, false) |
                          a_ui.set_ruckblock_taste(BTN_A_RUCKBLOCK, false) |
                          a_ui.set_abgabe_taste(BTN_A_ABGABE, false) |
                          a_ui.set_anfangsfeld(PX_A_ANFANGSFELD, false) |
                          a_ui.set_endfeld(PX_A_ENDFELD, false) |
                          a_ui.set_erlaubnisfeld(PX_A_ERLAUBNISFELD, false);

const uint16_t b_ui_rdy = b_ui.set_vorblock_taste(BTN_B_VORBLOCK, false) |
                          b_ui.set_ruckblock_taste(BTN_B_RUCKBLOCK, false) |
                          b_ui.set_abgabe_taste(BTN_B_ABGABE, false) |
                          b_ui.set_anfangsfeld(PX_B_ANFANGSFELD, false) |
                          b_ui.set_endfeld(PX_B_ENDFELD, false) |
                          b_ui.set_erlaubnisfeld(PX_B_ERLAUBNISFELD, false);

const uint16_t c_ui_rdy = c_ui.set_vorblock_taste(BTN_C_VORBLOCK, false) |
                          c_ui.set_ruckblock_taste(BTN_C_RUCKBLOCK, false) |
                          c_ui.set_abgabe_taste(BTN_C_ABGABE, false) |
                          c_ui.set_anfangsfeld(PX_C_ANFANGSFELD, false) |
                          c_ui.set_endfeld(PX_C_ENDFELD, false) |
                          c_ui.set_erlaubnisfeld(PX_C_ERLAUBNISFELD, false);

const LnGpioDefn ln_defs[] = {
    {LNGPIO_SENSOR, 1},        // LN_DETECTOR_SEN
    {LNGPIO_SWITCH_GREEN, 604},  // LN_BLGT_HMAG,

    // A is module 301, art ns. 3204
    // Erlaubnis 700. thrown is OUT
    // 710 = Erlaubnisabgabesperre (CV38)
    // CV22 = CV24 = 57. Busy (rotausleuchtung).
    {LNGPIO_SWITCH, 700},      // LN_A_ERLAUBNIS_MAG, 2
    {LNGPIO_SENSOR, 52},      // LN_A_ERLAUBNIS_SEN
    {LNGPIO_SENSOR, 57},      // LN_A_VORGEBLOCKT_SEN
    {LNGPIO_SWITCH, 710},      // LN_A_ERLAUBNIS_BLOCK_MAG ROT= blocked, 5
    {LNGPIO_SENSOR, 1020},      // LN_A_RT_IN_MAG route in magnetartikel
    {LNGPIO_SENSOR, 1030},      // LN_A_RT_IN_MAG2 route in magnetartikel
    {LNGPIO_SENSOR, 3010},      // LN_A_RT_OUT_MAG route in magnetartikel
    {LNGPIO_SENSOR, 2010},      // LN_A_RT_OUT_MAG2 route in magnetartikel
    {LNGPIO_SENSOR, 904},      // LN_A_RT_OUT_BLOCKED_SEN, 10
    {LNGPIO_UB_BUTTON_1, 402},  // LN_A_RBT_HMAG,

    // B is module 308, art nr. 3204
    // CV87 = 731
    // CV88 (richtung 1) = 502
    // CV89 (richtung 2) = 503
    // Sperre (cv38) = 730
    // CV22 = CV24 = 55. Busy (rotausleuchtung).
    //
    // SigB. Zieltaste 358 / 0
    // Route B/in set: sen 2010 HIGH is set, LOW is clear.
    {LNGPIO_SWITCH, 731},      // LN_B_ERLAUBNIS_MAG, 12
    {LNGPIO_SENSOR, 50},      // LN_B_ERLAUBNIS_SEN
    {LNGPIO_SENSOR, 55},      // LN_B_VORGEBLOCKT_SEN
    {LNGPIO_SWITCH, 730},      // LN_B_ERLAUBNIS_BLOCK_MAG ROT= blocked, 15
    {LNGPIO_SENSOR, 2010},      // LN_B_RT_IN_MAG route in magnetartikel
    {LNGPIO_SENSOR, 1020},      // LN_B_RT_OUT_MAG route in magnetartikel
    {LNGPIO_SENSOR, 910},      // LN_B_RT_OUT_BLOCKED_SEN
    {LNGPIO_UB_BUTTON_1, 407},  // LN_B_RBT_HMAG,

    // C is modul 208, art nr. 3204
    // Addresse 400's are free.
    // cv87 = 721
    // cv88 (richtung 1) = 512
    // cv89 (richtung 2) = 513
    // cv10 = 603 (eagt 603 gn)
    // cv38 = 720
    // CV22 = CV24 = 54. Busy (rotausleuchtung).
    {LNGPIO_SWITCH, 721},      // LN_C_ERLAUBNIS_MAG, 20
    {LNGPIO_SENSOR, 51},      // LN_C_ERLAUBNIS_SEN
    {LNGPIO_SENSOR, 54},      // LN_C_VORGEBLOCKT_SEN
    {LNGPIO_SWITCH, 720},      // LN_C_ERLAUBNIS_BLOCK_MAG ROT= blocked
    {LNGPIO_SENSOR, 3010},      // LN_C_RT_IN_MAG route in magnetartikel
    {LNGPIO_SENSOR, 1030},      // LN_C_RT_OUT_MAG route in magnetartikel, 25
    {LNGPIO_SENSOR, 915},      // LN_C_RT_OUT_BLOCKED_SEN
    {LNGPIO_UB_BUTTON_1, 107},  // LN_C_RBT_HMAG, 27

    {LNGPIO_SENSOR_LOW_EVENT, 1020},
    {LNGPIO_SENSOR_LOW_EVENT, 1030},
    {LNGPIO_SENSOR_LOW_EVENT, 2010},
    {LNGPIO_SENSOR_LOW_EVENT, 3010}, // 31
};

static_assert(LN_GPIO_END - LN_GPIO_START == ARRAYSIZE(ln_defs),
              "Loconet GPIO misaligned");

LnGpio ln_gpio{LN_GPIO_START, &LocoNet, ln_defs, ARRAYSIZE(ln_defs)};
EisenbachBelegtmelder block_det{BLOCK_DETECTOR};

const uint16_t a_eui_rdy =
    a_eui.set_erlaubnis_magnetart(LN_A_ERLAUBNIS_MAG, true) |  //
    a_eui.set_erlaubnis_sen(LN_A_ERLAUBNIS_SEN, true) |  //
    a_eui.set_block_busy_sensor(LN_A_VORGEBLOCKT_SEN, false) |  //
    a_eui.set_erlaubnis_blocked_magnetart(
        LN_A_ERLAUBNIS_BLOCK_MAG, true) |  // inverted, because blocked is true
                                           // but receiver needs RED to block.
    a_eui.set_route_in_gpio(LN_A_RT_IN_MAG, false) |
    a_eui.set_route_in_gpio2(LN_A_RT_IN_MAG2, false) |
    a_eui.set_route_out_gpio(LN_A_RT_OUT_MAG, false) |
    a_eui.set_route_out_gpio2(LN_A_RT_OUT_MAG2, false) |
    a_eui.set_detector_gpio(BLOCK_DETECTOR, false) |
    a_eui.set_route_out_blocked(LN_A_RT_OUT_BLOCKED_SEN, false) |
    a_eui.set_rbt(LN_A_RBT_HMAG, false) |  //
    a_eui.set_blgt(LN_BLGT_HMAG, true);

const uint16_t b_eui_rdy =
    b_eui.set_erlaubnis_magnetart(LN_B_ERLAUBNIS_MAG, true) |  //
    b_eui.set_erlaubnis_sen(LN_B_ERLAUBNIS_SEN, true) |  //
    b_eui.set_block_busy_sensor(LN_B_VORGEBLOCKT_SEN, false) |  //
    b_eui.set_erlaubnis_blocked_magnetart(
        LN_B_ERLAUBNIS_BLOCK_MAG, true) |  // inverted, because blocked is true
                                           // but receiver needs RED to block.
    b_eui.set_route_in_gpio(LN_B_RT_IN_MAG, false) |
    b_eui.set_route_out_gpio(LN_B_RT_OUT_MAG, false) |
    b_eui.set_detector_gpio(BLOCK_DETECTOR, false) |
    b_eui.set_route_out_blocked(LN_B_RT_OUT_BLOCKED_SEN, false) |
    b_eui.set_rbt(LN_B_RBT_HMAG, false) |  //
    b_eui.set_blgt(LN_BLGT_HMAG, true);

const uint16_t c_eui_rdy =
    c_eui.set_erlaubnis_magnetart(LN_C_ERLAUBNIS_MAG, true) |  //
    c_eui.set_erlaubnis_sen(LN_C_ERLAUBNIS_SEN, true) |  //
    c_eui.set_block_busy_sensor(LN_C_VORGEBLOCKT_SEN, false) |  //
    c_eui.set_erlaubnis_blocked_magnetart(
        LN_C_ERLAUBNIS_BLOCK_MAG, true) |  // inverted, because blocked is true
                                           // but receiver needs RED to block.
    c_eui.set_route_in_gpio(LN_C_RT_IN_MAG, false) |
    c_eui.set_route_out_gpio(LN_C_RT_OUT_MAG, false) |
    c_eui.set_detector_gpio(BLOCK_DETECTOR, false) |
    c_eui.set_route_out_blocked(LN_C_RT_OUT_BLOCKED_SEN, false) |
    c_eui.set_rbt(LN_C_RBT_HMAG, false) |  //
    c_eui.set_blgt(LN_BLGT_HMAG, true);

// Blinker blinkerln{LN_GPIO_START, 2000};

/// Arduino setup routine.
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(USER_BTN, INPUT_PULLUP);

  pinMode(PC2, INPUT_PULLUP);
  pinMode(PA9, INPUT_PULLUP);
  // digitalWrite(PA9, HIGH);
  pinMode(PC5, OUTPUT);
  digitalWrite(PC5, LOW);

  Serial.begin(115200);
  // delay(100);
  Serial.println("Hello, world");

  LocoNet.init(PC5);

  Executor::instance()->begin();
  strip.set_brightness(0x20);

  ASSERT(a_ui_rdy == a_ui.EXPECTED_SETUP);
  ASSERT(b_ui_rdy == b_ui.EXPECTED_SETUP);
  ASSERT(c_ui_rdy == c_ui.EXPECTED_SETUP);

  ASSERT(a_eui_rdy == a_eui.EXPECTED_SETUP);
  ASSERT(b_eui_rdy == a_eui.EXPECTED_SETUP);
  ASSERT(c_eui_rdy == a_eui.EXPECTED_SETUP);
}

#include <vector>

class BlockDebug : public Executable {
 public:
  BlockDebug(HardwareSerial* s) {
    Executor::instance()->add(this);
    s_ = s;
  }

  void begin() override {
    tmAb_.start_oneshot(1);
    s_->begin(16660, SERIAL_8N1);
  }

  void loop() override {
    while (s_->available()) {
      packet_.push_back(s_->read());
      tm_.start_oneshot(50);
    }
    if (packet_.size() && tm_.check()) {
      std::string p;
      for (uint8_t e : packet_) {
        p += StringPrintf("0x%02x ", e);
      }
      LOG(INFO, "LocoNet IN: %s", p.c_str());
      packet_.clear();
    }
    if (!digitalRead(USER_BTN) && tmAb_.check()) {
      tmAb_.start_oneshot(2000);
      LOG(INFO, "Sensor message");
      uint8_t abgabe[] = {0xB2, 0x39, 0x46, 0x32};
      s_->write(abgabe, sizeof(abgabe));
    }
  }

 private:
  Timer tm_;
  Timer tmAb_;
  HardwareSerial* s_;
  std::vector<uint8_t> packet_;
};  // ln_debug(&LnSerial);

class Analog : public Executable {
 public:
  Analog() {
    Executor::instance()->add(this);
    tm_.start_periodic(100);
  }

  void begin() override {}
  void loop() override {
    if (!tm_.check()) return;
    auto v = analogRead(PB0);
    LOG(LEVEL_INFO, "analog %d", v);
  }

 private:
  Timer tm_;
};  // reporter2;

class Copier : public Executable {
 public:
  Copier() {
    Executor::instance()->add(this);
    tm_.start_periodic(1);
  }

  void begin() override {}
  void loop() override {
    if (!tm_.check()) return;
    auto* s = GpioRegistry::instance()->get(110);
    auto* d = GpioRegistry::instance()->get(120);
    auto* l = GpioRegistry::instance()->get(LN_GPIO_START);
    for (unsigned i = 0; i < 9; i++) {
      // d->write(120 + i, s->read(110 + i));
    }
    // l->write(LN_TURNOUT_SENSOR, s->read(110 + 0));
    //  l->write(LN_SW_TEST, s->read(110 + 1));
    //  d->write(120 + 0, l->read(LN_SW_RED));
    //  d->write(120 + 1, l->read(LN_SW_GREEN));
  }

 private:
  Timer tm_;
} copier;

/// Arduino loop routine.
void loop() {
  // Calls the executor to do loop for all registered objects.
  ex.loop();
}
