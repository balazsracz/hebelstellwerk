
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

#include <Arduino.h>
#include <Hebelstellwerk.h>

#include "utils/AnalogDemux.h"
#include "utils/ArduinoArmPixel.h"
#include "utils/ArduinoStm32SpiPixel.h"
#include "utils/Blinker.h"
#include "utils/PixelGpio.h"
#include "stw/SimpleBlockUi.h"
#include "stw/DirectBlock.h"

#ifndef ARDUINO
#error baaa
#endif


enum GpioPin : gpio_pin_t {
  ONBOARD_LED = 13,
  ONBOARD_BTN = USER_BTN,

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
};

#define LED_TO_USE 13

Blinker blinker2{LED_TO_USE, 750};

static const int16_t kCenters[] = {913, 786, 683, 559, 461, 346, 254, 171, 93};
AnalogDemux gpio_an{110, PB0, kCenters, sizeof(kCenters) / sizeof(kCenters[0])};
HardwareSerial BlockASerial(PC11 /*rx*/, PC10 /*tx*/);
HardwareSerial BlockBSerial(PD2 /*rx*/, PC12 /*tx*/);
HardwareSerial BlockCSerial(PB11 /*rx*/, PB10 /*tx*/);

SpiPixelStrip strip(9, PA7, PB4, PB3);

const uint32_t kOutputParams[] = {
  0, Pixel::RED, Pixel::WHITE, //
  1, Pixel::RED, Pixel::WHITE, //
  2, Pixel::RED, Pixel::WHITE, //
  3, Pixel::RED, Pixel::WHITE, //
  4, Pixel::RED, Pixel::WHITE, //
  5, Pixel::RED, Pixel::WHITE, //
  6, Pixel::RED, Pixel::WHITE, //
  7, Pixel::RED, Pixel::WHITE, //
  8, Pixel::RED, Pixel::WHITE, //
};

PixelGpio px_gpio{&strip, 120, 9, kOutputParams};

DirectBlock block_a{"A", &BlockASerial};
SimpleBlockUi a_ui{&block_a};
DirectBlock block_b{"B", &BlockBSerial};
SimpleBlockUi b_ui{&block_b};
DirectBlock block_c{"C", &BlockCSerial};
SimpleBlockUi c_ui{&block_c};

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

/// Arduino setup routine.
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(USER_BTN, INPUT_PULLUP);

  Serial.begin(115200);
  // delay(100);
  Serial.println("Hello, world");
  Executor::instance()->begin();
  strip.set_brightness(0x20);

  ASSERT(a_ui_rdy == a_ui.EXPECTED_SETUP);
  ASSERT(b_ui_rdy == b_ui.EXPECTED_SETUP);
  ASSERT(c_ui_rdy == c_ui.EXPECTED_SETUP);
}

#include <vector>

class BlockDebug : public Executable {
 public:
  BlockDebug(HardwareSerial* s) {
    Executor::instance()->add(this);
    s_ = s;
  }

  void begin() override {
    s_->begin(9600, SERIAL_8N1);
    tmAb_.start_oneshot(1);
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
      LOG(INFO, "Block IN: %s", p.c_str());
      packet_.clear();
    }
    if (!digitalRead(USER_BTN) && tmAb_.check()) {
      LOG(INFO, "Abgabe");
      uint8_t abgabe[3] = {0xC0, 0x2C, 0xC0};
      s_->write(abgabe, 3);
      tmAb_.start_oneshot(2000);
    }
  }

 private:
  Timer tm_;
  Timer tmAb_;
  HardwareSerial* s_;
  std::vector<uint8_t> packet_;
};// block_debug(&BlockASerial);

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
};// reporter2;


class Copier : public Executable {
 public:
  Copier() {
    Executor::instance()->add(this);
    tm_.start_periodic(10);
  }

  void begin() override {}
  void loop() override {
    if (!tm_.check()) return;
    auto* s = GpioRegistry::instance()->get(110);
    auto* d = GpioRegistry::instance()->get(120);
    for (unsigned i = 0; i < 9; i++) {
      d->write(120 + i, s->read(110 + i));
    }
  }

 private:
  Timer tm_;
};// copier;


/// Arduino loop routine.
void loop() {
  // Calls the executor to do loop for all registered objects.
  ex.loop();
}
