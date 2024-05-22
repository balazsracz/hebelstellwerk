
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
 * Simple Block operating UI for an Abzweig like Teichholzhagen.
 *
 * @author Balazs Racz
 * @date 19 May 2024
 */

#include <Arduino.h>
#include <Hebelstellwerk.h>

#include "utils/ArduinoStm32SpiPixel.h"
#include "utils/Blinker.h"
#include "utils/PixelGpio.h"
#include "stw/SimpleBlockUi.h"
#include "stw/I2CBlockImpl.h"
#include "stw/DirectBlock.h"

#ifndef ARDUINO
#error baaa
#endif

enum GpioPin : gpio_pin_t {
  ONBOARD_LED = 13,
  ONBOARD_BTN = USER_BTN,

  GPIO_EXT1_BTN_START = 130,

  GPIO_EXT2_BTN_START = 150,

  FLD1_START = GPIO_EXT1_BTN_START,
  FLD1_BTN1 = FLD1_START,
  FLD1_BTN2,
  FLD1_BTN3,
  FLD1_BTN4,
  FLD1_BTN5,
  FLD1_BTN6,

  FLD1_LEDF1,
  FLD1_LEDF2,
  FLD1_LEDF3,
  FLD1_KURBEL,

  FLD1_SW3L,
  FLD1_SW3H,
  FLD1_SW2L,
  FLD1_SW2H,
  FLD1_SW1L,
  FLD1_SW1H,
  FLD1_END,

  FLD2_START = GPIO_EXT2_BTN_START,
  FLD2_BTN1 = FLD2_START,
  FLD2_BTN2,
  FLD2_BTN3,
  FLD2_BTN4,
  FLD2_BTN5,
  FLD2_BTN6,

  FLD2_LEDF1,
  FLD2_LEDF2,
  FLD2_LEDF3,
  FLD2_KURBEL,

  FLD2_SW3L,
  FLD2_SW3H,
  FLD2_SW2L,
  FLD2_SW2H,
  FLD2_SW1L,
  FLD2_SW1H,
  FLD2_END,
  
  BTN_A_RUCKBLOCK = FLD1_BTN4,
  BTN_A_ABGABE = FLD1_BTN5,
  BTN_A_VORBLOCK = FLD1_BTN6,
  BTN_C_VORBLOCK = FLD2_BTN1,
  BTN_C_ABGABE = FLD2_BTN2,
  BTN_C_RUCKBLOCK = FLD2_BTN3,
  BTN_B_VORBLOCK = FLD2_BTN4,
  BTN_B_ABGABE = FLD2_BTN5,
  BTN_B_RUCKBLOCK = FLD2_BTN6,

  PIXEL_START = 120,
  PX_A_ENDFELD = PIXEL_START,
  PX_A_ERLAUBNISFELD,
  PX_A_ANFANGSFELD,
  PX_C_ANFANGSFELD,
  PX_C_ERLAUBNISFELD,
  PX_C_ENDFELD,
  PX_B_ANFANGSFELD,
  PX_B_ERLAUBNISFELD,
  PX_B_ENDFELD,
};

static_assert(FLD1_END - FLD1_START == 16, "fld1 pinout wrong");
static_assert(FLD2_END - FLD2_START == 16, "fld2 pinout wrong");

#define LED_TO_USE 13

Blinker blinker2{LED_TO_USE, 750};

GlobalState st;
GlobalUnlocker unlocker{ONBOARD_BTN, true};

//TwoWire nucleoWire(PB7,PB6);
//TwoWire nucleoWire(D14,D15);
TwoWire& nucleoWire = Wire;

Gpio23017 ext_btn_left(GPIO_EXT1_BTN_START, 0x25, nucleoWire);
Gpio23017 ext_btn_right(GPIO_EXT2_BTN_START, 0x27, nucleoWire);


#if 0
HardwareSerial BlockASerial(PC11 /*rx*/, PC10 /*tx*/);
HardwareSerial BlockBSerial(PD2 /*rx*/, PC12 /*tx*/);
HardwareSerial BlockCSerial(PB11 /*rx*/, PB10 /*tx*/);

HardwareSerial LnSerial(PA10 /*rx*/, PA9 /*tx*/);
#endif

/// External SPI port for pixel driving.
SpiPixelStrip strip(9, PB5, PB4, PB3);

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

I2CBlock block_a{0x28, &nucleoWire}; /// @todo: check
SimpleBlockUi a_ui{&block_a};
I2CBlock block_b{0x2B, &nucleoWire}; /// @todo: check
SimpleBlockUi b_ui{&block_b};
I2CBlock block_c{0x29, &nucleoWire}; /// @todo: check
SimpleBlockUi c_ui{&block_c};

const uint16_t a_ui_rdy = a_ui.set_vorblock_taste(BTN_A_VORBLOCK, true) |
                          a_ui.set_ruckblock_taste(BTN_A_RUCKBLOCK, true) |
                          a_ui.set_abgabe_taste(BTN_A_ABGABE, true) |
                          a_ui.set_anfangsfeld(PX_A_ANFANGSFELD, false) |
                          a_ui.set_endfeld(PX_A_ENDFELD, false) |
                          a_ui.set_erlaubnisfeld(PX_A_ERLAUBNISFELD, false);

const uint16_t b_ui_rdy = b_ui.set_vorblock_taste(BTN_B_VORBLOCK, true) |
                          b_ui.set_ruckblock_taste(BTN_B_RUCKBLOCK, true) |
                          b_ui.set_abgabe_taste(BTN_B_ABGABE, true) |
                          b_ui.set_anfangsfeld(PX_B_ANFANGSFELD, false) |
                          b_ui.set_endfeld(PX_B_ENDFELD, false) |
                          b_ui.set_erlaubnisfeld(PX_B_ERLAUBNISFELD, false);

const uint16_t c_ui_rdy = c_ui.set_vorblock_taste(BTN_C_VORBLOCK, true) |
                          c_ui.set_ruckblock_taste(BTN_C_RUCKBLOCK, true) |
                          c_ui.set_abgabe_taste(BTN_C_ABGABE, true) |
                          c_ui.set_anfangsfeld(PX_C_ANFANGSFELD, false) |
                          c_ui.set_endfeld(PX_C_ENDFELD, false) |
                          c_ui.set_erlaubnisfeld(PX_C_ERLAUBNISFELD, false);

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
      uint8_t abgabe[] = {0xB2,0x39,0x46,0x32};
      s_->write(abgabe, sizeof(abgabe));
    }
  }

 private:
  Timer tm_;
  Timer tmAb_;
  HardwareSerial* s_;
  std::vector<uint8_t> packet_;
};// ln_debug(&LnSerial);

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


class Report3 : public Executable {
 public:
  Report3() {
    Executor::instance()->add(this);
    tm_.start_periodic(1000);
  }

  void begin() override {}
  void loop() override {
    if (!tm_.check()) return;
#if 0    
    ++i;
    if (entsp_a.read() && entsp_b.read() && (i & 1)) {
      GlobalState::instance()->is_unlocked_ ^= 1;
      LOG(LEVEL_INFO, "Global unlock %s",
          GlobalState::instance()->is_unlocked_ ? "true" : "false");
    }
#endif
    // Serial.print("Hello ");
    // Serial.println(i);
    // LOG(LEVEL_INFO, "hello2 1ok %d 2ok %d 3ok %d 4ok %d",
    // ext_hebel_sig_w.ok(), ext_hebel_fstr.ok(), ext_hebel_taster.ok(),
    // ext_detector.ok());
    LOG(LEVEL_INFO,
        "extl %04x extr %04x taster %04x det %04x wire %d blka %d %02x %s blkb %d %02x %s blkc %d %02x %s",
        ext_btn_left.input_states(), ext_btn_right.input_states(),
        ext_btn_left.input_states(), ext_btn_right.input_states(),
        0,
        0, block_a.get_status(),
        block_to_string(block_a.get_status()).c_str(),
        0, block_b.get_status(),
        block_to_string(block_b.get_status()).c_str(),
        0, block_c.get_status(),
        block_to_string(block_c.get_status()).c_str());

    Wire.end();
    Wire.begin();
  }

 private:
  //GpioAccessor entsp_a{GPIO_BTN_ANFANG_C, true, GPIO_INPUT};
  //GpioAccessor entsp_b{GPIO_BTN_ENDFELD_D, true, GPIO_INPUT};
  Timer tm_;
  int i = 0;
} reporter3;


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




/// Arduino setup routine.
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(USER_BTN, INPUT_PULLUP);

  Serial.begin(115200);
  nucleoWire.setSDA(PB7);
  nucleoWire.setSCL(PB6);
  nucleoWire.begin();
  // delay(100);
  Serial.println("Hello, world");

  ASSERT(a_ui_rdy == a_ui.EXPECTED_SETUP);
  ASSERT(b_ui_rdy == b_ui.EXPECTED_SETUP);
  ASSERT(c_ui_rdy == c_ui.EXPECTED_SETUP);
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
      Adafruit_I2CDevice dev(a);//, &nucleoWire);
      if (dev.begin(true)) {
        LOG(LEVEL_INFO, "Found i2c %02x", a);
      }
      delay(10);
    }

    Executor::instance()->begin();
    strip.set_brightness(0x18);
  }
  
  // Calls the executor to do loop for all registered objects.
  ex.loop();
}
