
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

#ifndef ARDUINO
#error baaa
#endif

#define LED_TO_USE 13

Blinker blinker2{LED_TO_USE, 750};

static const int16_t kCenters[] = {913, 786, 683, 559, 461, 346, 254, 171, 93};
AnalogDemux gpio_an{110, PB0, kCenters, sizeof(kCenters) / sizeof(kCenters[0])};

HardwareSerial BlockASerial(PC11 /*rx*/, PC10 /*tx*/);

SpiPixelStrip strip(9, PA7, PB4, PB3);

class PxGpio : public DummyGpio {
 public:
  PxGpio() { GpioRegistry::instance()->register_obj(this, 101); }

  void write(gpio_pin_t, bool value) const override {
    if (value) {
      for (int i = 0; i < 9; i++) {
        strip.set(i, i % 3, 0x3f);
        strip.set(i, (i + 1) % 3, 0);
        strip.set(i, (i + 2) % 3, 0);
      }
    } else {
      for (int i = 0; i < 9; i++) {
        strip.set(i, i % 3, 0);
        strip.set(i, (i + 1) % 3, 0x3f);
        strip.set(i, (i + 2) % 3, 0);
      }
    }
  }
} pxgpio_;

Blinker blinker3{101, 350};

/// Arduino setup routine.
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(USER_BTN, INPUT_PULLUP);

  Serial.begin(115200);
  // delay(100);
  Serial.println("Hello, world");
  Executor::instance()->begin();
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
} block_debug(&BlockASerial);

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
} reporter2;

/// Arduino loop routine.
void loop() {
  // Calls the executor to do loop for all registered objects.
  ex.loop();
}
