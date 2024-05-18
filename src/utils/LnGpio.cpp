/** \copyright
 * Copyright (c) 2024, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file LnGpio.cpp
 *
 * Gpio adapters for loconet messages.
 *
 * @author Balazs Racz
 * @date 16 May 2024
 */

#ifdef USE_LNGPIO

#include "utils/LnGpio.h"

#include <LocoNet.h>
#include "utils/Logging.h"


/// What type is the incoming message (primary type)
static LnGpioType ln_type;
/// What address did the incoming message come for
static uint16_t ln_address;
/// Binary value of the incoming message.
static bool ln_value;

/// Alternate type of incoming message. Used for Swith RED and Switch GREEN
/// messages.
static LnGpioType ln_type_b;
/// Alternate value for incoming message.
static bool ln_value_b;

void LnGpio::loop() {
  // Performs clear
  if (clear_tm_.check()) {
    for (unsigned i = 0; i < count_; ++i) {
      auto* p = defs_ + i;
      if (p->type == LNGPIO_SWITCH_GREEN || p->type == LNGPIO_SWITCH_RED) {
        auto pos = get_pos(i);
        set_bit(pos.first, pos.second, true);
      }
    }
  }

  // Check for any received LocoNet packets
  lnMsg *ln_packet;
  ln_packet = ln_->receive();

  ln_type = LNGPIO_NONE;
  ln_type_b = LNGPIO_NONE;
  
  if (ln_packet) {
    ln_->processSwitchSensorMessage(ln_packet);
    if (ln_type == LNGPIO_NONE && ln_type_b == LNGPIO_NONE) {
      // checks for direct button message
      //  [E5 0F 05 08 00 1F 41 68 03 6B 00 01 00 7F 38] 
      if (ln_packet->data[0] == 0xE5 &&
          ln_packet->data[1] == 0x0F &&
          ln_packet->data[2] == 0x05 &&
          ln_packet->data[3] == 0x08 &&
          ln_packet->data[4] == 0x00 &&
          ln_packet->data[5] == 0x1F) {
        uint8_t pxct = ln_packet->data[6];
        for (unsigned i = 0; i < 7; ++i) {
          if (pxct & (1u<<i)) {
            ln_packet->data[7+i] |= 0x80;
          }
        }
        uint16_t p = (ln_packet->data[8] << 8) | ln_packet->data[7];
        uint16_t q = (ln_packet->data[10] << 8) | ln_packet->data[9];
        uint16_t r = (ln_packet->data[12] << 8) | ln_packet->data[11];
        LOG(INFO, "LN ub  cls %04x data %d %04x", p, q, r);
        if (p == 0x3E8) {
          ln_address = q;
          uint16_t btn = r;
          ln_type = LNGPIO_UB_BUTTON_1;
          ln_value = btn & 1;
          ln_type_b = LNGPIO_UB_BUTTON_2;
          ln_value_b = btn & 2;
        }
      }
    }
  }

  if (ln_type == LNGPIO_NONE && ln_type_b == LNGPIO_NONE && !any_dirty_) {
    return; // nothing to do
  }
  bool any_failed = false; // do we need to keep any_dirty?
  for (unsigned i = 0; i < count_; ++i) {
    auto* p = defs_ + i;
    if (ln_type == p->type && ln_address == p->address) {
      auto pos = get_pos(i);
      set_bit(pos.first, pos.second, ln_value);
      set_bit(pos.first, pos.second << 1, false);
      if (ln_type == LNGPIO_UB_BUTTON_1) {
        LOG(INFO, "UB button %d value %d", ln_address, ln_value);
      }
      continue;
    }
    if (ln_type_b == p->type && ln_address == p->address) {
      LOG(INFO, "Switch side id %d, type %d address %d", i, ln_type_b, ln_address); 
      auto pos = get_pos(i);
      set_bit(pos.first, pos.second, ln_value_b);
      set_bit(pos.first, pos.second << 1, false);
      if (ln_type_b == LNGPIO_SWITCH_RED ||
          ln_type_b == LNGPIO_SWITCH_GREEN) {
        clear_tm_.start_oneshot(2000);
      }
      continue;
    }
    auto pos = get_pos(i);
    if (get_bit(pos.first, pos.second << 1)) {
      // This state is dirty. Send update.
      if (send_ln_update(p, get_bit(pos.first, pos.second))) {
        set_bit(pos.first, pos.second << 1, false);
      } else {
        any_failed = true;
      }
    }  // check for dirty
  }    // iterate over bits
  if (!any_failed) {
    any_dirty_ = false;
  }
}

bool LnGpio::send_ln_update(const LnGpioDefn* def, bool value) {
  LN_STATUS st = LN_UNKNOWN_ERROR;
  switch(def->type) {
    case LNGPIO_SWITCH:
      st = ln_->requestSwitch(def->address, false /*output off*/, value /* true = thrown/red false = closed/green*/);
      break;
    case LNGPIO_SENSOR:
      st = ln_->reportSensor(def->address, value);
      break;
    default:
      // ignore
      return true;
  }
  bool ok = (st == LN_DONE) || (st == LN_UNKNOWN_ERROR);
  LOG(LEVEL_INFO, "Ln send type %d address %d value %d: result %d ok %d",
      def->type, def->address, value, st, ok);
  return ok;
}

extern "C" {

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Sensor messages
void notifySensor( uint16_t Address, uint8_t State ) {
  LOG(INFO, "LN Sensor %d - %s", Address, State ? "Active" : "Inactive");
  ln_type = LNGPIO_SENSOR;
  ln_address = Address;
  ln_value = State != 0;
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Request messages
void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction ) {
  LOG(INFO, "LN Switch Req %d - %s - %s", Address, Direction ? "Closed/Grn" : "Thrown/Red", Output ? "On" : "Off");
  ln_address = Address;
  if (Output) {
    ln_type = LNGPIO_SWITCH;
    // true = thrown, false = closed.
    ln_value = Direction != 0;
  }
  if (Direction) {
    ln_type_b = LNGPIO_SWITCH_GREEN;
  } else {
    ln_type_b = LNGPIO_SWITCH_RED;
  }
  ln_value_b = Output;
}

#if 0 
// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Output Report messages
void notifySwitchOutputsReport( uint16_t Address, uint8_t ClosedOutput, uint8_t ThrownOutput) {
  Serial.print("Switch Outputs Report: ");
  Serial.print(Address, DEC);
  Serial.print(": Closed - ");
  Serial.print(ClosedOutput ? "On" : "Off");
  Serial.print(": Thrown - ");
  Serial.println(ThrownOutput ? "On" : "Off");
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Sensor Report messages
void notifySwitchReport( uint16_t Address, uint8_t State, uint8_t Sensor ) {
  Serial.print("Switch Sensor Report: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Sensor ? "Switch" : "Aux");
  Serial.print(" - ");
  Serial.println( State ? "Active" : "Inactive" );
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch State messages
void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction ) {
  Serial.print("Switch State: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(" - ");
  Serial.println(Output ? "On" : "Off");
}

#endif // if 0 -- these are not needed

}

#endif // USE_LNGPIO
