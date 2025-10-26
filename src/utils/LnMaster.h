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
 * \file LnMaster.h
 *
 * Helper functions that operate as a Loconet command station in case we are
 * running a standalone LocoNet.
 *
 * @author Balazs Racz
 * @date 25 Oct 2025
 */

#ifndef _UTILS_LNMASTER_
#define _UTILS_LNMASTER_

/// When true, we start operating as loconet master.
bool g_use_lnmaster = true;
/// Number of "get switch state address 2000" messages we've seen so far.
unsigned num_sw2000 = 0;
/// Number of longack messages we've seen so far.
unsigned num_longack = 0;
uint32_t switch_state[2048/32] = {0};


extern "C" {

extern void notifySwitchState(uint16_t Address, uint8_t Output,
                              uint8_t Direction) {
  if (Address == 2000) {
    ++num_sw2000;
  }
  if (g_use_lnmaster) {
    unsigned a = Address - 1;
    bool is_thrown = (switch_state[a / 32] & (UINT32_C(1) << (a & 31)));
    LocoNet.send(OPC_LONG_ACK, OPC_SW_STATE & 0x7f, is_thrown ? 0x10 : 0x30);
  }
}

extern void notifyLongAck(uint8_t d1, uint8_t d2) { num_longack++; }

extern void secondNotifySwitchRequest(uint16_t Address, uint8_t Output,
                                      uint8_t Direction) {
  LOG(INFO, "LN Switch Req %d - %s - %s", Address, Direction ? "Closed/Grn" : "Thrown/Red", Output ? "On" : "Off");
  if (!Output) return;
  unsigned a = Address - 1;
  if (Direction) {
    switch_state[a / 32] |= UINT32_C(1) << (a & 31);
  } else {
    switch_state[a / 32] &= ~(UINT32_C(1) << (a & 31));
  }
}

}  // extern "C"

#endif // _UTILS_LNMASTER_
