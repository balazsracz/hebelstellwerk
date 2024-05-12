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
 * \file I2CBlock.h
 *
 * Interface and implementation for interacting with the I2C FremoBlock module.
 *
 * @author Balazs Racz
 * @date 6 Apr 2024
 */

#ifndef _STW_I2CBLOCK_H_
#define _STW_I2CBLOCK_H_

#include <string>
#include <stdint.h>

enum class BlockBits : uint16_t {
  /// The block is in startup mode and it does not know what the status bits
  /// should be.
  STARTUP = 0x08,
  /// A piece of information is written from the local operator into the
  /// status, and this is still pending transmission to the remote station.
  NEWOUTPUT = 0x02,
  /// A new piece of information has arrived from the opposing station.
  NEWINPUT = 0x04,
  /// 1 when there is a transmission error on the line.
  ERROR = 0x01,
  /// 1 = we have the trackage rights (==out). 0 = the opposite station has the
  /// trackage rights (== in)
  TRACK_OUT = 0x10,
  /// 1 = we are handing off the track to the opposing station. 0 = we are not
  /// handing off the track.
  HANDOFF = 0x20,
  /// 1 = there is an outbounds train on the track. 0 = there is no outbounds
  /// train on the track.
  OUT_BUSY = 0x40,
  /// 1 = there is an inbounds train on the track. 0 = there is no inbounds
  /// train on the track.
  IN_BUSY = 0x80,
};

static inline constexpr BlockBits operator|(BlockBits a, BlockBits b) {
  uint16_t r = ((uint16_t)a) | ((uint16_t)b);
  return BlockBits(r);
}

static inline constexpr uint16_t operator&(uint16_t a, BlockBits b) {
  uint16_t r =  a & ((uint16_t)b);
  return r;
}

static inline std::string block_to_string(uint16_t blk) {
  std::string ret;
  if (blk & BlockBits::STARTUP) ret += "Start,";
  if (blk & BlockBits::NEWOUTPUT) ret += "DatNeuFeld,";
  if (blk & BlockBits::NEWINPUT) ret += "DatNeuBlock,";
  if (blk & BlockBits::ERROR) ret += "Störung,";  
  if (blk & BlockBits::TRACK_OUT) ret += "ErlaubWir,";  
  if (blk & BlockBits::HANDOFF) ret += "ErlaubAnd,";  
  if (blk & BlockBits::IN_BUSY) ret += "EndeRot,";  
  if (blk & BlockBits::OUT_BUSY) ret += "AnfangRot,";
  if (!ret.empty()) ret.pop_back();
  return ret;
}

class I2CBlockInterface {
 public:
  /// Sets the status bits. Currently only the lower 8 bits are defined.
  /// @param status the new status bits to write to the block interface module.
  virtual void set_status(uint16_t status) = 0;

  /// Adds some bits to the status word. This is usually used to send a
  /// message.
  void send_bit(BlockBits bit) {
    set_status(get_status() | (uint16_t)bit | (uint16_t)BlockBits::NEWOUTPUT);
  }

  /// Removes the incoming notification bit. This bit is set by the peripheral
  /// when an incoming message changes the state.
  void clear_incoming_notify() {
    set_status(get_status() & ~(uint16_t)BlockBits::NEWINPUT);
  }

  /// @return true if there is a new message from the opposing side.
  bool has_incoming_notify() {
    return get_status() & (uint16_t)BlockBits::NEWINPUT;
  }

  /// Requests the handoff (Erlaubnisabgabe).
  virtual void abgabe() {
    uint16_t ost = get_status();
    uint16_t nst =
        (ost | (uint16_t)BlockBits::HANDOFF | (uint16_t)BlockBits::NEWOUTPUT) &
        ~(uint16_t)BlockBits::TRACK_OUT;
    set_status(nst);
  }

  /// Requests the out_busy state (Vorblocken)
  virtual void vorblocken() {
    send_bit(BlockBits::OUT_BUSY | BlockBits::TRACK_OUT);
  }

  /// Resets the block state to in_busy, in which state it is possible to send
  /// a Rückblocken message.
  void reset_endfeld_rot() {
    set_status((uint16_t)(BlockBits::HANDOFF | BlockBits::IN_BUSY |
                          BlockBits::NEWOUTPUT));
  }

  /// Resets the block state to in_free.
  void reset_in() {
    set_status((uint16_t)(BlockBits::HANDOFF | BlockBits::NEWOUTPUT));
  }
  
  /// Resets the block state to out_busy.
  void reset_anf_rot() {
    set_status((uint16_t)(BlockBits::TRACK_OUT | BlockBits::OUT_BUSY |
                          BlockBits::NEWOUTPUT));
  }

  /// Resets the block state to out_free.
  void reset_out() {
    set_status((uint16_t)(BlockBits::TRACK_OUT | BlockBits::NEWOUTPUT));
  }
  
  /// Requests the in_free state (Rückblocken)
  virtual void ruckblocken() {
    uint16_t ost = get_status();
    uint16_t nst =
        (ost | (uint16_t)BlockBits::HANDOFF | (uint16_t)BlockBits::NEWOUTPUT) &
        ~uint16_t(BlockBits::IN_BUSY | BlockBits::TRACK_OUT);
    set_status(nst);
  }
  
  /// Gets the current status bits.
  /// @return the last known status bits from the block interface module.
  virtual uint16_t get_status() = 0;
};


#endif //_STW_I2CBLOCK_H_
