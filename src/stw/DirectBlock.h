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
 * \file DirectBlock.h
 *
 * Implementation for interacting a Fremo block with direct Serial connection.
 *
 * @author Balazs Racz
 * @date 12 May 2024
 */

#ifndef _STW_DIRECTBLOCK_H_
#define _STW_DIRECTBLOCK_H_

#include "stw/I2CBlock.h"
#include "utils/Executor.h"
#include "utils/Logging.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

class DirectBlock : public I2CBlockInterface, private Executable {
 public:
  DirectBlock(const char* name, HardwareSerial* serial)
      : serial_(serial),
        name_(name),
        pending_vorblocken_(false),
        pending_ruckblocken_(false),
        pending_abgabe_(false),
        pending_esc_(false),
        packet_overflow_(false) {
    Executor::instance()->add(this);
    clear_packet();
  }

  void set_status(uint16_t status) override {
    if ((status & BlockBits::HANDOFF) && !(status_ & BlockBits::HANDOFF)) {
      pending_abgabe_ = 1;
    }
    if ((status_ & BlockBits::TRACK_OUT) && !(status & BlockBits::TRACK_OUT)) {
      pending_abgabe_ = 1;
    }
    if ((status & BlockBits::OUT_BUSY) && !(status_ & BlockBits::OUT_BUSY)) {
      pending_vorblocken_ = 1;
    }
    if ((status_ & BlockBits::IN_BUSY) && !(status & BlockBits::IN_BUSY)) {
      pending_ruckblocken_ = 1;
    }
    status_ = status;
  }

  uint16_t get_status() override { return status_; }

  void ruckblocken() override {
    pending_ruckblocken_ = 1;
    I2CBlockInterface::ruckblocken();
  }

  void vorblocken() override {
    pending_vorblocken_ = 1;
    I2CBlockInterface::vorblocken();
  }

  void abgabe() override {
    pending_abgabe_ = 1;
    I2CBlockInterface::abgabe();
  }

  void begin() override { serial_->begin(9600, SERIAL_8N1); }

  void loop() override {
    while (serial_->available()) {
      process_byte(serial_->read());
    }
    if (pending_abgabe_) {
      serial_->write(kAbgabe, 3);
      pending_abgabe_ = 0;
      clear_bit(BlockBits::TRACK_OUT);
      set_bit(BlockBits::HANDOFF);
      LOG(INFO, "Block %s sent Abgabe", name_);
    }
    if (pending_vorblocken_) {
      serial_->write(kVorblock, 3);
      pending_vorblocken_ = 0;
      set_bit(BlockBits::OUT_BUSY);
      LOG(INFO, "Block %s sent Vorblocken", name_);
    }
    if (pending_ruckblocken_) {
      serial_->write(kRuckblock, 3);
      pending_ruckblocken_ = 0;
      clear_bit(BlockBits::IN_BUSY);
      LOG(INFO, "Block %s sent Ruckblocken", name_);
    }
    if (!pending_vorblocken_ && !pending_ruckblocken_ && !pending_abgabe_) {
      clear_bit(BlockBits::NEWOUTPUT);
    }
  }

 private:
  void process_byte(uint8_t data) {
    if (pending_esc_) {
      switch (data) {
        case kEscEnd:
          append_to_packet(kEnd);
          break;
        case kEscEsc:
          append_to_packet(kEsc);
          break;
        default:
          append_to_packet(data);
          break;
      }
      pending_esc_ = false;
    } else {
      switch (data) {
        case kEnd:
          process_packet();
          break;
        case kEsc:
          pending_esc_ = true;
          break;
        default:
          append_to_packet(data);
          break;
      }
    }
  }

  /// Adds an incoming data byte to the end of the packet. Handles packet
  /// overflow condition.
  void append_to_packet(uint8_t data) {
    if (packet_len_ < sizeof(incoming_packet_)) {
      incoming_packet_[packet_len_++] = data;
    } else {
      packet_overflow_ = true;
    }
  }

  /// Handles the full incoming packet.
  void process_packet() {
    if (!packet_len_) {
      clear_packet();
      return;
    }
    std::string p;
    for (unsigned ofs = 0; ofs < packet_len_; ++ofs) {
      p += StringPrintf("0x%02x ", incoming_packet_[ofs]);
    }
    if (packet_overflow_) {
      p += "OVERFLOW";
    }
    LOG(INFO, "Block %s IN: %s", name_, p.c_str());

    if (packet_len_ == 1) {
      switch (incoming_packet_[0]) {
        case kCmdAbgabe: {
          status_ = RECV_ABGABE_STATUS;
          break;
        }
        case kCmdVorblock: {
          status_ = RECV_VORBLOCK_STATUS;
          break;
        }
        case kCmdRuckblock: {
          status_ = RECV_RUCKBLOCK_STATUS;
          break;
        }
        default:
          break;
      }
    }
    clear_packet();
  }

  void clear_packet() {
    pending_esc_ = false;
    packet_overflow_ = false;
    packet_len_ = 0;
  }

  void set_bit(BlockBits bit) { status_ |= uint16_t(bit); }

  void clear_bit(BlockBits bit) { status_ &= ~uint16_t(bit); }

  static constexpr uint8_t kEnd = 0300;    /* indicates end of packet */
  static constexpr uint8_t kEsc = 0333;    /* indicates byte stuffing */
  static constexpr uint8_t kEscEnd = 0334; /* ESC ESC_END means END data byte */
  static constexpr uint8_t kEscEsc = 0335; /* ESC ESC_ESC means ESC data byte */

  static constexpr uint8_t kCmdAbgabe = 0x2C;
  static constexpr uint8_t kCmdRuckblock = 0x2B;
  static constexpr uint8_t kCmdVorblock = 0x2A;

  static constexpr const uint8_t kAbgabe[] = {0xC0, kCmdAbgabe, 0xC0};
  static constexpr const uint8_t kRuckblock[] = {0xC0, 0x2B, 0xC0};
  static constexpr const uint8_t kVorblock[] = {0xC0, 0x2A, 0xC0};

  static constexpr uint16_t STARTUP_STATUS =
      uint16_t(BlockBits::STARTUP | BlockBits::ERROR);

  /// Status to set after we received an Abgabe message from the remote
  /// station.
  static constexpr uint16_t RECV_ABGABE_STATUS =
      uint16_t(BlockBits::NEWINPUT | BlockBits::TRACK_OUT);

  /// Status to set after we received a Vorblock message from the remote
  /// station.
  static constexpr uint16_t RECV_VORBLOCK_STATUS =
      uint16_t(BlockBits::NEWINPUT | BlockBits::HANDOFF | BlockBits::IN_BUSY);

  /// Status to set after we received a Ruckblock message from the remote
  /// station.
  static constexpr uint16_t RECV_RUCKBLOCK_STATUS =
      uint16_t(BlockBits::NEWINPUT | BlockBits::TRACK_OUT);

  /// Serial port for TX and RX of packets.
  HardwareSerial* serial_;
  /// User-visible name of the block for debug output.
  const char* name_;
  /// Status word exposed to the client via API.
  uint16_t status_{STARTUP_STATUS};
  /// True if we need to send a Vorblocken to the remote station.
  bool pending_vorblocken_ : 1;
  /// True if we need to send a Ruckblocken to the remote station.
  bool pending_ruckblocken_ : 1;
  /// True if we need to send an Abgabe to the remote station.
  bool pending_abgabe_ : 1;
  /// True if the SLIP decoder has seen an escape byte.
  bool pending_esc_ : 1;
  /// True if the SLIP decoder packet has overflowed.
  bool packet_overflow_ : 1;
  /// Buffer for incoming SLIP packet.
  uint8_t incoming_packet_[32];
  /// Number of bytes filled in in incoming_packet_.
  uint8_t packet_len_{0};
};  // class directblock

#endif  // _STW_DIRECTBLOCK_H_
