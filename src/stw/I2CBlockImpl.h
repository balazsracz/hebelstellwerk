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

#ifndef _STW_I2CBLOCKIMPL_H_
#define _STW_I2CBLOCKIMPL_H_

#include "stw/I2CBlock.h"
#include "utils/Executor.h"
#include "utils/Logging.h"
#include "utils/Timer.h"

#ifdef ARDUINO

#include <Adafruit_I2CDevice.h>

class I2CBlock : public I2CBlockInterface, private Executable {
 public:
  /// Constructor
  ///
  /// @param addr 7-bit address of the I2C block interface.
  /// @param theWire optional, I2C bus instance.
  I2CBlock(uint8_t addr, TwoWire *theWire = &Wire) : dev_(addr, theWire) {
    Executor::instance()->add(this);
  }

  void begin() override {
    tm_.start_drifting(POLL_MSEC);

    if (!dev_.begin(true)) {
      LOG(LEVEL_ERROR, "I2C Block addr %02x is not responding", dev_.address());
    } else {
      refresh();
    }
  }

  void loop() override {
    if (dirty_) {
      if (!dev_.write((uint8_t *)&last_status_, 2)) {
        LOG(LEVEL_INFO, "Failed I2C Block %02x status write (st=%02x).", dev_.address(), last_status_);
      } else {
        LOG(LEVEL_INFO, "Success I2C Block %02x status write (st=%02x).", dev_.address(), last_status_);
      }
      dirty_ = false;
      // Prevents a race condition where we query the state again too quickly.
      tm_.start_drifting(POLL_MSEC);
    }
    if (tm_.check()) {
      refresh();
    }
  }

  void set_status(uint16_t status) override {
    if (status != last_status_) {
      dirty_ = true;
      last_status_ = status;
    }
  }

  uint16_t get_status() override { return last_status_; }

 private:
  static constexpr unsigned POLL_MSEC = 43;

  /// Reloads the status word from the i2c device.
  void refresh() {
    if (!dev_.read((uint8_t *)&last_status_, 2)) {
      LOG(LEVEL_INFO, "Failed I2C Block %02x status read.", dev_.address());
    }
#if 0    
    if (last_status_ & BlockBits::STARTUP) {
      last_status_ &= ~(uint16_t)BlockBits::STARTUP;
      dev_.write((uint8_t *)&last_status_, 2);
    }
#endif    
  }

  Adafruit_I2CDevice dev_;

  Timer tm_;
  /// Last read status word.
  uint16_t last_status_{0};

  bool dirty_{false};
  bool last_error_{false};
};

#endif  // Arduino

#endif  // _STW_I2CBLOCKIMPL_H_
