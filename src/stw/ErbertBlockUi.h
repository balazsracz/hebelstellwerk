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
 * \file ErbertBlockUi.h
 *
 * Block user interface that matches the input/output characteristics of an
 * Erbert DrS Stellwerk via LocoNet.
 *
 * @author Balazs Racz
 * @date 16 May 2024
 */

#ifndef _STW_ERBERTBLOCKUI_H_
#define _STW_ERBERTBLOCKUI_H_

class ErbertBlockUi : public Executable {
 public:
  ErbertBlockUi(const char* name, I2CBlockInterface* block)
      : block_(block),
        name_(name),
        shadow_erlaubnis_(true),
        seen_route_out_(false),
        seen_route_in_(false),
        seen_train_(false) {
    Executor::instance()->add(this);
  }

  void begin() override {
    tm_.start_drifting(16);
  }

  void loop() override {
    if (!tm_.check()) return;
    tm_.start_drifting(16);
    
    // Updates output fields.
    auto state = block_->get_status();

    if (state & BlockBits::NEWOUTPUT) {
      // Wait with the state machine until the outputs to the block interface
      // are completed.
      return;
    }

    /// @todo add code to set all loconet outputs back and forth.
    
    // ====== ROUTE BLOCKED ======
    route_out_blocked_.write(!(state & BlockBits::TRACK_OUT) ||
                             (state & BlockBits::OUT_BUSY));

    // ====== ERLAUBNIS =====
    if ((state & BlockBits::TRACK_OUT) && !shadow_erlaubnis_) {
      // We received Erlaubnis from the remote station.
      set_erlaubnis_out_ui(true);
      seen_route_in_ = false;
      return;
    }

    if (!erlaubnis_out_.read() && shadow_erlaubnis_) {
      // UI handed off erlaubnis.
      block_->abgabe();
      set_erlaubnis_out_ui(false);
      seen_route_out_ = false;
      return;
    }
    // Checks if the UI reversed the handoff when it should not have.
    if ((state & BlockBits::HANDOFF) && erlaubnis_out_.read()) {
      if (need_force_erlaubnis_) {
        set_erlaubnis_out_ui(false);
      } else {
        need_force_erlaubnis_ = true;
        // delay a while with our reaction.
        tm_.start_oneshot(300);
      }
      return;
    }
    if (erlaubnis_out_sen_.read() != shadow_erlaubnis_) {
      // Update sensor.
      erlaubnis_out_sen_.write(shadow_erlaubnis_);
    }

    // ====== VORBLOCKEN ======
    if (shadow_erlaubnis_ && !seen_route_out_) {
      if (block_route_out_.read() ||
          (block_route_out2_.has() && block_route_out2_.read())) {
        seen_route_out_ = true;
      }
    }
    if (!(state & BlockBits::OUT_BUSY) && seen_route_out_ && detector_.read()) {
      block_->vorblocken();
      seen_route_out_ = false;
    }

    // ====== RECV RÜCKBLOCKEN ======
    if ((state & BlockBits::TRACK_OUT) && !(state & BlockBits::OUT_BUSY) &&
        (state & BlockBits::NEWINPUT)) { 
      seen_route_out_ = false;
      block_->clear_incoming_notify();
    }

    // ====== RECV VORBLOCKEN ======
    if ((state & BlockBits::HANDOFF) && (state & BlockBits::IN_BUSY) &&
        (state & BlockBits::NEWINPUT)) { 
      seen_route_in_ = false;
      seen_train_ = false;
      block_->clear_incoming_notify();
    }

    // ====== RÜCKBLOCKEN ======
    if (!shadow_erlaubnis_ && !seen_route_in_) {
      if (block_route_in_.read() ||
          (block_route_in2_.has() && block_route_in2_.read())) {
        LOG(INFO, "Ruckblocken %s: seen route in.", name_);
        seen_route_in_ = true;
      }
    }
    if (seen_route_in_ && !seen_train_ && detector_.read()) {
      LOG(INFO, "Ruckblocken %s: seen train.", name_);
      seen_train_ = true;
    }
    if (seen_route_in_ && seen_train_) {
      if (debug_tm_.check()) {
        LOG(INFO, "ruckblocken %s: rbt %d blgt %d", name_, rbt_.read(), blgt_.read());
      }
    }
    if ((state & BlockBits::IN_BUSY) && seen_route_in_ && seen_train_ && rbt_.read() && blgt_.read()) {
      LOG(INFO, "Ruckblocken %s.", name_);
      block_->ruckblocken();
      seen_route_in_ = false;
      seen_train_ = false;
    }

    block_busy_.write((state & BlockBits::IN_BUSY) ||
                      (state & BlockBits::OUT_BUSY));
  }

  /// Sets a GPIO which is linked to a loconet magnetartikel (both for input
  /// and output) for controlling the trackage direction LED. True (normally
  /// closed/green) means OUT, false (normally thrown / red) means IN/handoff.
  uint16_t set_erlaubnis_magnetart(gpio_pin_t pin, bool inverted) {
    erlaubnis_out_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u << 0;
  }

  /// Secondary IO for Erlaubnis. This should be set to a Sensor. We will write
  /// this with the Erlaubnis value that is correct, after the
  /// Magnetartikel. True if we do have the erlaubnis. Will be write only.
  uint16_t set_erlaubnis_sen(gpio_pin_t pin, bool inverted) {
    erlaubnis_out_sen_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u << 9;
  }
  
  /// Output (only) GPIO that will be set to true when the block is busy
  /// (vorgeblockt). This is linked to a Sensor message in LocoNet that will
  /// turn the lines red there.
  uint16_t set_block_busy_sensor(gpio_pin_t pin, bool inverted) {
    block_busy_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u << 1;
  }

  /// Sets a GPIO which is linked to a loconet magnetartikel for output, which
  /// blocks the Erlaubnistaster. True (normally closed/green) means it is
  /// blocked. Has to be inverted for Erbert.
  uint16_t set_erlaubnis_blocked_magnetart(gpio_pin_t pin, bool inverted) {
    erlaubnis_blocked_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u << 2;
  }

  /// Input GPIO that is true when there is an incoming route set through this
  /// block.
  uint16_t set_route_in_gpio(gpio_pin_t pin, bool inverted) {
    block_route_in_.setup(pin, inverted, GPIO_INPUT);
    return 1u << 3;
  }

  /// Input GPIO that is true when there is an incoming route set through this
  /// block (alternate).
  uint16_t set_route_in_gpio2(gpio_pin_t pin, bool inverted) {
    block_route_in2_.setup(pin, inverted, GPIO_INPUT);
    return 0;
  }

  /// Input GPIO that is true when there is an outgoing route set through this
  /// block.
  uint16_t set_route_out_gpio(gpio_pin_t pin, bool inverted) {
    block_route_out_.setup(pin, inverted, GPIO_INPUT);
    return 1u << 4;
  }

  /// Input GPIO that is true when there is an outgoing route set through this
  /// block (alternate).
  uint16_t set_route_out_gpio2(gpio_pin_t pin, bool inverted) {
    block_route_out2_.setup(pin, inverted, GPIO_INPUT);
    return 0;
  }

  /// Detector: true if there is a train on it.
  uint16_t set_detector_gpio(gpio_pin_t pin, bool inverted) {
    detector_.setup(pin, inverted, GPIO_INPUT);
    return 1u << 5;
  }

  /// Will be set to true to block outgoing routes.
  uint16_t set_route_out_blocked(gpio_pin_t pin, bool inverted) {
    route_out_blocked_.setup(pin, inverted, GPIO_INPUT);
    return 1u << 6;
  }

  /// Input: RbT. True when pressed.
  uint16_t set_rbt(gpio_pin_t pin, bool inverted) {
    rbt_.setup(pin, inverted, GPIO_INPUT);
    return 1u << 7;
  }

  /// Input: BlGT. True when pressed.
  uint16_t set_blgt(gpio_pin_t pin, bool inverted) {
    blgt_.setup(pin, inverted, GPIO_INPUT);
    return 1u << 8;
  }
  
  /// These bits should be set after all the setup is done.
  static constexpr uint16_t EXPECTED_SETUP = (1u << 10) - 1;
  
  
 private:
  void set_erlaubnis_out_ui(bool has_out) {
    shadow_erlaubnis_ = has_out;
    erlaubnis_out_.write(has_out);
    erlaubnis_blocked_.write(!has_out);
    need_force_erlaubnis_ = false;
  }

  /// I-O that determines whether we have the right to send a train
  /// (Erlaubnis). True if we do have the erlaubnis. Will be both read and
  /// written.
  DelayedGpioAccessor erlaubnis_out_;

  /// Secondary IO for Erlaubnis. This should be set to a Sensor. We will write
  /// this with the Erlaubnis value that is correct, after the
  /// Magnetartikel. True if we do have the erlaubnis. Will be write only.
  DelayedGpioAccessor erlaubnis_out_sen_;
  
  /// Output, will be set to true to block the Erlaubnis button when we don't
  /// have the Erlaubnis. This prevents unilaterally taking the Erlaubnis from
  /// the remote station.
  DelayedGpioAccessor erlaubnis_blocked_;
  
  /// Output, will be set to true to block the setting of an outgoing route.
  DelayedGpioAccessor route_out_blocked_;

  /// Output (only) GPIO that will be set to true when the block is busy
  /// (vorgeblockt). This is linked to a Sensor message in LocoNet that will
  /// turn the lines red there.
  DelayedGpioAccessor block_busy_;

  /// Input GPIO that is true when there is an outgoing route set through this
  /// block.
  DelayedGpioAccessor block_route_out_;
  /// Input GPIO that is true when there is an outgoing route set through this
  /// block (alternate).
  DelayedGpioAccessor block_route_out2_;

  /// Input GPIO that is true when there is an incoming route set through this
  /// block.
  DelayedGpioAccessor block_route_in_;
  /// Input GPIO that is true when there is an incoming route set through this
  /// block (alternate).
  DelayedGpioAccessor block_route_in2_;

  /// Input GPIO that is true when there is a train occupying the section
  /// belonging to this block.
  DelayedGpioAccessor detector_;

  /// Input GPIO that is true when the user is pressing RbT.
  DelayedGpioAccessor rbt_;

  /// Input GPIO that is true when the user is pressing BlGT.
  DelayedGpioAccessor blgt_;
  
  /// Block accecssor interface.
  I2CBlockInterface* block_;
  /// User-visible name for debug output.
  const char* name_;
  /// Helper for timed operations.
  Timer tm_;
  Timer debug_tm_{Timer::Drifting{1000}};
  
  /// True if the last set "erlaubnis out" on loconet was true.  
  bool shadow_erlaubnis_ : 1;

  bool seen_route_out_ : 1;
  bool seen_route_in_ : 1;
  bool seen_train_ : 1;
  bool need_force_erlaubnis_ : 1;
};

#endif  // _STW_ERBERTBLOCKUI_H_
