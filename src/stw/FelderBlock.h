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
 * \file FelderBlock.h
 *
 * Class implementing a Block that has the Felderblock user interface elements.
 *
 * @author Balazs Racz
 * @date 6 Apr 2024
 */

#ifndef _STW_FELDERBLOCK_H_
#define _STW_FELDERBLOCK_H_

#include "stw/Block.h"
#include "stw/GlobalCommand.h"
#include "stw/I2CBlock.h"
#include "stw/RouteLever.h"
#include "utils/Timer.h"

class FelderBlock : public Block {
 public:
  FelderBlock(I2CBlockInterface* iface, BlockId id,
              gpio_pin_t track_detector_pin, bool track_detector_inverted,
              gpio_pin_t route_lock_button_pin, bool route_lock_button_inverted,
              gpio_pin_t route_locked_lamp_pin, bool route_locked_lamp_inverted,
              gpio_pin_t route_unlocked_btn_pin, bool route_unlock_btn_inverted)
      : Block(id, track_detector_pin, track_detector_inverted,
              route_lock_button_pin, route_lock_button_inverted,
              route_locked_lamp_pin, route_locked_lamp_inverted,
              route_unlocked_btn_pin, route_unlock_btn_inverted),
        iface_(iface),
        have_route_locked_(false),
        route_is_out_(false),
        seen_route_locked_out_(false),
        seen_route_locked_in_(false),
        kurbel_last_(false),
        startup_err_(false),
        defer_operations_(0) {}

  // Setup functions. Allows defining the GPIO mapping for this given block in
  // a way that is a bit more readable, than just having a super long list of
  // numbers in the constructor. The return values should be combined with |,
  // and at the end the resulting bitmask should equal the constant
  // EXPECTED_SETUP.

  uint16_t set_vorblock_taste(gpio_pin_t pin, bool inverted) {
    vorblock_taste_.setup(pin, inverted, GPIO_INPUT);
    return 1u << 0;
  }
  uint16_t set_ruckblock_taste(gpio_pin_t pin, bool inverted) {
    ruckblock_taste_.setup(pin, inverted, GPIO_INPUT);
    return 1u << 1;
  }
  uint16_t set_abgabe_taste(gpio_pin_t pin, bool inverted) {
    abgabe_taste_.setup(pin, inverted, GPIO_INPUT);
    return 1u << 2;
  }
  uint16_t set_kurbel(gpio_pin_t pin, bool inverted) {
    kurbel_.setup(pin, inverted, GPIO_INPUT);
    return 1u << 3;
  }

  uint16_t set_anfangsfeld(gpio_pin_t pin, bool inverted) {
    anfangsfeld_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u << 4;
  }
  uint16_t set_endfeld(gpio_pin_t pin, bool inverted) {
    endfeld_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u << 5;
  }
  uint16_t set_erlaubnisfeld(gpio_pin_t pin, bool inverted) {
    erlaubnisfeld_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u << 6;
  }

  uint16_t set_signalhaltmelder(gpio_pin_t pin, bool inverted) {
    signalhaltmelder_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u << 7;
  }
  uint16_t set_storungsmelder(gpio_pin_t pin, bool inverted) {
    storungsmelder_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u << 8;
  }
  uint16_t set_streckentastensperre(gpio_pin_t pin, bool inverted) {
    streckentastensperre_.setup(pin, inverted, GPIO_OUTPUT);
    return 1u << 9;
  }

  /// These bits should be set after all the setup is done.
  static constexpr uint16_t EXPECTED_SETUP = (1u << 10) - 1;

  void check_setup(uint16_t setup_value) {
    if (EXPECTED_SETUP != setup_value) {
      LOG(LEVEL_ERROR,
          "Missing setup GPIO. Expected %03x actual %03x missing %03x",
          EXPECTED_SETUP, setup_value, EXPECTED_SETUP & ~setup_value);
      DIE("Missing GPIO setup for block.");
    }
  }

  // ==========================================

  bool allow_outgoing_train() override {
    return (state_ == State::OUT_FREE) || startup_err_;
  }

  void notify_route_locked(RouteId id, bool is_out) override
  {
    Block::notify_route_locked(id, is_out);
    if (have_route_locked_) {
      LOG(LEVEL_INFO, "WARN Block %d: two routes locked (old %d, new %d)", id_,
          locked_route_, id);
    }
    have_route_locked_ = true;
    locked_route_ = id;
    route_is_out_ = is_out;
    if (is_out) {
      seen_route_locked_out_ = true;
    } else {
      seen_route_locked_in_ = true;
    }
  }

  void notify_route_cancelled(RouteId id) override {
    Block::notify_route_cancelled(id);
    if (id == locked_route_) {
      have_route_locked_ = false;
      seen_route_locked_in_ = false;
      seen_route_locked_out_ = false;
    }
  }
  
  void notify_route_complete(RouteId id) override {
    Block::notify_route_complete(id);
    if (id == locked_route_) {
      have_route_locked_ = false;
    } else {
      LOG(LEVEL_ERROR,
          "ERR Block %d: unlock (%d) for different route than locked(%d)", id_,
          id, locked_route_);
    }
  }
  
  /// @return The current internal state (enum State) as int for debugging.
  uint8_t state() {
    return (uint8_t)state_;
  }

  enum class State : uint8_t {
    /// No permission, track is free.
    IN_FREE,
    /// No permission, track is occupied.
    IN_OCC,
    /// Have permission, track is free.
    OUT_FREE,
    /// Have permission, track is occupied.
    OUT_OCC,
    /// We don't know the state.
    STARTUP,
  };

 private:
  void begin() override {
    Block::begin();
    tm_.start_drifting(POLL_MSEC);
    // Searches for entry signal levers for this block.
    SignalLever* lever = nullptr;
    bool ours = false;
    for (const LockTableEntry& e : *LockTable::instance()) {
      if (e.type_ == ROUTE_ROW) {
        register_signal_lever(ours, lever);
        lever = nullptr;
        ours = false;
      } else if (e.type_ == SIGNAL_HP1) {
        auto id = signal_registry_idx(SignalId(e.arg_), HP1);
        lever = SignalRegistry::instance()->get(id);
      } else if (e.type_ == SIGNAL_HP2) {
        auto id = signal_registry_idx(SignalId(e.arg_), HP2);
        lever = SignalRegistry::instance()->get(id);
      } else if (e == BlockIn(id_)) {
        // This route uses our block inbound.
        ours = true;
      }
    }
    // Last row.
    register_signal_lever(ours, lever);
  }

  void register_signal_lever(bool ours, SignalLever* lever) {
    if (!ours) return;
    if (!lever) {
      LOG(LEVER_ERROR, "Found route for block %d, but no signal lever.", id_);
      return;
    }
    auto* p = new SignalLeverPtr;
    p->lever = lever;
    p->next = signals_in_;
    signals_in_ = p;
  }

  void loop() override {
    Block::loop();

#if 0
    // This code can be used to test the hardware connection to the colored
    // fields.
    anfangsfeld_.write(vorblock_taste_.read());
    endfeld_.write(ruckblock_taste_.read());
    erlaubnisfeld_.write(abgabe_taste_.read());
    storungsmelder_.write(iface_->get_status() & BlockBits::ERROR);
    return;
#endif 
    
    // Sets the output GPIOs.
    switch (state_) {
      case State::IN_FREE: {
        /// @todo is the Anfangsfeld red when the Erlaubnisfeld is red?
        anfangsfeld_.write(WHITE);
        endfeld_.write(WHITE);
        erlaubnisfeld_.write(RED);
        break;
      }
      case State::IN_OCC: {
        /// @todo is the Anfangsfeld red when the Erlaubnisfeld is red?
        anfangsfeld_.write(WHITE);
        endfeld_.write(RED);
        erlaubnisfeld_.write(RED);
        break;
      }
      case State::OUT_FREE: {
        anfangsfeld_.write(WHITE);
        endfeld_.write(WHITE);
        erlaubnisfeld_.write(WHITE);
        break;
      }
      case State::OUT_OCC: {
        anfangsfeld_.write(RED);
        endfeld_.write(WHITE);
        erlaubnisfeld_.write(WHITE);
        break;
      }
      case State::STARTUP: {
        anfangsfeld_.write(RED);
        endfeld_.write(RED);
        erlaubnisfeld_.write(RED);
        break;
      }
    }

    // The rest of the logic we only do occasionally.
    if (!tm_.check()) {
      return;
    }

    if (defer_operations_) {
      --defer_operations_;
      return;
    }

    // ==== Handle Crank ====
    bool kurbel = kurbel_.read();
    // Performs positive edge detection for kurbel. After this section, the
    // kurbel variable will be true for only one round after it went positive.
    if (kurbel) {
      if (kurbel_last_) {
        kurbel = false;
      } else {
        kurbel_last_ = true;
      }
    } else {
      kurbel_last_ = false;
    }
    
    
    uint16_t status = iface_->get_status();

    // ==== Handle Störungsmelder ====
    if (status & BlockBits::ERROR) {
        storungsmelder_.write(true);
    } else {
        storungsmelder_.write(false);
        startup_err_ = false;
    }

    // ==== Handle Signalhaltmelder ====
    bool halt = true;
    for (const auto* p = signals_in_; p; p = p->next) {
      if (p->lever->is_proceed()) halt = false;
    }
    signalhaltmelder_.write(halt);

    // ==== Handle elektrische Streckentastensperre
    streckentastensperre_.write(
        state_ == State::IN_OCC && have_route_locked_ &&
        locked_route()->streckentastensperre(locked_route_));

    // ==== Handle Startup of the Arduino ====
    if (state_ == State::STARTUP && status != 0) {
      bool has_out = status & BlockBits::TRACK_OUT;
      bool has_in = status & BlockBits::HANDOFF;
      bool has_start = status & BlockBits::STARTUP;
      bool has_err = status & BlockBits::ERROR;
      if (has_out && has_in) {
        // Confusing.
        LOG(LEVEL_ERROR,
            "ERR Block %d startup: both out and in is set in status (%02x), "
            "resetting to out+free",
            id_, status);
        cold_start();
      } else if (has_out) {
        state_ =
            status & BlockBits::OUT_BUSY ? State::OUT_OCC : State::OUT_FREE;
        LOG(LEVEL_INFO, "Block %d: start with block state OUT (%02x) - %d", id_,
            status, (int)state_);
      } else if (has_in) {
        state_ =
            status & BlockBits::IN_BUSY ? State::IN_OCC : State::IN_FREE;
        LOG(LEVEL_INFO, "Block %d: start with block state IN (%02x) - %d", id_,
            status, (int)state_);
      } else if (has_err) {
        // We have an error, maybe the block line is not connected.
        if (status & BlockBits::NEWINPUT) {
          iface_->clear_incoming_notify();
        }
        startup_err_ = true;
        // We don't leave the startup state. This will show three red fields.
      } else if (has_start) {
        // The block doesn't know the state and we don't know it either. This
        // is a cold start. We set up the state for outgoing track free. In
        // this state the operator can do a handoff (Erlaubnisabgabe).
        LOG(LEVEL_INFO, "Block %d: cold start(%02x) - setting out+free", id_,
            status);
        cold_start();
        return;
      } else {
        LOG(LEVEL_INFO, "Block %d (state %d): unknown start(%02x) - setting out+free", id_, (int)state_, status);
        cold_start();
        return;
      }
    }

    // ==== Handle Startup of the Block I2C interface ====
    if ((status & (BlockBits::STARTUP | BlockBits::TRACK_OUT |
                   BlockBits::HANDOFF)) == (uint16_t)BlockBits::STARTUP) {
      // Now we have a state but the block interface has reset and it doesn't
      // have a state.
      switch(state_) {
        case State::IN_FREE:
          iface_->reset_in();
          return;
        case State::IN_OCC:
          iface_->reset_endfeld_rot();
          return;
        case State::OUT_FREE:
          iface_->reset_out();
          return;
        case State::OUT_OCC:
          iface_->reset_anf_rot();
          return;
        case State::STARTUP:
          break;
      }
    }

    // Abgabe / handoff
    if (abgabe_taste_.read() && kurbel_.read()) {
      // We purposefully are not using the edge-detected `kurbel` variable
      // above. There is a bug in the I2C block that sometimes makes the block
      // state flip back to "ErlaubWir" the next tick after we performed
      // handoff ("ErlaubAnd"). We can fix this by performing the handoff a
      // second time. So long as the user is still holding the button and
      // turning the crank, we will re-enter the condition here.

      const char* from = nullptr;
      if (state_ == State::OUT_FREE && !have_route_locked_) {
        // Now handoff is possible.
        from = "OUT_FREE";
      } else if (global_is_unlocked() && kurbel) {
        // For global unlocked, we do use the edge-detected kurbel condition.
        from = "forced";
      }
      if (from) {
        seen_route_locked_out_ = false;
        seen_route_locked_in_ = false;
        iface_->abgabe();
        LOG(LEVEL_INFO, "Block %d: %02x %s->IN_FREE (Erlaubnis gesendet)", id_,
            status, from);
        return wait_for_complete(State::IN_FREE);
      }
    }

    // Vorblocken / Out-busy
    if (vorblock_taste_.read() && kurbel) {
      const char* from = global_is_unlocked() ? "forced" : nullptr;
      // Check if a train has traveled outbounds through this block, and the
      // route lever matching that has been re-set.
      /// @todo does the route lever really have to be re-set here?
      if (state_ == State::OUT_FREE && seen_route_locked_out_ &&
          !have_route_locked_) {
        from = "OUT_FREE";
      }
      if (from) {
        seen_route_locked_out_ = false;
        iface_->vorblocken();
        LOG(LEVEL_INFO, "Block %d: %02x %s->OUT_OCC (Vorblocken gesendet)", id_,
            status, from);
        return wait_for_complete(State::OUT_OCC);
      }
    }

    // Ruckblocken / In-free
    if (ruckblock_taste_.read() && kurbel) {
      const char* from = nullptr;
      // Check if a train has traveled inbounds through this block, and the
      // route lever matching that has been unlocked (route traveled).
      if (state_ == State::IN_OCC && seen_route_locked_in_ &&
          !have_route_locked_) {
        from = "IN_OCC";
      } else if (global_is_unlocked()) {
        // In order to send forced "Rückblocken" we have to ensure first that
        // the I2C Block believes that "EndeRot" is set.
        if (status & BlockBits::IN_BUSY) {
          // We can send back the signal.
          from = "forced";
        } else {
          // We have to reset the block status first.
          iface_->reset_endfeld_rot();
          state_ = State::IN_OCC;
          // We need to wait a bit to ensure that the block is settled.
          defer_operations_ = 63;
          // This will make us go into this branch again.
          kurbel_last_ = false;
        }
      }
      if (from) {
        seen_route_locked_in_ = false;
        iface_->ruckblocken();
        LOG(LEVEL_INFO, "Block %d: %02x, %s->IN_FREE (Ruckblocken gesendet)",
            id_, status, from);
        return wait_for_complete(State::IN_FREE);
      }
    }

    switch (state_) {
      case State::IN_FREE: {
        if (iface_->has_incoming_notify()) {
          if (status & BlockBits::IN_BUSY) {
            iface_->clear_incoming_notify();
            LOG(LEVEL_INFO,
                "Block %d: %02x IN_FREE->IN_OCC (Vorblocken erhalten)", id_,
                status);
            state_ = State::IN_OCC;
            return;
          }
          if (status & BlockBits::TRACK_OUT) {
            iface_->clear_incoming_notify();
            LOG(LEVEL_INFO,
                "Block %d: %02x IN_FREE->OUT_FREE (Erlaubnis erhalten)", id_,
                status);
            state_ = State::OUT_FREE;
            return;
          }
        }
        break;
      }
      case State::IN_OCC: {
        break;
      }
      case State::OUT_FREE: {
        break;
      }
      case State::OUT_OCC: {
        if (iface_->has_incoming_notify() &&
            (status & BlockBits::OUT_BUSY) == 0) {
          iface_->clear_incoming_notify();
          LOG(LEVEL_INFO,
              "Block %d: %02x OUT_OCC->OUT_FREE (Ruckblocken erhalten)", id_,
              status);
          state_ = State::OUT_FREE;
          return;
        }
        break;
      }
      case State::STARTUP: {
        break;
      }
    }
  }

  void wait_for_complete(State s) {
    /// @todo implement some way to delay until a block operation completes.
    state_ = s;
  }

  /// Setup both the block and the local state for a cold start.
  void cold_start() {
    state_ = State::OUT_FREE;
    auto nst = BlockBits::TRACK_OUT | BlockBits::NEWOUTPUT;
    iface_->set_status((uint16_t)nst);
  }

  /// Call this only when have_route_locked == true.
  /// @return the route lever that is locked in.
  RouteLever* locked_route() {
    return RouteRegistry::instance()->get(locked_route_);
  }
  
  struct SignalLeverPtr {
    SignalLever* lever;
    SignalLeverPtr* next;
  };

  /// How often we should evaluate the internal state machine.
  static constexpr uint16_t POLL_MSEC = 13;
  
  /// Hardware connection for the actual block PCB.
  I2CBlockInterface* iface_{nullptr};

  /// Signal levers that are used for the inbounds signal on this block.
  SignalLeverPtr* signals_in_{nullptr};

  /// Button to signal an outgoing train to the next station, i.e., set the
  /// track to busy with an outgoing train. True when the user presses the
  /// button.
  DelayedGpioAccessor vorblock_taste_;
  /// Button to signal that an incoming train from the next station has
  /// arrived, i.e., to set a busy incoming track to clear.  True when the user
  /// presses the button.
  DelayedGpioAccessor ruckblock_taste_;
  /// Button to hand off the permission to the next station, i.e., set an
  /// outgoing track to an incoming track.  True when the user presses the
  /// button.
  DelayedGpioAccessor abgabe_taste_;

  /// Input for the crank. For an analog crank, this should be a virtual Gpio
  /// using the AnalogGpio class.
  DelayedGpioAccessor kurbel_;

  static constexpr bool RED = false;
  static constexpr bool WHITE = true;

  /// Output for a red/white field showing that an outgoing track is
  /// clear. Value is 1 for white, 0 for red.
  DelayedGpioAccessor anfangsfeld_;
  /// Output for a red/white field showing that an incoming track is
  /// clear. Value is 1 for white, 0 for red.
  DelayedGpioAccessor endfeld_;
  /// Output for a red/white field showing that we have the permission to send
  /// trains to the track. Value is 1 for white, 0 for red.
  DelayedGpioAccessor erlaubnisfeld_;

  /// Output for a white field showing that the track button is locked. This is
  /// controlled by the detector. 1 for lit, 0 for dark.
  DelayedGpioAccessor streckentastensperre_;
  /// Output for a red field showing that the incoming signal is showing the
  /// stop aspect. 1 for red, 0 for dark.
  DelayedGpioAccessor signalhaltmelder_;
  /// Output for a field showing that there is an error. Value is 1 for error,
  /// 0 for no error.
  DelayedGpioAccessor storungsmelder_;

  Timer tm_;

  /// Current state of the block.
  State state_{State::STARTUP};

  /// @todo these need to be reset to zero when appropriate

  /// ID of the route that was last locked.
  RouteId locked_route_{(RouteId)0};
  /// True if a route is locked through this block.
  bool have_route_locked_ : 1;
  /// True if the last locked route was outbounds.
  bool route_is_out_ : 1;
  /// True if we have seen a route locked outbound through this block.
  bool seen_route_locked_out_ : 1;
  /// True if we have seen a route locked inbound through this block.
  bool seen_route_locked_in_ : 1;

  /// Records the state of the kurbel from the last iteration for edge
  /// detection. We only apply kurbel when there is a false to true edge on it.
  bool kurbel_last_ : 1;

  /// True if we've seen error at startup from the block and never exited this
  /// error. We just never use the block and allow all movements.
  bool startup_err_ : 1;

  /// If non-zero, we defer block operations for this many ticks.
  uint8_t defer_operations_ : 6;
};

#endif  // _STW_FELDERBLOCK_H_
