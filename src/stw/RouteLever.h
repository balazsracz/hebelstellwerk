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
 * \file RouteLever.h
 *
 * Implements the state machine for a dual-state route lever.
 *
 * @author Balazs Racz
 * @date 20 Mar 2024
 */

#ifndef _STW_ROUTELEVER_H_
#define _STW_ROUTELEVER_H_

#include "stw/Block.h"
#include "stw/LockTable.h"
#include "stw/SignalLever.h"
#include "stw/TurnoutLever.h"
#include "stw/Types.h"
#include "stw/GlobalCommand.h"
#include "utils/Executor.h"
#include "utils/Gpio.h"
#include "utils/Registry.h"
#include "utils/Timer.h"

class RouteLever;

/// Partial template specialization to not have to specify a default turnout to
/// the registry.
template <>
class Instance<RouteLever> {
 public:
  static RouteLever* get() { return nullptr; }
};

class RouteRegistry : public AbstractRegistry<int16_t, RouteLever, RouteLever>,
                      public Singleton<RouteRegistry> {};

class RouteLever : private Executable {
 public:
  RouteLever(RouteId route_up, RouteId route_dn, gpio_pin_t lever_up,
             bool lever_up_inverted, gpio_pin_t lever_down,
             bool lever_down_inverted, gpio_pin_t lock_output, bool lock_invert)
      : up_(route_up, lever_up, lever_up_inverted),
        dn_(route_dn, lever_down, lever_down_inverted),
        lock_(lock_output, lock_invert, GPIO_OUTPUT) {
    Executor::instance()->add(this);
    RouteRegistry::instance()->register_obj(this, route_up);
    RouteRegistry::instance()->register_obj(this, route_dn);
  }

  /// @return true if the lever is locked.
  bool is_locked() { return up_.is_locked() && dn_.is_locked(); }

  /// @return true if the route lever is lifted out of neutral position and set
  /// towards the given route. This does not mean that the route is locked in
  /// that place, but the mechanical locks for turnout levers and lockouts of
  /// other route levers etc should be engaged.
  bool is_route_set(RouteId id) {
    if (up_.is(id)) {
      return up_.is_route_set();
    } else if (dn_.is(id)) {
      return dn_.is_route_set();
    } else {
      DIE("Asked about a route we don't own.");
    }
  }

  void begin() override {
    up_.begin();
    dn_.begin();
    // The period how frequently we check will be different for different route
    // levers. This is to avoid coordinates CPU use between different state
    // machines.
    tm_.start_drifting(CHECK_PERIOD_MSEC + up_.id_);
  }

  void loop() override {
    lock_.write(is_locked() && !global_is_unlocked());
    if (!tm_.check()) {
      return;
    }
    up_.loop();
    dn_.loop();
    /// @todo implement state machine.
    if (!is_locked()) {
#if 0
      bool input_is_proceed = get_lever_is_proceed();
      if (state_ == State::STOP && input_is_proceed) {
        LOG(LEVEL_INFO, "Route %d/Hp%d is proceed", id_, aspect_);
        state_ = State::PROCEED;
      } else if (state_ == State::PROCEED && !input_is_proceed) {
        LOG(LEVEL_INFO, "Route %d/Hp%d is stop+lock", id_, aspect_);
        state_ = State::STOP_LOCKED;
      }
#endif
    }
  }

 private:
  /// Verifies that the preconditions for setting a route are fulfilled. Goes
  /// through the lock table, and checks for Turnouts that they are in the
  /// right direction, Aux that they are set. Ignores signals (nothing to
  /// check) and block as well (because the direction of the block is not
  /// given).
  /// @param row row_up_ or row_dn_.
  /// @return true if the preconditions are okay for setting this route.
  static bool check_preconditions(const LockTable::Row& row) {
    for (const LockTableEntry& e : row) {
      switch (e.type_) {
        case TURNOUT_PLUS: {
          if (TurnoutRegistry::instance()
                  ->get((TurnoutId)e.arg_)
                  ->get_direction() != TurnoutLever::PLUS) {
            return false;
          }
          break;
        }
        case TURNOUT_MINUS:
          if (TurnoutRegistry::instance()
                  ->get((TurnoutId)e.arg_)
                  ->get_direction() != TurnoutLever::MINUS) {
            return false;
          }
          break;
        case SIGNAL_HP1:
        case SIGNAL_HP2:
          // Nothing to check here.
          break;
        case AUX_PLUS:
          /// @todo implement
          break;
        case BLOCK_OUT:
          /// @todo implement
          break;
        case BLOCK_IN:
          // There are no preconditions for Block In.
          break;
        case ROUTE_EXC:
          if (RouteRegistry::instance()
                  ->get((RouteId)e.arg_)
                  ->is_route_set((RouteId)e.arg_)) {
            return false;
          }
          break;
        case ROUTE_ROW: {
          // Checks for route on opposite side of the same lever.
          auto id = (RouteId)e.arg_;
          auto* parent = RouteRegistry::instance()->get(id);
          if (parent->up_.is(id) && parent->dn_.is_route_set()) {
            return false;
          }
          if (parent->dn_.is(id) && parent->up_.is_route_set()) {
            return false;
          }
          break;
        }
        case NONE:
          return true;
      }
    }
    return true;
  }

  /// Locks the preconditions of a route. This means that Turnout and Aux
  /// objects mentioned on the lock table row will get locked.
  static void lock_route_preconditions(const LockTable::Row& row) {
    for (const LockTableEntry& e : row) {
      switch (e.type_) {
        case TURNOUT_MINUS:
        case TURNOUT_PLUS: {
          TurnoutRegistry::instance()->get((TurnoutId)e.arg_)->add_lock();
          break;
        }
        case AUX_PLUS:
          /// @todo implement
          break;
        case SIGNAL_HP1:
        case SIGNAL_HP2:
        case BLOCK_OUT:
        case BLOCK_IN:
        case ROUTE_EXC:
        case ROUTE_ROW:
        case NONE:
          // Nothing to do here.
          break;
      }
    }
  }

  /// Removes locks from the preconditions of a route. This means that Turnout
  /// and Aux objects mentioned on the lock table row will get unlocked (if no
  /// other Route locks them).
  static void unlock_route_preconditions(const LockTable::Row& row) {
    for (const LockTableEntry& e : row) {
      switch (e.type_) {
        case TURNOUT_MINUS:
        case TURNOUT_PLUS: {
          TurnoutRegistry::instance()->get((TurnoutId)e.arg_)->remove_lock();
          break;
        }
        case AUX_PLUS:
          /// @todo implement
          break;
        case SIGNAL_HP1:
        case SIGNAL_HP2:
        case BLOCK_OUT:
        case BLOCK_IN:
        case ROUTE_EXC:
        case ROUTE_ROW:
        case NONE:
          // Nothing to do here.
          break;
      }
    }
  }

  /// @return the signal lever for the given row.
  static SignalLever* find_signal_lever(const LockTable::Row& row) {
    SignalAspect a = HP0;
    SignalId id = LockTable::find_signal(row, &a);
    SignalLever* lever =
        SignalRegistry::instance()->get(signal_registry_idx(id, a));
    return lever;
  }

  static bool check_block(Block* blk, bool out) {
    /// @todo implement once the block has the necessary APIs.
    return true;
  }

  // Verification rules:
  // - there should be a block, and exactly one block
  // - the up and down should have the same preconditions
  // - there should be exactly one signal allowed per Fstr
  //
  // - when different routes are going to the same block, they have to somehow
  //   exclude each other.
  //
  // - when different blocks have the same fahrstrassenfestlegetaste, then they
  //   also have to be somehow excluding each other. When one of these route
  //   levers is set, all others need to be immediately locked by the lock
  //   table.
  //
  // - for every block, when we look at all the rows of the lock table that
  //   have that block as input, the signal name has to be the same across
  //   them. It may be different aspect (hp1/hp2).

  /// How often do we run the state machine.
  static constexpr uint32_t CHECK_PERIOD_MSEC = 10;

  enum class State : uint8_t {
    /// Lever is in the middle state. No routes are set. Lever is locked
    /// because preconditions for settings the routes are not met.
    NEUTRAL_LOCKED,
    /// Lever is in the middle state. No routes are set. The preconditions for
    /// setting a route are fulfilled. The lever is unlocked and can be moved.
    NEUTRAL,
    /// The lever is up/dn, but the route is not locked yet.
    SET,
    /// The lever is up/dn and the route is locked in.
    SET_LOCKED,
  };

  struct Route;
  friend struct Route;

  struct Route {
    Route(RouteId route, gpio_pin_t lever, bool lever_inverted)
        : input_(lever, lever_inverted, GPIO_INPUT),
          id_(route),
          seen_proceed_(false),
          seen_train_(false),
          error_unexpected_set_(false) {}

    bool is(RouteId id) { return id == id_; }

    bool is_route_set() {
      return state_ == State::SET || state_ == State::SET_LOCKED;
    }

    bool is_locked() {
      switch (state_) {
        case State::NEUTRAL_LOCKED:
        case State::SET_LOCKED:
          return true;
        default:
          return false;
      }
    }

    /// This function is called once at the beginning by the parent.
    void begin() {
      row_ = LockTable::instance()->find_route(id_);
      auto block_id = LockTable::find_block(row_, &block_out_);
      if (block_id == NO_BLOCK) {
        block_ = nullptr;
      } else {
        block_ = BlockRegistry::instance()->get(block_id);
      }
      if (input_.read()) {
        state_ = State::SET;
      } else {
        state_ = State::NEUTRAL;
      }
    }

    /// This function is called every 10 msec by the parent.
    void loop() {
      auto* signal_lever = find_signal_lever(row_);
      switch (state_) {
        case State::NEUTRAL_LOCKED: {
          if (check_preconditions(row_)) {
            error_unexpected_set_ = false;
            state_ = State::NEUTRAL;
            LOG(LEVEL_INFO, "Fstr %d unlocked", id_);
          } else {
            if (input_.read()) {
              if (!error_unexpected_set_) {
                error_unexpected_set_ = true;
                LOG(LEVEL_ERROR, "Fstr %d unexpected set", id_);
              }
            } else {
              error_unexpected_set_ = false;
            }
          }
          break;
        }
        case State::NEUTRAL: {
          if (input_.read()) {
            LOG(LEVEL_INFO, "Fstr %d lever set", id_);
            state_ = State::SET;
            lock_route_preconditions(row_);
          } else if (!check_preconditions(row_)) {
            state_ = State::NEUTRAL_LOCKED;
            LOG(LEVEL_INFO, "Fstr %d locked", id_);
          }
          break;
        }
        case State::SET: {
          if (!input_.read()) {
            LOG(LEVEL_INFO, "Fstr %d lever removed", id_);
            state_ = State::NEUTRAL;
            unlock_route_preconditions(row_);
          } else if (block_ && block_->route_lock_button().read() &&
                     check_block(block_, block_out_)) {
            state_ = State::SET_LOCKED;
            LOG(LEVEL_INFO, "Fstr %d locking route", id_);
            block_->notify_route_locked(id_, block_out_);
            seen_proceed_ = false;
            seen_train_ = false;
            signal_lever->unlock();
          }
          break;
        }
        case State::SET_LOCKED: {
          // Checks for signal lever on proceed.
          if (!seen_proceed_ && signal_lever->is_proceed()) {
            LOG(LEVEL_INFO, "Fstr %d seen proceed", id_);
            seen_proceed_ = true;
          }
          // Checks for train seen on detector afterwards.
          if (seen_proceed_ && !seen_train_ &&
              block_->track_detector().read()) {
            LOG(LEVEL_INFO, "Fstr %d seen train", id_);
            seen_train_ = true;
          }
          // Checks for signal lever re-set. Depending on inbounds or outbounds
          // direction, we may have to keep the lever locked or unlock it again.
          if (seen_proceed_ && !seen_train_ && !block_out_ &&
              !signal_lever->is_proceed() && signal_lever->is_locked()) {
            // Note: we don't clear seen_proceed here because it is impossible
            // to tell whether the signal was reset before a train passed it or
            // after.
            //
            // seen_proceed_ = false;
            signal_lever->unlock();
          }
          if (seen_proceed_ && seen_train_ &&
              !signal_lever->is_proceed() &&
              !block_->track_detector().read()) {
            LOG(LEVEL_INFO, "Fstr %d complete&unlock", id_);
            // All conditions have been fulfilled to release the route:
            //
            // - we've seen the signal lever go to proceed
            // - we've seen a train in the detector
            // - the detector is now clear
            // - the signal lever is taken back
            signal_lever->lock();
            block_->notify_route_complete(id_);
            state_ = State::SET;  // will unlock the route lever
          }
          break;
        }
      }
    }

    /// Gpio. HIGH means the lever is up (if not inverted).
    GpioAccessor input_;

    /// Lock table row for the route.
    LockTable::Row row_;

    /// Pointer to the block object for route id_.
    Block* block_;

    /// Number of the route for upwards.
    RouteId id_;

    /// true if block_ is outbounds in the lock table.
    bool block_out_;

    /// True if the signal was seen as set to proceed.
    bool seen_proceed_ : 1;
    /// True if the detector was seen as occupied.
    bool seen_train_ : 1;

    /// Prevents repeating unexpected set error.
    bool error_unexpected_set_ : 1;
    
    /// Internal route state.
    State state_;
  } up_, dn_;

  /// Gpio for the lock. HIGH means the lever is locked.
  GpioAccessor lock_;

  /// Used to periodically check the preconditions.
  Timer tm_;
};

#endif  // _STW_ROUTELEVER_H_
