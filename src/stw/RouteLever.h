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
      : id_up_(route_up),
        id_dn_(route_dn),
        input_up_(lever_up, lever_up_inverted, GPIO_INPUT),
        input_dn_(lever_down, lever_down_inverted, GPIO_INPUT),
        lock_(lock_output, lock_invert, GPIO_OUTPUT) {
    Executor::instance()->add(this);
    RouteRegistry::instance()->register_obj(this, route_up);
    RouteRegistry::instance()->register_obj(this, route_dn);
  }

  enum class State {
    /// Lever is in the middle state. No routes are set. Lever is locked
    /// because preconditions for settings the routes are not met.
    NEUTRAL_LOCKED,
    /// Lever is in the middle state. No routes are set. The preconditions for
    /// setting a route are fulfilled. The lever is unlocked and can be moved.
    NEUTRAL,
    /// The lever is up, but the route is not locked yet.
    UP,
    /// The lever is up and the route is locked in.
    UP_LOCKED,
    /// The route is set and locked, the Signal was set to proceed and returned
    /// to Stop (i.e., the route has been traveled by a train).
    UP_REPTLOCK,
    /// The lever is dn, but the route is not locked yet.
    DN,
    /// The lever is dn and the route is locked in.
    DN_LOCKED,
    /// The route is set and locked, the Signal was set to proceed and returned
    /// to Stop (i.e., the route has been traveled by a train).
    DN_REPTLOCK,
  };

  /// @return true if the lever is locked.
  bool is_locked() {
    switch (state_) {
      case State::NEUTRAL_LOCKED:
      case State::UP_LOCKED:
      case State::DN_LOCKED:
        return true;
      default:
        return false;
    }
  }

  /// @return true if the route lever is lifted out of neutral position and set
  /// towards the given route. This does not mean that the route is locked in
  /// that place, but the mechanical locks for turnout levers and lockouts of
  /// other route levers etc should be engaged.
  bool is_route_set(RouteId id) {
    if (id == id_up_) {
      return state_ == State::UP || state_ == State::UP_LOCKED ||
             state_ == State::UP_REPTLOCK;
    } else if (id == id_dn_) {
      return state_ == State::DN || state_ == State::DN_LOCKED ||
             state_ == State::DN_REPTLOCK;
    } else {
      DIE("Asked about a route we don't own.");
    }
  }

  void begin() override {
    row_up_ = LockTable::instance()->find_route(id_up_);
    row_dn_ = LockTable::instance()->find_route(id_dn_);
    auto block_up_id = LockTable::find_block(row_up_, &block_up_out_);
    if (block_up_id == NO_BLOCK) {
      block_up_ = nullptr;
    } else {
      block_up_ = BlockRegistry::instance()->get(block_up_id);
    }
    auto block_dn_id = LockTable::find_block(row_dn_, &block_dn_out_);
    if (block_dn_id == NO_BLOCK) {
      block_dn_ = nullptr;
    } else {
      block_dn_ = BlockRegistry::instance()->get(block_dn_id);
    }

    
    if (input_up_.read()) {
      state_ = State::UP;
    } else if (input_dn_.read()) {
      state_ = State::DN;
    } else {
      state_ = State::NEUTRAL_LOCKED;
    }
    // The period how frequently we check will be different for different route
    // levers. This is to avoid coordinates CPU use between different state
    // machines.
    tm_.start_drifting(CHECK_PERIOD_MSEC + id_up_);
  }

  void loop() override {
    lock_.write(is_locked());
    if (!tm_.check()) {
      return;
    }
    switch (state_) {
      case State::NEUTRAL_LOCKED: {
        if (check_preconditions(row_up_) && check_preconditions(row_dn_)) {
          state_ = State::NEUTRAL;
          LOG(LEVEL_INFO, "Fstr %d/%d unlocked", id_up_, id_dn_);
        } else {
          if (input_up_.read()) {
            LOG(LEVEL_ERROR, "Fstr %d unexpected up", id_up_);
          }
          if (input_dn_.read()) {
            LOG(LEVEL_ERROR, "Fstr %d unexpected dn", id_up_);
          }
        }
        break;
      }
      case State::NEUTRAL: {
        if (input_up_.read()) {
          LOG(LEVEL_INFO, "Fstr %d lever up set", id_up_);
          state_ = State::UP;
          lock_route_preconditions(row_up_);
        } else if (input_dn_.read()) {
          LOG(LEVEL_INFO, "Fstr %d lever dn set", id_dn_);
          state_ = State::DN;
          lock_route_preconditions(row_dn_);
        }
        break;
      }
      case State::UP: {
        if (!input_up_.read()) {
          LOG(LEVEL_INFO, "Fstr %d lever up removed", id_up_);
          state_ = State::NEUTRAL;
          unlock_route_preconditions(row_up_);
        } else if (block_up_ && block_up_->route_lock_button().read() &&
                   check_block(block_up_, block_up_out_)) {
          state_ = State::UP_LOCKED;
          unlock_signal(row_up_);
        }
        break;
      }
      case State::UP_LOCKED:
        break;
      case State::UP_REPTLOCK:
        break;
      case State::DN: {
        if (!input_dn_.read()) {
          LOG(LEVEL_INFO, "Fstr %d lever dn removed", id_dn_);
          state_ = State::NEUTRAL;
          unlock_route_preconditions(row_dn_);
        }
        if (block_dn_ && block_dn_->route_lock_button().read() &&
            check_block(block_dn_, block_dn_out_)) {
          state_ = State::DN_LOCKED;
          unlock_signal(row_dn_);
        }
        break;
      }
      case State::DN_LOCKED:
        break;
      case State::DN_REPTLOCK:
        /// @todo implement
        break;
    }
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
  bool check_preconditions(const LockTable::Row& row) {
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
          /// @todo implement
          break;
        case ROUTE_ROW:
        case NONE:
          return true;
      }
    }
    return true;
  }

  /// Locks the preconditions of a route. This means that Turnout and Aux
  /// objects mentioned on the lock table row will get locked.
  void lock_route_preconditions(const LockTable::Row& row) {
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
  void unlock_route_preconditions(const LockTable::Row& row) {
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
        case ROUTE_ROW:
        case NONE:
          // Nothing to do here.
          break;
      }
    }
  }

  /// Unlocks the signal lever matching the route. Dies if there is no signal
  /// lever in the route.
  void unlock_signal(const LockTable::Row& row) {
    SignalAspect a = HP0;
    SignalId id = LockTable::find_signal(row, &a);
    SignalLever* lever =
        SignalRegistry::instance()->get(signal_registry_idx(id, a));
    lever->unlock();
  }

  bool check_block(Block* blk, bool out) {
    /// @todo implement once the block has the necessary APIs.
    return true;
  }
  
  
  // Verification rules:
  // - there should be a block, and exactly one block
  // - the up and down should have the same preconditions
  // - there should be exactly one signal allowed per Fstr
  //

  static constexpr uint32_t CHECK_PERIOD_MSEC = 10;

  /// Number of the route for upwards.
  RouteId id_up_;
  /// Number of the route for downwards.
  RouteId id_dn_;

  /// Gpio for upwards. HIGH means the lever is up (if not inverted).
  GpioAccessor input_up_;
  /// Gpio for downwards. HIGH means the lever is down (if not inverted).
  GpioAccessor input_dn_;

  /// Gpio for the lock. HIGH means the lever is locked.
  GpioAccessor lock_;

  /// Lock table row for the upwards route.
  LockTable::Row row_up_;
  /// Lock table row for the dnwards route.
  LockTable::Row row_dn_;

  /// Pointer to the block object for route_up_.
  Block* block_up_;
  /// Pointer to the block object for route_dn_.
  Block* block_dn_;
  /// true if block_up is outbounds in the lock table.
  bool block_up_out_;
  /// true if block_dn is outbounds in the lock table.
  bool block_dn_out_;
  
  /// Internal route state.
  State state_;

  /// Used to periodically check the preconditions.
  Timer tm_;
};

#endif  // _STW_ROUTELEVER_H_
