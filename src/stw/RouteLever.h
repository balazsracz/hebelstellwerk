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
    RouteRegistry::instance()->register_obj(this, route_up,
                                            RouteId(route_up + 1));
    RouteRegistry::instance()->register_obj(this, route_dn,
                                            RouteId(route_dn + 1));
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
    switch(state_) {
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
          LOG(LEVEL_ERROR, "Fstr %d lever up set", id_up_);
          state_ = State::UP;
        } else if (input_dn_.read()) {
          LOG(LEVEL_ERROR, "Fstr %d lever dn set", id_dn_);
          state_ = State::DN;
        }
        break;
      }
      case State::UP:
      case State::UP_LOCKED:
      case State::UP_REPTLOCK:
      case State::DN:
      case State::DN_LOCKED:
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

  /// Internal route state.
  State state_;

  /// Used to periodically check the preconditions.
  Timer tm_;
};

#endif  // _STW_ROUTELEVER_H_