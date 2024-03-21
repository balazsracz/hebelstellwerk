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

#include "stw/Types.h"
#include "stw/LockTable.h"

class RouteLever;

/// Partial template specialization to not have to specify a default turnout to
/// the registry.
template<> class Instance<RouteLever> {
 public:
  static RouteLever* get() { return nullptr; }
};


class RouteRegistry
    : public AbstractRegistry<int16_t, RouteLever, RouteLever>,
      public Singleton<RouteRegistry> {};

class RouteLever : private Executable {
 public:
  RouteLever(RouteId route_up, RouteId route_dn, gpio_pin_t lever_up,
             bool lever_up_inverted, gpio_pin_t lever_down,
             bool lever_down_inverted, gpio_pin_t lock_output, bool lock_invert)
      : id_(route),
        aspect_(aspect),
        input_invert_(lever_invert),
        output_invert_(lock_invert),
        lever_input_(lever_input),
        lock_output_(lock_output) {
    input_ = GpioRegistry::instance()->get(lever_input_);
    lock_ = GpioRegistry::instance()->get(lock_output_);
    Executor::instance()->add(this);
    auto idx = route_registry_idx(route, aspect);
    RouteRegistry::instance()->register_obj(this, idx, idx+1);
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
    switch(state_) {
      case State::NEUTRAL_LOCKED:
      case State::UP_LOCKED:
      case State::DN_LOCKED:
        return true;
      default:
        return false;
    }
  }

  void begin() override {
    if (input_up_.read()) {
      state_ = State::UP;
    } else if (input_dn_.read()) {
      state_ = State::DN;
    } else {
      state_ = State::NEUTRAL_LOCKED;
    }
  }
  
  void loop() override {
    lock_->write(is_locked());
    /// @todo implement state machine.
    if (!is_locked()) {
      bool input_is_proceed = get_lever_is_proceed();
      if (state_ == State::STOP && input_is_proceed) {
        LOG(LEVEL_INFO, "Route %d/Hp%d is proceed", id_, aspect_);
        state_ = State::PROCEED;
      } else if (state_ == State::PROCEED && !input_is_proceed) {
        LOG(LEVEL_INFO, "Route %d/Hp%d is stop+lock", id_, aspect_);
        state_ = State::STOP_LOCKED;
      }
    }
  }

 private:
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

  /// First entry in the lock table for the upwards route. This is after the
  /// ROUTE declaration.
  const LockTableEntry* locktable_up_;
  /// Number of entries in the lock table row for the upwards route.
  uint16_t locktable_up_size_;
  /// First entry in the lock table for the dnwards route. This is after the
  /// ROUTE declaration.
  const LockTableEntry* locktable_dn_;
  /// Number of entries in the lock table row for the dnwards route.
  uint16_t locktable_dn_size_;
  
  /// Internal route state.
  State state_;
};


#endif // _STW_ROUTELEVER_H_
