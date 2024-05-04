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
 * \file Block.h
 *
 * Class for representing the state and I/Os related to a block, meaning a
 * particular mainline leaving a station in a particular direction.
 *
 * @author Balazs Racz
 * @date 23 Mar 2024
 */

#ifndef _STW_BLOCK_H_
#define _STW_BLOCK_H_

#include "stw/Types.h"
#include "utils/Executor.h"
#include "utils/Gpio.h"
#include "utils/Registry.h"
#include "utils/Logging.h"

class Block;

/// Partial template specialization to not have to specify a default block to
/// the registry.
template <>
class Instance<Block> {
 public:
  static Block* get() { return nullptr; }
};

class BlockRegistry : public AbstractRegistry<BlockId, Block, Block>,
                      public Singleton<BlockRegistry> {};

class Block : public Executable {
 public:
  Block(BlockId id, gpio_pin_t track_detector_pin, bool track_detector_inverted,
        gpio_pin_t route_lock_button_pin, bool route_lock_button_inverted,
        gpio_pin_t route_locked_lamp_pin, bool route_locked_lamp_inverted,
        gpio_pin_t unlock_button_pin, bool unlock_button_inverted)
      : id_(id),
        track_detector_(track_detector_pin, track_detector_inverted,
                        GPIO_INPUT),
        route_lock_button_(route_lock_button_pin, route_lock_button_inverted,
                           GPIO_INPUT),
        route_em_unlock_button_(unlock_button_pin, unlock_button_inverted,
                                GPIO_INPUT),
        route_locked_lamp_(route_locked_lamp_pin, route_locked_lamp_inverted,
                           GPIO_OUTPUT) {
    BlockRegistry::instance()->register_obj(this, id);
    Executor::instance()->add(this);
  }

  void begin() override {
    // No route locked.
    route_locked_lamp_.write(false);
  }

  void loop() override {}

  const GpioAccessor& track_detector() { return track_detector_; }

  const GpioAccessor& route_lock_button() { return route_lock_button_; }

  const GpioAccessor& route_locked_lamp() { return route_locked_lamp_; }

  const GpioAccessor& route_em_unlock_button() {
    return route_em_unlock_button_;
  }

  /// @return true if an outgoing train is allowed to go towards this
  /// block. This means that we have permission for the track and the track is
  /// free.
  virtual bool allow_outgoing_train() { return true; }
  
  /// Called by the route lever state machine when a route for this block is
  /// set and locked.
  /// @param id the route that was set and locked.
  /// @param is_out true if the route is outbounds (towards the block).
  virtual void notify_route_locked(RouteId id, bool is_out) {
    LOG(LEVEL_INFO, "Route %d (out:%d) locked lamp on", id, is_out); 
    route_locked_lamp_.write(true);
  }

  /// Called by the route lever state machine when a route was set & locked for
  /// this block, and a train has traversed it.
  /// @param id the route that was set, locked and is now unlocked.
  virtual void notify_route_complete(RouteId id) {
    LOG(LEVEL_INFO, "Route %d locked lamp off", id); 
    route_locked_lamp_.write(false);
  }

  /// Called by the route lever state machine when a route was set & locked for
  /// this block, but it was cancelled with the FHT.
  /// @param id the route that was set, locked and is now unlocked.
  virtual void notify_route_cancelled(RouteId id) {
    LOG(LEVEL_INFO, "Route %d locked lamp off", id); 
    route_locked_lamp_.write(false);
  }
  
 protected:
  BlockId id_;

 private:
  /// Block occupancy detector that covers the piece of track that is between
  /// the inbounds signal and the first turnout of the station. When inverted =
  /// false, HIGH means that there is a train on that section.
  GpioAccessor track_detector_;

  /// Route lock button on the user interface. When inverted = false, HIGH
  /// means the button is pressed.
  GpioAccessor route_lock_button_;

  /// Emergency unlock button on the user interface. When inverted = false,
  /// HIGH means the button is pressed.
  GpioAccessor route_em_unlock_button_;
  
  /// A red/white lamp showing whether there is a route locked for this
  /// block. When inverted = false, HIGH means there is a route locked (output
  /// red), LOW means there is no route locked (output white).
  GpioAccessor route_locked_lamp_;

};  // class Block

#endif  // _STW_BLOCK_H_
