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
 * \file ServoGpio.h
 *
 * Adapter to perform a GPIO output on a PWM pin.
 *
 * @author Balazs Racz
 * @date 18 Mar 2024
 */

#ifndef _UTILS_SERVOGPIO_H_
#define _UTILS_SERVOGPIO_H_

#include "utils/Executable.h"
#include "utils/Executor.h"
#include "utils/Gpio.h"
#include "utils/Pwm.h"
#include "utils/Timer.h"

#include <stdio.h>

class Servo {
 public:
  /// Sets the servo right now to a given degree rotation.
  /// @param deg degree of rotation (nominal 0..180, extended -90..270).
  virtual void command_deg(int16_t deg) = 0;
};

/// Instantiate this class when driving an on/off object, such as a mechanical
/// block, using a servo connected to a PWM pin. The Servo endpoints and
/// running time are parameters.
class ServoGpio : public Gpio, public Servo, private Executable {
 public:
  /// Constructor.
  ///
  /// @param gpio_pin pin number to use for registering this virtual GPIO pin.
  /// @param pwm_pin pin number for the PWM pin to drive for this use-case.
  /// @param deg_off degree of rotation (nominal 0..180, extended -90..270) to
  /// use when the GPIO is commanded to off.
  /// @param deg_on degree of rotation (nominal 0..180, extended -90..270) to
  /// use when the GPIO is commanded to on.
  /// @param travel_time_msec length of time to travel in milliseconds
  /// @param disable_in_steady if true, the servo control line will be
  /// permanently low once reaching steady state. Otherwise the servo line will
  /// continuously command the last commanded degree of rotation.
  /// @param deg_over_off degree of rotation (relative to deg_off, +-90) to
  /// over-rotate on the off side.
  /// @param deg_over_on degree of rotation (relative to deg_on, +-90) to
  /// over-rotate on the on side.
  /// @param overrotate_time_msec length of time to travel back from
  /// over-rotation, in milliseconds
  ServoGpio(gpio_pin_t gpio_pin, pwm_pin_t pwm_pin, int16_t deg_off,
            int16_t deg_on, uint16_t travel_time_msec,
            bool disable_in_steady = false, int16_t deg_over_off = 0,
            int16_t deg_over_on = 0, uint16_t overrotate_time_msec = 0)
      : pwm_pin_(pwm_pin),
        deg_off_(deg_off),
        deg_on_(deg_on),
        travel_time_msec_(travel_time_msec),
        overrotate_time_msec_(overrotate_time_msec),
        deg_over_off_(deg_over_off),
        deg_over_on_(deg_over_on),
        last_state_(false),
        pending_overrotate_(false),
        steady_state_(true),
        commanded_deg_(INT16_MAX),
        source_deg_(0),
        target_deg_(0) {
    Executor::instance()->add(this);
    GpioRegistry::instance()->register_obj(this, gpio_pin);
    pwm_ = PwmRegistry::instance()->get(pwm_pin);
  }

  void begin() override {
    command_deg(deg_off_);
    steady_state_ = true;
  }

  void loop() override {
    if (steady_state_) {
      // nothing to do
      return;
    }
    auto millis = Executor::instance()->millis();
    if (millis >= target_time_millis_) {
      // We reached an end point.
      command_deg(target_deg_);
      if (pending_overrotate_) {
        // Reached overrotation. Set target for real.
        source_time_millis_ = target_time_millis_;
        target_time_millis_ = source_time_millis_ + overrotate_time_msec_;
        source_deg_ = target_deg_;
        target_deg_ = last_state_ ? deg_on_ : deg_off_;
        pending_overrotate_ = false;
        tm_.start_drifting(SERVO_UPDATE_MSEC);
        return;
      }
      // Reached final state.
      steady_state_ = true;
      return;
    }
    // Still in the middle of the travel
    if (tm_.check()) {
      // Update servo position.
      fprintf(stderr, "int(%u/%u, %u/%u, %u)\n", source_time_millis_,
              source_deg_, target_time_millis_, target_deg_, millis);
      auto deg = interpolate(source_time_millis_, source_deg_,
                             target_time_millis_, target_deg_, millis);
      command_deg(deg);
    }
  }

  /// Sets the servo right now to a given degree rotation.
  void command_deg(int16_t deg) override {
    if (deg == commanded_deg_) {
      return;
    }
    commanded_deg_ = deg;
    // one msec = 0 degree. two  msec = 180 degrees.
    int32_t msec = pwm_->tick_per_msec();
    // 0 deg = 1 msec in ticks, 180 deg = 2 msec in ticks, interpolate `deg`.
    int32_t tick = interpolate(0, msec, 180, 2 * msec, deg);
    pwm_->write(pwm_pin_, tick);
  }

  /// Computes a linear interpolation or extrapolation. Based on x/y of two
  /// points (start point or end point) and an x value of an arbitrary point on
  /// this line, either inside or outside of the start points's x values,
  /// computes the y value for that point.
  ///
  /// @param ax key / x coordinate of starting point
  /// @param ay value / y coordinate of starting point
  /// @param bx key / x coordinate of end point
  /// @param by value / y coordinate of end point
  /// @param midx x coordinate of interpolated point
  ///
  /// @return y coordinate of interpolated point
  ///
  static constexpr int32_t interpolate(int32_t ax, int32_t ay, int32_t bx, int32_t by,
                             int32_t midx) {
    int32_t diffx = bx - ax;
    int32_t diffy = by - ay;
    // (ret - ay) / (midx - ax) == (by - ay) / (bx - ax)
    // ret == ay + (by - ay) * (midx - ax) / (bx - ax)
    return ay + diffy * (midx - ax) / diffx;
  }

  void set_output(gpio_pin_t pin) const override {
    // Nothing to do, always an output.
  }

  void write(gpio_pin_t pin, bool value) const override {
    if (value == last_state_) {
      return;
    }
    last_state_ = value;
    steady_state_ = false;
    pending_overrotate_ = true;
    source_deg_ = commanded_deg_;
    source_time_millis_ = Executor::instance()->millis();
    target_time_millis_ = source_time_millis_ + travel_time_msec_;
    if (last_state_) {
      target_deg_ = deg_on_ + deg_over_on_;
    } else {
      target_deg_ = deg_off_ + deg_over_off_;
    }
    tm_.start_drifting(SERVO_UPDATE_MSEC);
  }

  bool read(gpio_pin_t pin) const override { return last_state_; }

 private:
  static constexpr uint32_t SERVO_UPDATE_MSEC = 50;

  /// PWM pin object from the registry.
  const Pwm* pwm_;
  /// Number of the PWM pin to drive.
  const pwm_pin_t pwm_pin_;
  /// Degree of rotation (nominal 0..180, extended -90..270) to
  /// use when the GPIO is commanded to off.
  const int16_t deg_off_;
  /// Degree of rotation (nominal 0..180, extended -90..270) to
  /// use when the GPIO is commanded to on.
  const int16_t deg_on_;
  /// Length of time to travel in milliseconds.
  const uint16_t travel_time_msec_;
  /// Length of time to travel back from over-rotation, in milliseconds.
  const uint16_t overrotate_time_msec_;
  /// Degree of rotation (relative to deg_off, +-90) to over-rotate on the off
  /// side.
  const int16_t deg_over_off_;
  /// Degree of rotation (relative to deg_on, +-90) to over-rotate on the on
  /// side.
  const int16_t deg_over_on_;
  /// Last commanded state of the GPIO pin.
  mutable bool last_state_ : 1;

  /// 1 if we are currently traveling to the overrotate target.
  mutable bool pending_overrotate_ : 1;
  /// 1 if we have reached the steady state.
  mutable bool steady_state_ : 1;

  /// Degree that we commanded last time.
  mutable int16_t commanded_deg_;

  /// Where the current rotation starts from.
  mutable int16_t source_deg_;
  /// Where the current rotation starts from.
  mutable int16_t target_deg_;
  /// Absolute time (from Executor::millis()) when we left from source_deg_.
  mutable uint32_t source_time_millis_{0};
  /// Absolute time (from Executor::millis()) when we should reach the given
  /// target_deg_.
  mutable uint32_t target_time_millis_{0};
  /// Helper class to pace when we are commanding the servos.
  mutable Timer tm_;
};

/// Computes a reverse mapping of a servo pulse length in microseconds
/// (1000..2000) and turns it into a degree.
/// @param micros how long the servo pulse should be, between 1000 and 2000
/// (could also overhang a bit, so 500 to 2500 are OK).
/// @return degree to set the servo to, -90 .. 270.
static constexpr int16_t usec(uint16_t micros) {
  return ServoGpio::interpolate(1000, 0, 2000, 180, micros);
}

#endif  // _UTILS_SERVOGPIO_H_
