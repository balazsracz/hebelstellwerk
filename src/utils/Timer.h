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
 * \file Timer.h
 *
 * Class to perform periodic actions in polling loops.
 *
 * @author Balazs Racz
 * @date 8 Mar 2024
 */

#ifndef _UTILS_TIMER_H_
#define _UTILS_TIMER_H_

#include "utils/Executor.h"
#include "utils/Types.h"

/// Helper class to keep track of time or perform a timed action.
///
/// Usage:
///
/// - Start the timer either with the constructor
///
/// `Timer tm(Timer::OneShot{350});`
///
/// or with the `tm.start_oneshot(350);` call.
///
/// - In the loop(), add the following code:
///
/// ```
///   if (tm.check()) {
///     // perform action upon timeout here
///   }
/// ```
///
/// Timer comes in three varieties:
///
/// - Oneshot timer invokes the code exactly once. Then it needs to be started
///   again.
///
/// - Periodic timer tried to invoke the code in exactly the given period, and
///   compensates for any drift of the loop being slow. This means that if one
///   period got longer due to loop not calling check, the next period will be
///   shortened. This can go to extremes when after a longer hiccup multiple
///   back to back loop functions will execute the timer body. This is good for
///   keeping track of time.
///
/// - Drifting timer, which always delays at least the given amount of time
///   (but possibly more) between two expiries. This is good for polling for
///   changes on I/O.
class Timer {
 public:
  /// Constructor creating a stopped timer.
  Timer() : nextExpiryMillis_(0), restart_(0), drift_(0), periodMillis_(0) {}

  /// Helper srtucture for disambiguating constructors.
  struct OneShot {
    millis_t millis;
  };
  /// Helper srtucture for disambiguating constructors.
  struct Periodic {
    millis_t millis;
  };
  /// Helper srtucture for disambiguating constructors.
  struct Drifting {
    millis_t millis;
  };

  /// Constructor creating a one-shot timer.
  Timer(OneShot arg) { start_oneshot(arg.millis); }

  /// Constructor creating a periodic timer.
  Timer(Periodic arg) { start_periodic(arg.millis); }

  /// Constructor creating a drifting timer.
  Timer(Drifting arg) { start_drifting(arg.millis); }

  /// Starts a periodic timer without drift. The timer will compensate for
  /// execution jitter by shortening periods. This is acceptable to keep time
  /// over a long term.
  void start_periodic(millis_t msec) {
    nextExpiryMillis_ = get_time() + msec;
    restart_ = 1;
    drift_ = 0;
    periodMillis_ = msec;
  }

  /// Starts a periodic timer with drift. This timer will ensure that between
  /// each two invocations at least msec delay elapses. This is ideal for
  /// polling peripherals, where it does not make sense to shorten period
  /// between invocations.
  void start_drifting(millis_t msec) {
    nextExpiryMillis_ = get_time() + msec;
    restart_ = 1;
    drift_ = 1;
    periodMillis_ = msec;
  }

  /// Starts a non-repeating timer.
  void start_oneshot(millis_t msec) {
    nextExpiryMillis_ = get_time() + msec;
    restart_ = 0;
    drift_ = 0;
    periodMillis_ = 0;
  }

  /// Checks the expiry of the timer. Call this from loop().
  /// @return true if the timer is expired. The caller is required to perfom
  /// the timer action.
  bool check() {
    auto t = get_time();
    if (nextExpiryMillis_ == 0 || t < nextExpiryMillis_) {
      // not set or not expired.
      return false;
    }
    if (!restart_) {
      nextExpiryMillis_ = 0;
    } else if (drift_) {
      nextExpiryMillis_ = t + periodMillis_;
    } else {
      nextExpiryMillis_ += periodMillis_;
    }
    return true;
  }

  /// @return true if this timer is running, false if it is stopped.
  bool is_running() { return nextExpiryMillis_ > 0; }

 private:
  static millis_t get_time() { return Executor::instance()->millis(); }

  /// Absolute time when the timer will next expire. 0 if the timer is stopped.
  millis_t nextExpiryMillis_;
  /// 1 if the timer should be restarted after expiry.
  millis_t restart_ : 1;
  /// 1 if the restart should drift.
  millis_t drift_ : 1;
  /// What should be the period after expiry.
  millis_t periodMillis_ : 30;
};

#endif  // _UTILS_TIMER_H_
