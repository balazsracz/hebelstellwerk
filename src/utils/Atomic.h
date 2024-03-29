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
 * \file Atomic.h
 *
 * An empty lock class that can be used for multi-threaded arduino
 * implementations.
 *
 * @author Balazs Racz
 * @date 7 Mar 2024
 */

#ifndef _STW_ATOMIC_H_
#define _STW_ATOMIC_H_

/// Lightweight locking class for protecting small critical sections.
///
/// Properties:
/// - May be recursively acquired
/// - FreeRTOS: No call inside an Atomic-protected section may block
/// - FreeRTOS: Not allowed to use from interrupt context. Kernel-compatible
///   ISRs are disabled during an Atomic held.
///
/// Under FreeRTOS locking a mutex is more than 2x more expensive than locking
/// an Atomic. On desktop OS's Atomic is just a recursive mutex.
///
/// Usage: Declare Atomic as a private base class, add a class member
/// variable or a global variable of type Atomic. Then use AtomicHolder to
/// protect the critical sections.
#if defined(ESP_PLATFORM)
class Atomic {
 public:
  /// Locks the specific critical section.
  void lock() { portENTER_CRITICAL_SAFE(&mux); }
  /// Unlocks the specific critical section.
  void unlock() { portEXIT_CRITICAL_SAFE(&mux); }

 private:
  /// Performs fine-grained spinloop locking.
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
};

#else
struct Atomic {
  // This is the empty implementation for a single-threaded arduino execution
  // environment. It will be optimized away by the compiler entirely.
  
  /// Locks the specific critical section.
  void lock() {}

  /// Unlocks the specific critical section.
  void unlock() {}
};
#endif

class AtomicHolder {
 public:
  /// Constructor. Grabs the mutex as a side effect.
  ///
  /// @param parent the mutex (atomic) to hold.
  ///
  AtomicHolder(Atomic *parent) : parent_(parent) { parent_->lock(); }
  /// Destructor. Releases the mutex as a side effect.
  ~AtomicHolder() { parent_->unlock(); }

 private:
  /// Parent mutex we are holding.
  Atomic *parent_;
};

/// This stanza catches a common bug when someone allocates an AtomicHolder
/// without specifying the variable name:
///
/// BAD:
/// AtomicHolder(&lock_);
///
/// GOOD:
/// AtomicHolder h(&lock_);
///
/// The problem with the first instance is that is creates a temporary obejct
/// that gets immediately destructed. Thus while the code seems like correct,
/// it does not actually keep the lock.
#define AtomicHolder(l) int error_omitted_lock_holder_variable[-1]

#endif  // _STW_ATOMIC_H_
