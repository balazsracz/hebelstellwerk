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
 * \file Macros.h
 *
 * COmmon macros used in many different places.
 *
 * @author Balazs Racz
 * @date 7 Mar 2024
 */

#ifndef _STW_MACROS_H_
#define _STW_MACROS_H_

#include <assert.h>

#define ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))

#if defined(ARDUINO) && defined(LED_BUILTIN)

extern const char* volatile g_death_file;
extern volatile int g_death_lineno;

//     Serial.printf("%s:%d: %s\n", __FILE__, __LINE__, x);

#define DIE(x)                                         \
  g_death_file = __FILE__;                             \
  g_death_lineno = __LINE__;                           \
  if (Serial) {                                        \
    Serial.print(__FILE__);                            \
    Serial.print(":");                                 \
    Serial.print(__LINE__);                            \
    Serial.print(": ");                                \
    Serial.println(x);                                 \
  }                                                    \
  do {                                                 \
    pinMode(LED_BUILTIN, OUTPUT);                      \
    while (1) {                                        \
      volatile int xx = 0;                             \
      digitalWrite(LED_BUILTIN, HIGH);                 \
      for (unsigned j = 0; j < F_CPU / 100; ++j) ++xx; \
      digitalWrite(LED_BUILTIN, LOW);                  \
      for (unsigned j = 0; j < F_CPU / 100; ++j) ++xx; \
    }                                                  \
  } while (0)

#define ASSERT(x)                \
  do {                           \
    if (!(x)) {                  \
      DIE("assert failed: " #x); \
    }                            \
  } while (0)

//      assert((x));

#elif defined(NDEBUG)

#warning bad DIE because NDEBUG

#define DIE(x)                               \
  do {                                       \
    int can_not_compile_die_with_NDEBUG[-1]; \
  } while (0)

#else  // NDEBUG is not defined, use assert to die

#define DIE(x) assert(false && x)

#define ASSERT(x) assert(x)

#endif  // NDEBUG is not defined, use assert

/// Adds a constructor to the current class that proxies every argument to the
/// base constructor.
///
/// @param CURRENT_CLASS the name of the current class
/// @param BASE_CLASS the name of the immediate base class
#define INHERIT_CONSTRUCTOR(CURRENT_CLASS, BASE_CLASS)                         \
    template <typename... Args>                                                \
    explicit CURRENT_CLASS(Args... args)                                       \
        : BASE_CLASS(args...)                                                  \
    {                                                                          \
    }

#endif  // _STW_MACROS_H_
