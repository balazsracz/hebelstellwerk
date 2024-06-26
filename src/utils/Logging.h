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
 * \file Logging.h
 *
 * Mechanism to output log entries to the console
 *
 * @author Balazs Racz
 * @date 19 Mar 2024
 */

#ifndef _UTILS_LOGGING_H_
#define _UTILS_LOGGING_H_

#define LEVEL_INFO 2
#define LEVEL_ERROR 3

#ifdef GTEST
#include <stdio.h>

#define LOG(level, fmt...) \
  do {                     \
    fprintf(stderr, fmt);  \
    fprintf(stderr, "\n"); \
  } while (0)

#elif defined(ARDUINO)

static char logbuf__[300];

#define LOG(level, fmt...)                     \
  do {                                         \
    snprintf(logbuf__, sizeof(logbuf__), fmt); \
    Serial.println(logbuf__);                  \
  } while (0)

#endif // Arduino

#include <string>

std::string StringPrintf(const char *format, ...)
    __attribute__((format(printf, 1, 2)));

#endif  // _UTILS_LOGGING_H_
