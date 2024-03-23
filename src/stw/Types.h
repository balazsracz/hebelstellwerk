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
 * \file stw/Types.h
 *
 * Forward declarations of enums of various objects.
 *
 * @author Balazs Racz
 * @date 21 Mar 2024
 */

#ifndef _STW_TYPES_H_
#define _STW_TYPES_H_

#include <cstdint>

/// This enum lists the routes (Fahrstrassen).
enum RouteId : uint8_t;

/// This enum lists the signals (Signale).
enum SignalId : uint8_t;

static constexpr SignalId NO_SIGNAL = (SignalId)-1;

enum SignalAspect : uint8_t {
  HP0 = 0,
  ASPECT_H = HP0,
  HP1,
  ASPECT_F1 = HP1,
  HP2,
  ASPECT_F2 = HP2
};

/// This enum lists the turnouts (Weichen).
enum TurnoutId : uint8_t;

/// This enum lists the blocks.
enum BlockId : uint8_t;

static constexpr BlockId NO_BLOCK = (BlockId)-1;

/// This enum lists the auxiliary items (X-ing, Gleissperren).
enum AuxId : uint8_t;

#endif // _STW_TYPES_H_
