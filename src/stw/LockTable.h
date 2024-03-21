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
 * \file LockTable.h
 *
 * Declarations for representing the lock table (Verschlusstabelle).
 *
 * @author Balazs Racz
 * @date 21 Mar 2024
 */

#ifndef _STW_LOCKTABLE_H_
#define _STW_LOCKTABLE_H_

#include <cstddef>
#include <initializer_list>
#include <utility>

#include "stw/Types.h"
#include "utils/Singleton.h"

/// All the different symbols we can have in a lock table.
enum LockTableEntryType : uint8_t {
  NONE,
  ROUTE_ROW,
  TURNOUT_PLUS,
  TURNOUT_MINUS,
  SIGNAL_HP1,
  SIGNAL_F1 = SIGNAL_HP1,
  SIGNAL_HP2,
  SIGNAL_F2 = SIGNAL_HP2,
  AUX_PLUS,
  BLOCK_OUT,
  BLOCK_IN,
};

/// We keep entries of this type in the lock table.
struct LockTableEntry {
  constexpr LockTableEntry(LockTableEntryType type, uint8_t arg)
      : type_(type), arg_(arg) {}
  LockTableEntryType type_;
  uint8_t arg_;

  bool operator!=(const LockTableEntry o) const {
    return type_ != o.type_ || arg_ != o.arg_;
  }
};

static_assert(sizeof(LockTableEntry) == 2,
              "lock table entry should be optimal");

static constexpr LockTableEntry lock_table_helper(LockTableEntryType type,
                                                  uint8_t value) {
  return LockTableEntry(type, value);
  // (((uint16_t)type) << 8) | value;
}

static constexpr LockTableEntry Route(RouteId id) {
  return lock_table_helper(ROUTE_ROW, id);
}

static constexpr LockTableEntry TurnoutPlus(TurnoutId id) {
  return lock_table_helper(TURNOUT_PLUS, id);
}

static constexpr LockTableEntry WeichePlus(TurnoutId id) {
  return lock_table_helper(TURNOUT_PLUS, id);
}

static constexpr LockTableEntry TurnoutMinus(TurnoutId id) {
  return lock_table_helper(TURNOUT_MINUS, id);
}

static constexpr LockTableEntry WeicheMinus(TurnoutId id) {
  return lock_table_helper(TURNOUT_MINUS, id);
}

static constexpr LockTableEntry Hp1(SignalId id) {
  return lock_table_helper(SIGNAL_HP1, id);
}

static constexpr LockTableEntry Hp2(SignalId id) {
  return lock_table_helper(SIGNAL_HP2, id);
}

/// Instantiate this class to provide the lock table (Verschlusstabelle). The
/// necessary syntax is this:
///
/// LockTable tbl({ //
///   Route(ROUTE_A1), TurnoutPlus(TURNOUT_1), Hp2(SIGNAL_A), //
///   Route(ROUTE_B1), TurnoutMinus(TURNOUT_8), Hp1(SIGNAL_C), //
/// });
class LockTable : public Singleton<LockTable> {
 public:
  LockTable(std::initializer_list<LockTableEntry> entries) : ar_(entries) {}

  const LockTableEntry* begin() { return ar_.begin(); }

  const LockTableEntry* end() { return ar_.end(); }

  /// Data structure representing a row in the lock table.
  struct Row {
    /// Points to the first entry in the row; may be nullptr if size == 0.
    const LockTableEntry* ptr;
    /// Number of entries in the row.
    ptrdiff_t size;

    const LockTableEntry* begin() const { return ptr; }
    const LockTableEntry* end() const { return ptr + size; }
  };

  /// Finds a given row in the lock table, and returns it.
  Row find_route(RouteId id) {
    const LockTableEntry* ptr;
    for (ptr = begin(); ptr != end() && *ptr != Route(id); ++ptr)
      ;
    if (ptr == end()) {
      return {nullptr, 0};
    }
    ++ptr;
    const LockTableEntry* eptr;
    for (eptr = ptr; eptr != end() && eptr->type_ != ROUTE_ROW; ++eptr)
      ;
    return {ptr, eptr - ptr};
  }

 private:
  std::initializer_list<LockTableEntry> ar_;
};

using Verschlusstabelle = LockTable;

#endif  // _STW_LOCKTABLE_H_
