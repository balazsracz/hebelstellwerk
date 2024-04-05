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
  ROUTE_EXC,
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

/// Starts a new row in the lock table. The row is for a specific route
/// (usually one half of a route lever).
static constexpr LockTableEntry Route(RouteId id) {
  return lock_table_helper(ROUTE_ROW, id);
}

/// Declares that a turnout has to be in the + state in order to allow the
/// current route. It will be locked there when the route is locked.
static constexpr LockTableEntry TurnoutPlus(TurnoutId id) {
  return lock_table_helper(TURNOUT_PLUS, id);
}

/// Alias for TurnoutPlus in German.
static constexpr LockTableEntry WeichePlus(TurnoutId id) {
  return lock_table_helper(TURNOUT_PLUS, id);
}

/// Declares that a turnout has to be in the - state in order to allow the
/// current route. It will be locked there when the route is locked.
static constexpr LockTableEntry TurnoutMinus(TurnoutId id) {
  return lock_table_helper(TURNOUT_MINUS, id);
}

/// Alias for TurnoutMinus in German.
static constexpr LockTableEntry WeicheMinus(TurnoutId id) {
  return lock_table_helper(TURNOUT_MINUS, id);
}

/// Declares that when a route is set and locked, a given signal should be
/// enabled for the aspect Hp1.
static constexpr LockTableEntry Hp1(SignalId id) {
  return lock_table_helper(SIGNAL_HP1, id);
}

/// Declares that when a route is set and locked, a given signal should be
/// enabled for the aspect Hp2.
static constexpr LockTableEntry Hp2(SignalId id) {
  return lock_table_helper(SIGNAL_HP2, id);
}

/// Declares that an Aux input has to be engaged in order to allow the current
/// route.  It will be locked there when the route is locked.
static constexpr LockTableEntry Aux(AuxId id) {
  return lock_table_helper(AUX_PLUS, id);
}

/// Declares that the current route uses the particular block in the outbounds
/// direction. This is a dependency; the block has to be set with permission
/// for outbounds train, and the track has to be free. The detector circuit of
/// this block will be used to determine when the route lock can be released.
static constexpr LockTableEntry BlockOut(BlockId id) {
  return lock_table_helper(BLOCK_OUT, id);
}

/// Declares that the current route uses the particular block in the inbounds
/// direction. This is a not dependency; the block may have an arbitrary
/// state. The detector circuit of this block will be used to determine when
/// the route lock can be released.
static constexpr LockTableEntry BlockIn(BlockId id) {
  return lock_table_helper(BLOCK_IN, id);
}

/// Declares that the given route must NOT be set in order to set the current
/// route. This is purely a dependency. THIS SETTING IS NOT SYMMETRIC. If two
/// routes mutually exclude each other, then the mutual declaration has to be
/// made in both rows of the lock table. An example when routes are not
/// mutually excluding is a Durchfahrt, where the inbounds route has to be set
/// first, then the outbounds route may be set. Combining the routes in the
/// opposite order is however not allowed.
static constexpr LockTableEntry RouteExc(RouteId id) {
  return lock_table_helper(ROUTE_EXC, id);
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

  /// Finds the block which is referenced in a given row of the route table.
  ///
  /// @param route reference to the row with the route.
  /// @param is_outbounds if non-null, will be set to true of the block is
  /// referenced outbounds, to false if its is referenced inbounds.
  ///
  /// @return the ID of the block in the route row.
  ///
  static BlockId find_block(Row route, bool* is_outbounds) {
    for (const auto& entry : route) {
      if (entry.type_ == BLOCK_OUT) {
        if (is_outbounds) *is_outbounds = true;
        return (BlockId)entry.arg_;
      } else if (entry.type_ == BLOCK_IN) {
        if (is_outbounds) *is_outbounds = false;
        return (BlockId)entry.arg_;
      }
    }
    return NO_BLOCK;
  }

  /// Finds the signal which is referenced in a given row of the route table.
  ///
  /// @param route reference to the row with the route.
  /// @param aspect if non-null, will be set to the aspect to which the signal
  /// should be set.
  ///
  /// @return the ID of the signal in the route row.
  ///
  static SignalId find_signal(Row route, SignalAspect *aspect) {
    for (const auto& entry : route) {
      if (entry.type_ == SIGNAL_HP1) {
        if (aspect) *aspect = HP1;
        return (SignalId)entry.arg_;
      } else if (entry.type_ == SIGNAL_HP2) {
        if (aspect) *aspect = HP2;
        return (SignalId)entry.arg_;
      }
    }
    return NO_SIGNAL;
  }
  
 private:
  std::initializer_list<LockTableEntry> ar_;
};

using Verschlusstabelle = LockTable;

#endif  // _STW_LOCKTABLE_H_
