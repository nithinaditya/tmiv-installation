/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _TMIV_COMMON_COMMON_H_
#define _TMIV_COMMON_COMMON_H_

// Common data types and functions that are often used and do not need a
// separate header file

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>

namespace TMIV::Common {
constexpr auto radperdeg{0.01745329251994329576923690768489F};
constexpr auto degperrad{57.295779513082320876798154814092F};
constexpr auto pi{3.1415926535897932384626433832795F};
constexpr auto fullCycle{2.F * pi};     // rad
constexpr auto halfCycle{pi};           // rad
constexpr auto quarterCycle{0.5F * pi}; // rad
constexpr auto fullSphere{4.F * pi};    // sr
constexpr auto hemiSphere{2.F * pi};    // sr

// http://open-std.org/JTC1/SC22/WG21/docs/papers/2018/p0051r3.pdf
template <typename... Ts> struct Overload : public Ts... {
  template <typename... Us> Overload(Us &&...values) : Ts{std::forward<Us>(values)}... {}
  using Ts::operator()...;
};
template <typename... Ts>
auto overload(Ts &&...values) -> Overload<std::remove_reference_t<Ts>...> {
  return {std::forward<Ts>(values)...};
}

// The maximum level for an unsigned integer of the specified number of bits
constexpr auto maxLevel(unsigned bits) -> unsigned;

// Expand an integral value to floating-point using a linear transfer function
constexpr auto expandValue(uint16_t x, unsigned bits) -> float {
  assert(0 < bits);
  return static_cast<float>(x) / static_cast<float>(maxLevel(bits));
}

// Quantize a value using a linear transfer function
constexpr auto quantizeValue(float x, unsigned bits) -> uint16_t;

// Does a collection contain a specified value?
template <typename Collection, typename Value>
auto contains(const Collection &collection, Value &&value) -> bool {
  using std::cbegin;
  using std::cend;
  return std::any_of(cbegin(collection), cend(collection),
                     [&value](const auto &x) { return x == value; });
}
} // namespace TMIV::Common

#include "Common.hpp"

#endif
