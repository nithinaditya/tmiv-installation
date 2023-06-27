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

#ifndef _TMIV_COMMON_MATH_H_
#define _TMIV_COMMON_MATH_H_

#include <cmath>
#include <complex>
#include <limits>

namespace TMIV::Common {
#ifndef M_PI
constexpr double M_PI = 3.141592653589793238462643383279;
#endif
constexpr double M_PI2 = M_PI / 2.0;

template <typename T> auto deg2rad(T x) -> T { return x * static_cast<T>(M_PI / 180.); }
template <typename T> auto rad2deg(T x) -> T { return x * static_cast<T>(180. / M_PI); }

template <typename T> auto sqr(T val) -> T { return val * val; }
template <typename T> auto cube(T val) -> T { return val * val * val; }
template <typename T> auto sgn(T val) -> int { return int{T{} < val} - int{val < T{}}; }
template <typename T> auto inRange(T val, T min, T max) -> bool {
  return ((min <= val) && (val <= max));
}
template <typename T> auto is_zero(T val) -> T {
  using std::abs;
  return (abs(val) < std::numeric_limits<T>::epsilon());
}
template <typename T> auto pps2ppd(T pps) -> T {
  return std::sqrt(pps) * static_cast<T>(M_PI / 180.);
}

template <typename T> auto conjugate(T v) -> T {
  if constexpr (std::is_arithmetic_v<T>) {
    return v;
  } else {
    return std::conj(v);
  }
}
template <typename T> auto align(T value, T alignment) -> T {
  T misalignment = value % alignment;
  return (misalignment != 0) ? (value + (alignment - misalignment)) : value;
}

} // namespace TMIV::Common

#endif
