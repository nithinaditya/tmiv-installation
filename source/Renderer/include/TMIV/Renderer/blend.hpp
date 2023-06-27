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

#ifndef _TMIV_RENDERER_BLEND_H_
#error "Include the .h, not the .hpp"
#endif

#include <cmath>
#include <tuple>
#include <type_traits>

namespace TMIV::Renderer {
// Blend two arithmetic tensors of fixed size
template <typename T> static auto blendValues(float w_a, T a, float w_b, T b) -> T {
  if constexpr (std::is_floating_point_v<T>) {
    return w_a * a + w_b * b;
  } else if constexpr (std::is_integral_v<T>) {
    return static_cast<T>(std::lround(w_a * static_cast<float>(a) + w_b * static_cast<float>(b)));
  } else {
    T result;
    static_assert(result.size() == a.size()); // req. constexpr size()
    for (unsigned i = 0; i < result.size(); ++i) {
      result[i] = blendValues(w_a, a[i], w_b, b[i]);
    }
    return result;
  }
}

// Blend three arithmetic tensors of fixed size
template <typename T> auto blendValues(float w_a, T a, float w_b, T b, float w_c, T c) -> T {
  if constexpr (std::is_floating_point_v<T>) {
    return w_a * a + w_b * b + w_c * c;
  } else if constexpr (std::is_integral_v<T>) {
    return static_cast<T>(std::lround(w_a * static_cast<float>(a) + w_b * static_cast<float>(b) +
                                      w_c * static_cast<float>(c)));
  } else {
    T result;
    static_assert(result.size() == a.size()); // req. constexpr size()
    for (unsigned i = 0; i < result.size(); ++i) {
      result[i] = blendValues(w_a, a[i], w_b, b[i], w_c, c[i]);
    }
    return result;
  }
}

// Blend the attributes of two pixels
template <typename T0, typename... T>
auto blendAttributes(float w_a, const std::tuple<T0, T...> &a, float w_b,
                     const std::tuple<T0, T...> &b) -> std::tuple<T0, T...> {
  std::tuple<T0, T...> result;
  std::get<0>(result) = blendValues(w_a, std::get<0>(a), w_b, std::get<0>(b));
  if constexpr (sizeof...(T) >= 1) {
    std::get<1>(result) = blendValues(w_a, std::get<1>(a), w_b, std::get<1>(b));
  }
  if constexpr (sizeof...(T) >= 2) {
    std::get<2>(result) = blendValues(w_a, std::get<2>(a), w_b, std::get<2>(b));
  }
  if constexpr (sizeof...(T) >= 3) {
    std::get<3>(result) = blendValues(w_a, std::get<3>(a), w_b, std::get<3>(b));
  }
  static_assert(sizeof...(T) <= 3);
  return result;
}

// Blend the attributes of three pixels
template <typename T0, typename... T>
auto blendAttributes(float w_a, const std::tuple<T0, T...> &a, float w_b,
                     const std::tuple<T0, T...> &b, float w_c, const std::tuple<T0, T...> &c)
    -> std::tuple<T0, T...> {
  std::tuple<T0, T...> result;
  std::get<0>(result) = blendValues(w_a, std::get<0>(a), w_b, std::get<0>(b), w_c, std::get<0>(c));
  if constexpr (sizeof...(T) >= 1) {
    std::get<1>(result) =
        blendValues(w_a, std::get<1>(a), w_b, std::get<1>(b), w_c, std::get<1>(c));
  }
  if constexpr (sizeof...(T) >= 2) {
    std::get<2>(result) =
        blendValues(w_a, std::get<2>(a), w_b, std::get<2>(b), w_c, std::get<2>(c));
  }
  if constexpr (sizeof...(T) >= 3) {
    std::get<3>(result) =
        blendValues(w_a, std::get<3>(a), w_b, std::get<3>(b), w_c, std::get<3>(c));
  }
  static_assert(sizeof...(T) <= 3);
  return result;
}
} // namespace TMIV::Renderer
