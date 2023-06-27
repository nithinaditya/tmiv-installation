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

#ifndef _TMIV_COMMON_FILTER_H_
#define _TMIV_COMMON_FILTER_H_

#include "Matrix.h"
#include "Vector.h"

#include <cassert>

namespace TMIV::Common {
template <typename S, typename T> auto integralImage(const Mat<T> &in) -> Mat<S> {
  const auto rows = in.height();
  const auto cols = in.width();

  auto out = Mat<S>({rows + 1, cols + 1});

  for (size_t i = 1; i <= rows; ++i) {
    for (size_t j = 1; j <= cols; ++j) {
      out(i, j) = S{in(i - 1, j - 1)} + out(i - 1, j) + out(i, j - 1) - out(i - 1, j - 1);
    }
  }
  return out;
}

template <typename S>
auto sumRect(const Mat<S> &ii, const Vec2i &p_1, const Vec2i &p_2) noexcept -> S {
  assert(p_1.x() <= p_2.x() && p_1.y() <= p_2.y());
  const auto i_1 = std::max(0, p_1.y());
  const auto j_1 = std::max(0, p_1.x());
  const auto i_2 = std::min(static_cast<int>(ii.height() - 1), p_2.y());
  const auto j_2 = std::min(static_cast<int>(ii.width() - 1), p_2.x());

  if (i_1 < i_2 && j_1 < j_2) {
    return ii(i_1, j_1) - ii(i_1, j_2) - ii(i_2, j_1) + ii(i_2, j_2);
  }
  return S{};
}

template <typename S>
auto countRect(const Mat<S> &ii, const Vec2i &p_1, const Vec2i &p_2) noexcept -> int {
  assert(p_1.x() <= p_2.x() && p_1.y() <= p_2.y());
  const auto i_1 = std::max(0, p_1.y());
  const auto j_1 = std::max(0, p_1.x());
  const auto i_2 = std::min(static_cast<int>(ii.height() - 1), p_2.y());
  const auto j_2 = std::min(static_cast<int>(ii.width() - 1), p_2.x());

  if (i_1 < i_2 && j_1 < j_2) {
    return (i_2 - i_1) * (j_2 - j_1);
  }
  return 0;
}

template <typename S, typename T> auto boxBlur(const Mat<T> &in, int k) -> Mat<T> {
  const auto ii = integralImage<S>(in);
  auto out = Mat<T>{{in.height(), in.width()}};
  const auto rows = static_cast<int>(in.height());
  const auto cols = static_cast<int>(in.width());

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      const auto p_1 = Vec2i{j - k, i - k};
      const auto p_2 = Vec2i{j + k + 1, i + k + 1};
      const auto sum = sumRect(ii, p_1, p_2);
      const auto count = countRect(ii, p_1, p_2);
      if (0 < count) {
        if (0 <= sum) {
          out(i, j) = (sum + count / 2) / count;
        } else {
          out(i, j) = (sum - count / 2) / count;
        }
      }
    }
  }
  return out;
}
} // namespace TMIV::Common

#endif
