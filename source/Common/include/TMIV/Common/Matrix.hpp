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

#ifndef _TMIV_COMMON_MATRIX_H_
#error "Include the .h, not the .hpp"
#endif

#include <algorithm>

namespace TMIV::Common {
template <typename A> auto MatrixInterface<A>::isPositive() const -> bool {
  return std::all_of(this->cbegin(), this->cend(),
                     [](const auto element) { return static_cast<value_type>(0.F) < element; });
}

template <typename Mat1, typename Mat2> auto transpose(const Mat1 &in, Mat2 &out) -> Mat2 & {
  out.resize({in.n(), in.m()});

  if (in.isRow() || in.isColumn()) {
    std::copy(in.begin(), in.end(), out.begin());
  } else {
    for (Array::size_type i = 0; i < out.m(); i++) {
      std::copy(in.col_begin(i), in.col_end(i), out.row_begin(i));
    }
  }
  return out;
}

template <typename Mat> auto transpose(const Mat &m) -> decltype(transpose_type(Mat())) {
  decltype(transpose_type(Mat())) out;
  return transpose(m, out);
}
} // namespace TMIV::Common
