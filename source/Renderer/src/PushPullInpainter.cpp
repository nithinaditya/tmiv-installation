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

#include <TMIV/Renderer/PushPullInpainter.h>

#include "PushPull.h"

namespace TMIV::Renderer {
PushPullInpainter::PushPullInpainter(const Common::Json & /* rootNode */,
                                     const Common::Json & /* componentNode */) {}

namespace {
using YUVD = std::tuple<uint16_t, uint16_t, uint16_t, uint16_t>;

constexpr auto occupant(const YUVD &x) -> bool { return 0 < std::get<3>(x); }

auto weighedAverageWithMissingData(const std::array<YUVD, 4> &v, const std::array<int, 4> &weights)
    -> YUVD {
  auto sum = std::array<int, 4>{};
  auto count = 0;
  for (int i = 0; i < 4; ++i) {
    if (occupant(v[i])) {
      sum[0] += weights[i] * std::get<0>(v[i]);
      sum[1] += weights[i] * std::get<1>(v[i]);
      sum[2] += weights[i] * std::get<2>(v[i]);
      sum[3] += weights[i] * std::get<3>(v[i]);
      count += weights[i];
    }
  }
  if (count == 0) {
    static_assert(!occupant(YUVD{}));
    return YUVD{};
  }
  return YUVD{(sum[0] + count / 2) / count, (sum[1] + count / 2) / count,
              (sum[2] + count / 2) / count, (sum[3] + count / 2) / count};
}

auto pushFilter(const std::array<YUVD, 4> &v) -> YUVD {
  // { 1/2, 1/2 } x { 1/2, 1/2 } = { 1/4, 1/4, 1/4, 1/4 }
  static const auto linInterpPushWeights = std::array{1, 1, 1, 1};
  return weighedAverageWithMissingData(v, linInterpPushWeights);
}

auto pullFilter(const std::array<YUVD, 4> &v, const YUVD &x) -> YUVD {
  if (occupant(x)) {
    return x;
  }
  // { 3/4, 1/4 } x { 3/4, 1/4 } = { 9/16, 3/16, 3/16, 1/16 }
  static const auto linInterpPullWeights = std::array{9, 3, 3, 1};
  return weighedAverageWithMissingData(v, linInterpPullWeights);
}
} // namespace

void PushPullInpainter::inplaceInpaint(Common::Texture444Depth16Frame &viewport,
                                       const MivBitstream::ViewParams & /* metadata */) const {
  auto pushPull = PushPull{};
  viewport = pushPull.filter(viewport, pushFilter, pullFilter);
}
} // namespace TMIV::Renderer
