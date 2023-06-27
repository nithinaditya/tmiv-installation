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

#ifndef _TMIV_MIVBITSTREAM_DEPTHOCCUPANCYTRANSFORM_H_
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/Common.h>

namespace TMIV::MivBitstream {
inline OccupancyTransform::OccupancyTransform(const ViewParams &viewParams) {
  m_threshold = viewParams.dq.dq_depth_occ_map_threshold_default();
  if (m_threshold == 0 && viewParams.hasOccupancy) {
    m_threshold = 1; // Handle invalid depth for source views, transport views and viewports
  }
}

inline OccupancyTransform::OccupancyTransform(const ViewParams &viewParams,
                                              const PatchParams &patchParams) {
  m_threshold = patchParams.atlasPatchDepthOccMapThreshold()
                    ? *patchParams.atlasPatchDepthOccMapThreshold()
                    : viewParams.dq.dq_depth_occ_map_threshold_default();
  if (m_threshold == 0 && viewParams.hasOccupancy) {
    m_threshold = 1; // Handle invalid depth for source views, transport views and viewports
  }
}

inline auto OccupancyTransform::occupant(uint16_t x) const -> bool { return x >= m_threshold; }

template <typename DepthFrame>
auto DepthTransform::quantizeNormDisp(const Common::Mat<float> &matrix, uint16_t minLevel) const
    -> DepthFrame {
  assert(m_bits == DepthFrame::getBitDepth());
  auto frame = DepthFrame{static_cast<int>(matrix.width()), static_cast<int>(matrix.height())};
  std::transform(std::begin(matrix), std::end(matrix), std::begin(frame.getPlane(0)),
                 [=](float x) { return quantizeNormDisp(x, minLevel); });
  return frame;
}

} // namespace TMIV::MivBitstream
