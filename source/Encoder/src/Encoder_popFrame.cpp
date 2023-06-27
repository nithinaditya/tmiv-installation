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

#include <TMIV/Encoder/Encoder.h>

namespace TMIV::Encoder {
auto Encoder::popAtlas() -> Common::MVD10Frame {
  incrementFoc();

  if (m_config.haveGeometry) {
    auto frame = m_geometryDownscaler.transformFrame(
        m_geometryQuantizer->transformAtlases(m_videoFrameBuffer.front()));
    m_videoFrameBuffer.pop_front();
    return frame;
  }

  auto frame = Common::MVD10Frame(m_videoFrameBuffer.front().size());
  for (size_t i = 0; i < frame.size(); ++i) {
    frame[i].texture = std::move(m_videoFrameBuffer.front()[i].texture);
  }
  m_videoFrameBuffer.pop_front();
  return frame;
}

void Encoder::incrementFoc() {
  const auto &atlas0 = m_params.atlas.front();
  const auto log2FocLsb = atlas0.asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4U;
  auto focLsb = atlas0.ath.ath_atlas_frm_order_cnt_lsb();
  if (++focLsb >> log2FocLsb == 1U) {
    focLsb = 0;
  }
  for (auto &atlas : m_params.atlas) {
    atlas.ath.ath_atlas_frm_order_cnt_lsb(focLsb);
  }
}
} // namespace TMIV::Encoder
