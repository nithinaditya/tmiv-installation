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

#include <TMIV/Decoder/OccupancyReconstructor.h>

namespace TMIV::Decoder {
void OccupancyReconstructor::reconstruct(MivBitstream::AccessUnit &frame) {
  for (size_t k = 0; k <= frame.vps.vps_atlas_count_minus1(); ++k) {
    auto &atlas = frame.atlas[k];
    atlas.occFrame = Common::Occupancy10Frame{atlas.frameSize().x(), atlas.frameSize().y()};
    for (auto y = 0; y < atlas.frameSize().y(); y++) {
      for (auto x = 0; x < atlas.frameSize().x(); x++) {
        auto patchId = atlas.patchId(y, x);
        if (patchId == Common::unusedPatchId) {
          atlas.occFrame.getPlane(0)(y, x) = 0;
        } else if (!frame.vps.vps_occupancy_video_present_flag(frame.vps.vps_atlas_id(k))) {
          if (frame.vps.vps_miv_extension().vme_embedded_occupancy_enabled_flag()) {
            // occupancy is embedded in geometry
            uint32_t depthOccupancyThreshold = 0;
            if (!atlas.asps.asps_miv_extension_present_flag() ||
                !atlas.asps.asps_miv_extension().asme_depth_occ_threshold_flag()) {
              uint16_t v = atlas.patchParamsList[patchId].atlasPatchProjectionId();
              depthOccupancyThreshold =
                  frame.viewParamsList[v].dq.dq_depth_occ_map_threshold_default();
            } else {
              depthOccupancyThreshold =
                  *atlas.patchParamsList[patchId].atlasPatchDepthOccMapThreshold();
            }
            if (atlas.asps.asps_miv_extension_present_flag() &&
                atlas.asps.asps_miv_extension().asme_patch_constant_depth_flag()) {
              atlas.occFrame.getPlane(0)(y, x) = 1;
            } else {
              atlas.occFrame.getPlane(0)(y, x) =
                  (atlas.geoFrame.getPlane(0)(y, x) < depthOccupancyThreshold) ? 0 : 1;
            }
          } else {
            // no occupancy information is available (i.e. atlas is complete)
            atlas.occFrame.getPlane(0)(y, x) = 1;
          }
        } else {
          // occupancy is signaled explicitly
          int asmeOccupancyFrameScaleFactorX = 1;
          int asmeOccupancyFrameScaleFactorY = 1;

          const auto &asme = atlas.asps.asps_miv_extension();
          if (!asme.asme_embedded_occupancy_enabled_flag() &&
              asme.asme_occupancy_scale_enabled_flag()) {
            asmeOccupancyFrameScaleFactorX = asme.asme_occupancy_scale_factor_x_minus1() + 1;
            asmeOccupancyFrameScaleFactorY = asme.asme_occupancy_scale_factor_y_minus1() + 1;
          }
          atlas.occFrame.getPlane(0)(y, x) = atlas.decOccFrame.getPlane(0)(
              y / asmeOccupancyFrameScaleFactorY, x / asmeOccupancyFrameScaleFactorX);
        }
      }
    }
  }
}
} // namespace TMIV::Decoder
