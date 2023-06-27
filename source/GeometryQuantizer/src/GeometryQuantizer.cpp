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

#include <TMIV/GeometryQuantizer/GeometryQuantizer.h>

#include <TMIV/MivBitstream/DepthOccupancyTransform.h>

#include <iostream>
#include <stdexcept>

namespace TMIV::GeometryQuantizer {
GeometryQuantizer::GeometryQuantizer(uint16_t depthOccThresholdIfSet)
    : m_depthOccThresholdIfSet{depthOccThresholdIfSet} {
  if (depthOccThresholdIfSet < 1) {
    throw std::runtime_error("The depthOccThresholdIfSet parameter is only used when the encoder "
                             "needs to use occupancy. The value 0 is not allowed.");
  }
  if (depthOccThresholdIfSet >= 500) {
    throw std::runtime_error(
        "The GeometryQuantizer component takes a margin equal to the threshold, so "
        "setting the threshold this high will make it impossible to encode depth.");
  }
}

GeometryQuantizer::GeometryQuantizer(const Common::Json & /*unused*/,
                                     const Common::Json &nodeConfig)
    : GeometryQuantizer{
          static_cast<uint16_t>(nodeConfig.require("depthOccThresholdIfSet").as<int>())} {}

auto GeometryQuantizer::setOccupancyParams(MivBitstream::EncoderParams params)
    -> const MivBitstream::EncoderParams & {
  m_inParams = std::move(params);
  m_outParams = m_inParams;
  const auto isOccupancyEmbedded = m_outParams.vps.profile_tier_level().ptl_profile_toolset_idc() !=
                                   MivBitstream::PtlProfilePccToolsetIdc::MIV_Geometry_Absent;
  m_outParams.vme().vme_embedded_occupancy_enabled_flag(isOccupancyEmbedded);
  for (auto &atlas : m_outParams.atlas) {
    atlas.asme().asme_embedded_occupancy_enabled_flag(isOccupancyEmbedded);
  }
  return m_outParams;
}

auto GeometryQuantizer::transformParams(MivBitstream::EncoderParams params)
    -> const MivBitstream::EncoderParams & {
  m_inParams = std::move(params);
  m_outParams = m_inParams;

  for (auto &x : m_outParams.viewParamsList) {
    if (x.hasOccupancy) {
      x.dq.dq_depth_occ_map_threshold_default(m_depthOccThresholdIfSet); // =T
      const auto nearLevel = 1023.F;
      const auto farLevel = static_cast<float>(2 * m_depthOccThresholdIfSet);
      // Mapping is [2T, 1023] --> [old far, near]. What is level 0? (the new far)
      x.dq.dq_norm_disp_low(x.dq.dq_norm_disp_low() +
                            (0.F - farLevel) / (nearLevel - farLevel) *
                                (x.dq.dq_norm_disp_high() - x.dq.dq_norm_disp_low()));
    }
  }

  return m_outParams;
}

auto GeometryQuantizer::transformAtlases(const Common::MVD16Frame &inAtlases)
    -> Common::MVD10Frame {
  auto outAtlases = Common::MVD10Frame{};
  outAtlases.reserve(inAtlases.size());

  for (const auto &inAtlas : inAtlases) {
    outAtlases.emplace_back(
        inAtlas.texture, Common::Depth10Frame{inAtlas.depth.getWidth(), inAtlas.depth.getHeight()});
  }

  for (const auto &patch : m_outParams.patchParamsList) {
    const auto &inViewParams = m_inParams.viewParamsList[patch.atlasPatchProjectionId()];
    const auto &outViewParams = m_outParams.viewParamsList[patch.atlasPatchProjectionId()];
    const auto inOccupancyTransform = MivBitstream::OccupancyTransform{inViewParams};
#ifndef NDEBUG
    const auto outOccupancyTransform = MivBitstream::OccupancyTransform{outViewParams, patch};
#endif
    const auto inDepthTransform = MivBitstream::DepthTransform{inViewParams.dq, 16};
    const auto outDepthTransform = MivBitstream::DepthTransform{outViewParams.dq, patch, 10};
    const auto kIn = m_inParams.vps.indexOf(patch.atlasId);
    const auto kOut = m_outParams.vps.indexOf(patch.atlasId);

    for (size_t i = 0; i < static_cast<size_t>(patch.atlasPatch2dSizeY()); ++i) {
      for (size_t j = 0; j < static_cast<size_t>(patch.atlasPatch2dSizeX()); ++j) {
        const auto n = i + patch.atlasPatch2dPosY();
        const auto m = j + patch.atlasPatch2dPosX();

        const auto &plane = inAtlases[kIn].depth.getPlane(0);

        if (n >= plane.height() || m >= plane.width()) {
          abort();
        }

        const auto inLevel = plane(n, m);

        if (inOccupancyTransform.occupant(inLevel)) {
          const auto normDisp = inDepthTransform.expandNormDisp(inLevel);
          const auto outLevel = outDepthTransform.quantizeNormDisp(normDisp, 0);
          assert(outOccupancyTransform.occupant(outLevel));

          outAtlases[kOut].depth.getPlane(0)(n, m) = outLevel;
        }
      }
    }
  }

  return outAtlases;
}
} // namespace TMIV::GeometryQuantizer
