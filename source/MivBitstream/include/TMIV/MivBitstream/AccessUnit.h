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

#ifndef _TMIV_MIVBITSTREAM_ACCESSUNIT_H_
#define _TMIV_MIVBITSTREAM_ACCESSUNIT_H_

#include <TMIV/MivBitstream/AtlasAdaptationParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasTileLayerRBSP.h>
#include <TMIV/MivBitstream/GeometryUpscalingParameters.h>
#include <TMIV/MivBitstream/PatchParamsList.h>
#include <TMIV/MivBitstream/SequenceConfig.h>
#include <TMIV/MivBitstream/V3cParameterSet.h>
#include <TMIV/MivBitstream/ViewParamsList.h>
#include <TMIV/MivBitstream/ViewingSpace.h>

#include <TMIV/Common/Frame.h>

namespace TMIV::MivBitstream {
struct AtlasAccessUnit {
  AtlasSequenceParameterSetRBSP asps;
  AtlasFrameParameterSetRBSP afps;

  Common::Depth10Frame decGeoFrame;
  Common::Depth10Frame geoFrame;
  Common::Texture444Frame attrFrame;
  Common::Occupancy10Frame decOccFrame;
  Common::Occupancy10Frame occFrame;
  Common::Transparency10Frame transparencyFrame;

  Common::BlockToPatchMap blockToPatchMap;
  PatchParamsList patchParamsList;

  // Nominal atlas frame size
  [[nodiscard]] auto frameSize() const noexcept -> Common::Vec2i;

  // Geometry frame size
  [[nodiscard]] auto decGeoFrameSize(const V3cParameterSet &vps) const noexcept -> Common::Vec2i;

  // Occupancy frame size
  [[nodiscard]] auto decOccFrameSize(const V3cParameterSet &vps) const noexcept -> Common::Vec2i;

  // Index into the block to patch map using nominal atlas coordinates
  [[nodiscard]] auto patchId(unsigned row, unsigned column) const -> uint16_t;
};

struct AccessUnit {
  bool irap{};
  std::int32_t foc{-1};
  V3cParameterSet vps;
  std::optional<CommonAtlasSequenceParameterSetRBSP> casps;
  ViewParamsList viewParamsList;
  std::vector<AtlasAccessUnit> atlas;
  std::optional<ViewingSpace> vs;
  std::optional<VuiParameters> vui;
  std::optional<GeometryUpscalingParameters> gup;

  [[nodiscard]] auto sequenceConfig() const -> SequenceConfig;
};

void requireAllPatchesWithinProjectionPlaneBounds(const ViewParamsList &vpl,
                                                  const PatchParamsList &ppl);
} // namespace TMIV::MivBitstream

#endif
