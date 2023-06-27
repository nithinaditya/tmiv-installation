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

#ifndef _TMIV_GEOMETRYQUANTIZER_EXPLICITOCCUPANCY_H_
#define _TMIV_GEOMETRYQUANTIZER_EXPLICITOCCUPANCY_H_

#include <TMIV/GeometryQuantizer/IGeometryQuantizer.h>

#include <TMIV/Common/Json.h>

namespace TMIV::GeometryQuantizer {
class ExplicitOccupancy : public IGeometryQuantizer {
public:
  // Initialize with specified depthOccMapThresholdIfSet
  //
  // When incoming view parameters have useOccupancy() set, then the outgoing view parameters
  // will have the specified depthOccMapThresholdIfSet value.
  // explicit ExplicitOccupancy(uint16_t depthOccMapThresholdIfSet);

  ExplicitOccupancy(const Common::Json & /*unused*/, const Common::Json & /*unused*/);
  ExplicitOccupancy(const ExplicitOccupancy &) = default;
  ExplicitOccupancy(ExplicitOccupancy &&) = default;
  auto operator=(const ExplicitOccupancy &) -> ExplicitOccupancy & = default;
  auto operator=(ExplicitOccupancy &&) -> ExplicitOccupancy & = default;
  ~ExplicitOccupancy() override = default;

  auto setOccupancyParams(MivBitstream::EncoderParams params)
      -> const MivBitstream::EncoderParams & override;
  // No change when useOccupancy() is false. Otherwise set the depth/occupancy map threshold
  // to depthOccMapThresholdIfSet and adjust the normalized disparity range.
  auto transformParams(MivBitstream::EncoderParams) -> const MivBitstream::EncoderParams & override;

  void padGeometryFromLeft(Common::MVD10Frame &atlases);

  // Transform depth bit depth and range
  auto transformAtlases(const Common::MVD16Frame &inAtlases) -> Common::MVD10Frame override;

private:
  // uint16_t m_depthOccMapThresholdIfSet{};
  MivBitstream::EncoderParams m_inParams;
  MivBitstream::EncoderParams m_outParams;
  Common::Vec2i m_occupancyScale;
  bool m_occupancyScaleConfig;
  bool m_depthLowQualityFlag{};
  bool m_embeddedOccupancyFlag{};
  bool m_occupancyScaleEnabledFlag{};
};
} // namespace TMIV::GeometryQuantizer

#endif
