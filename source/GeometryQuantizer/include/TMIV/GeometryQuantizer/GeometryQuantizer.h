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

#ifndef _TMIV_GEOMETRYQUANTIZER_GEOMETRYQUANTIZER_H_
#define _TMIV_GEOMETRYQUANTIZER_GEOMETRYQUANTIZER_H_

#include <TMIV/GeometryQuantizer/IGeometryQuantizer.h>

#include <TMIV/Common/Json.h>

namespace TMIV::GeometryQuantizer {
class GeometryQuantizer : public IGeometryQuantizer {
public:
  // Initialize with specified depthOccThresholdIfSet
  //
  // When incoming view parameters have useOccupancy() set, then the outgoing view parameters
  // will have the specified depthOccThresholdIfSet value.
  explicit GeometryQuantizer(uint16_t depthOccThresholdIfSet);

  GeometryQuantizer(const Common::Json & /*unused*/, const Common::Json & /*unused*/);
  GeometryQuantizer(const GeometryQuantizer &) = default;
  GeometryQuantizer(GeometryQuantizer &&) = default;
  auto operator=(const GeometryQuantizer &) -> GeometryQuantizer & = default;
  auto operator=(GeometryQuantizer &&) -> GeometryQuantizer & = default;
  ~GeometryQuantizer() override = default;

  auto setOccupancyParams(MivBitstream::EncoderParams params)
      -> const MivBitstream::EncoderParams & override;
  // No change when useOccupancy() is false. Otherwise set the depth/occupancy map threshold
  // to depthOccThresholdIfSet and adjust the normalized disparity range.
  auto transformParams(MivBitstream::EncoderParams params)
      -> const MivBitstream::EncoderParams & override;

  // Transform depth bit depth and range
  auto transformAtlases(const Common::MVD16Frame &inAtlases) -> Common::MVD10Frame override;

private:
  uint16_t m_depthOccThresholdIfSet{};
  MivBitstream::EncoderParams m_inParams;
  MivBitstream::EncoderParams m_outParams;
};
} // namespace TMIV::GeometryQuantizer

#endif
