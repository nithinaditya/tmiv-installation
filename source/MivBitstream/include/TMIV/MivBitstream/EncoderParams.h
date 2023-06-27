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

#ifndef _TMIV_MIVBITSTREAM_ENCODERPARAMS_H_
#define _TMIV_MIVBITSTREAM_ENCODERPARAMS_H_

#include <TMIV/MivBitstream/AtlasAdaptationParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasTileLayerRBSP.h>
#include <TMIV/MivBitstream/CommonAtlasFrameRBSP.h>
#include <TMIV/MivBitstream/CommonAtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/PatchParamsList.h>
#include <TMIV/MivBitstream/V3cParameterSet.h>
#include <TMIV/MivBitstream/ViewParamsList.h>
#include <TMIV/MivBitstream/ViewingSpace.h>

namespace TMIV::MivBitstream {
struct EncoderAtlasParams {
  EncoderAtlasParams();

  AtlasSequenceParameterSetRBSP asps;
  AtlasFrameParameterSetRBSP afps;
  AtlasTileHeader ath;

  // Convenience function to access the ASPS MIV extension
  [[nodiscard]] auto asme() const noexcept -> const AspsMivExtension &;

  // Convenience function to create and access the ASPS MIV extension
  [[nodiscard]] auto asme() noexcept -> AspsMivExtension &;

  friend auto operator<<(std::ostream &, const EncoderAtlasParams &) -> std::ostream &;
  auto operator==(const EncoderAtlasParams &other) const -> bool;
  auto operator!=(const EncoderAtlasParams &other) const -> bool { return !operator==(other); }
};

struct EncoderParams {
  EncoderParams();
  explicit EncoderParams(bool haveTextureVideo, bool haveGeometryVideo, bool haveOccupancyVideo);
  EncoderParams(const Common::SizeVector &atlasSizes, bool haveTextureVideo, bool haveGeometryVideo,
                bool haveOccupancyVideo);

  explicit EncoderParams(std::uint8_t textureBitDepth, std::uint8_t occupancyBitDepth,
                         std::uint8_t geometryBitDepth, std::uint8_t transparencyBitDepth);
  EncoderParams(const Common::SizeVector &atlasSizes, std::uint8_t textureBitDepth,
                std::uint8_t occupancyBitDepth, std::uint8_t geometryBitDepth,
                std::uint8_t transparencyBitDepth);

  V3cParameterSet vps;
  CommonAtlasSequenceParameterSetRBSP casps;
  std::optional<ViewingSpace> viewingSpace{};

  double frameRate{};
  ViewParamsList viewParamsList;
  PatchParamsList patchParamsList;
  bool lengthsInMeters{true};

  bool dqParamsPresentFlag{true};
  std::uint16_t maxEntityId{0};

  bool randomAccess{};

  std::vector<EncoderAtlasParams> atlas;

  // Convenience function to access the CASPS MIV extension
  [[nodiscard]] auto casme() const noexcept -> const CaspsMivExtension &;

  // Convenience function to create and access the CASPS MIV extension
  [[nodiscard]] auto casme() noexcept -> CaspsMivExtension &;

  // Convenience function to access the VPS MIV extension
  [[nodiscard]] auto vme() const noexcept -> const VpsMivExtension &;

  // Convenience function to create and access the VPS MIV extension
  [[nodiscard]] auto vme() noexcept -> VpsMivExtension &;

  // Convenience function to help the transition
  [[nodiscard]] auto atlasSizes() const -> Common::SizeVector;

  friend auto operator<<(std::ostream &, const EncoderParams &) -> std::ostream &;
  auto operator==(const EncoderParams &other) const -> bool;
  auto operator!=(const EncoderParams &other) const -> bool { return !operator==(other); }
};
} // namespace TMIV::MivBitstream

#include "EncoderParams.hpp"

#endif
