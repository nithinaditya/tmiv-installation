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

#ifndef _TMIV_MIVBITSTREAM_PATCHPARAMSLIST_H_
#define _TMIV_MIVBITSTREAM_PATCHPARAMSLIST_H_

#include <TMIV/MivBitstream/AtlasTileLayerRBSP.h>

#include <TMIV/Common/LinAlg.h>
#include <TMIV/Common/Matrix.h>
#include <TMIV/Common/Vector.h>

#include <cstdint>
#include <iosfwd>
#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
// PatchParams is the in-memory representation of PatchDataUnit (PDU). The PDU is not suitable for
// in-memory use because of the delta coding and quantization of some of the fields.
struct PatchParams {
  AtlasId atlasId{};

  [[nodiscard]] constexpr auto atlasPatch2dPosX() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch2dPosY() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch2dSizeX() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch2dSizeY() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch3dOffsetU() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch3dOffsetV() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch3dOffsetD() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch3dRangeD() const noexcept;
  [[nodiscard]] constexpr auto atlasPatchProjectionId() const noexcept;
  [[nodiscard]] constexpr auto atlasPatchOrientationIndex() const noexcept;
  [[nodiscard]] constexpr auto atlasPatchLoDScaleX() const noexcept;
  [[nodiscard]] constexpr auto atlasPatchLoDScaleY() const noexcept;
  [[nodiscard]] constexpr auto atlasPatchEntityId() const noexcept;
  [[nodiscard]] constexpr auto atlasPatchDepthOccMapThreshold() const noexcept;
  [[nodiscard]] auto atlasPatchAttributeOffset() const;
  [[nodiscard]] constexpr auto atlasPatchInpaintFlag() const noexcept;

  constexpr auto atlasPatch2dPosX(std::int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch2dPosY(std::int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch2dSizeX(std::int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch2dSizeY(std::int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch3dOffsetU(std::int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch3dOffsetV(std::int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch3dOffsetD(std::int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch3dRangeD(std::int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatchProjectionId(std::uint16_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatchOrientationIndex(FlexiblePatchOrientation value) noexcept
      -> PatchParams &;
  constexpr auto atlasPatchLoDScaleX(std::int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatchLoDScaleY(std::int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatchEntityId(std::uint16_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatchDepthOccMapThreshold(std::uint32_t value) noexcept -> PatchParams &;
  auto atlasPatchAttributeOffset(Common::Vec3w value) noexcept -> PatchParams &;
  constexpr auto atlasPatchInpaintFlag(bool value) noexcept -> PatchParams &;

  // Is the patch rotated such that width and height swap?
  [[nodiscard]] constexpr auto isRotated() const noexcept;
  [[nodiscard]] constexpr decltype(auto) atlasPatch3dSizeU() const noexcept;
  [[nodiscard]] constexpr decltype(auto) atlasPatch3dSizeV() const noexcept;
  constexpr decltype(auto) atlasPatch3dSizeU(std::int32_t value) noexcept;
  constexpr decltype(auto) atlasPatch3dSizeV(std::int32_t value) noexcept;

  // Pixel position conversion from atlas (x, y) to view (u, v)
  //
  // * Implements ISO/IEC 23090-5:2021(2E) V3E [WG 07 N 0003], Eq. (49)
  // * Although compilers may be smart enough, computing the transform first may be faster
  [[nodiscard]] auto atlasToView(Common::Vec2i xy) const noexcept -> Common::Vec2i;
  [[nodiscard]] auto atlasToViewTransform() const noexcept -> Common::Mat3x3i;
  [[nodiscard]] static auto atlasToView(Common::Vec2i xy, const Common::Mat3x3i &m) noexcept
      -> Common::Vec2i;

  // Pixel position conversion from view (u, v) to atlas (x, y)
  //
  //  * Forms mapping with atlasToView when lodX == 1 and lodY == 1
  //  * For lodX > 1 or lodY > 1 only the transform is available
  //  * Although compilers may be smart enough, computing the transform first may be faster
  [[nodiscard]] auto viewToAtlas(Common::Vec2i uv) const noexcept -> Common::Vec2i;
  [[nodiscard]] auto viewToAtlasTransform() const noexcept -> Common::Mat3x3i;
  [[nodiscard]] static auto viewToAtlas(Common::Vec2i uv, const Common::Mat3x3i &m) noexcept
      -> Common::Vec2i;

  static auto decodePdu(const PatchDataUnit &pdu, const AtlasSequenceParameterSetRBSP &asps,
                        const AtlasFrameParameterSetRBSP &afps, const AtlasTileHeader &ath)
      -> PatchParams;
  auto encodePdu(const AtlasSequenceParameterSetRBSP &asps, const AtlasFrameParameterSetRBSP &afps,
                 const AtlasTileHeader &ath) const -> PatchDataUnit;

  auto operator==(const PatchParams &other) const -> bool;
  auto operator!=(const PatchParams &other) const -> bool { return !operator==(other); };

private:
  std::int32_t m_atlasPatch2dPosX{};
  std::int32_t m_atlasPatch2dPosY{};
  std::int32_t m_atlasPatch2dSizeX{};
  std::int32_t m_atlasPatch2dSizeY{};
  std::int32_t m_atlasPatch3dOffsetU{};
  std::int32_t m_atlasPatch3dOffsetV{};
  std::int32_t m_atlasPatch3dOffsetD{};
  std::int32_t m_atlasPatch3dRangeD{};
  std::uint16_t m_atlasPatchProjectionId{};
  std::int32_t m_atlasPatchLoDScaleX{1};
  std::int32_t m_atlasPatchLoDScaleY{1};
  FlexiblePatchOrientation m_atlasPatchOrientationIndex{FlexiblePatchOrientation::FPO_INVALID};
  std::optional<std::uint16_t> m_atlasPatchEntityId;
  std::optional<std::uint32_t> m_atlasPatchDepthOccMapThreshold;
  std::optional<Common::Vec3w> m_atlasPatchAttributeOffset{};
  bool m_atlasPatchInpaintFlag{};
};

using PatchParamsList = std::vector<PatchParams>;
} // namespace TMIV::MivBitstream

#include "PatchParamsList.hpp"

#endif
