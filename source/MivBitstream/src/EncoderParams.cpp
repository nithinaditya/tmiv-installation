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

#include <TMIV/MivBitstream/EncoderParams.h>

#include <TMIV/Common/verify.h>

#include <algorithm>

namespace TMIV::MivBitstream {
namespace {
auto getPtlProfilePccToolsetIdc(bool haveGeometryVideo, bool haveOccupancyVideo,
                                bool haveTransparencyVideo) -> PtlProfilePccToolsetIdc {
  if (haveTransparencyVideo) {
    return PtlProfilePccToolsetIdc::MIV_Extended;
  }
  if (!haveGeometryVideo) {
    return PtlProfilePccToolsetIdc::MIV_Geometry_Absent;
  }
  if (!haveOccupancyVideo) {
    return PtlProfilePccToolsetIdc::MIV_Main;
  }
  return PtlProfilePccToolsetIdc::MIV_Extended;
}
} // namespace

auto EncoderAtlasParams::asme() const noexcept -> const AspsMivExtension & {
  return asps.asps_miv_extension();
}

auto EncoderAtlasParams::asme() noexcept -> AspsMivExtension & {
  return asps.asps_extension_present_flag(true)
      .asps_miv_extension_present_flag(true)
      .asps_miv_extension();
}

EncoderParams::EncoderParams() : EncoderParams{false, false, false} {}

// TODO(BK): Move to Encoder class
EncoderParams::EncoderParams(bool haveTextureVideo, bool haveGeometryVideo, bool haveOccupancyVideo)
    : EncoderParams{Common::SizeVector{{0xFFFF, 0xFFFF, 0xFFFF}}, haveTextureVideo,
                    haveGeometryVideo, haveOccupancyVideo} {}

EncoderParams::EncoderParams(std::uint8_t textureBitDepth, std::uint8_t occupancyBitDepth,
                             std::uint8_t geometryBitDepth, std::uint8_t transparencyBitDepth)
    : EncoderParams{Common::SizeVector{{0xFFFF, 0xFFFF}}, textureBitDepth, occupancyBitDepth,
                    geometryBitDepth, transparencyBitDepth} {}

// TODO(BK): Move to Encoder class
EncoderParams::EncoderParams(const Common::SizeVector &atlasSizes, bool haveTextureVideo,
                             bool haveGeometryVideo, bool haveOccupancyVideo) {
  vps.profile_tier_level()
      .ptl_level_idc(PtlLevelIdc::Level_3_5)
      .ptl_profile_codec_group_idc(PtlProfileCodecGroupIdc::HEVC_Main10)
      .ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc::MIV_Main);

  vps.profile_tier_level().ptl_profile_toolset_idc(
      getPtlProfilePccToolsetIdc(haveGeometryVideo, haveOccupancyVideo, false));

  VERIFY_MIVBITSTREAM(!atlasSizes.empty());
  vps.vps_atlas_count_minus1(static_cast<uint8_t>(atlasSizes.size() - 1));

  for (uint8_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
    const auto j = AtlasId{k};
    vps.vps_atlas_id(k, j)
        .vps_frame_width(j, atlasSizes[k].x())
        .vps_frame_height(j, atlasSizes[k].y())
        .vps_geometry_video_present_flag(j, haveGeometryVideo)
        .vps_occupancy_video_present_flag(j, haveOccupancyVideo)
        .vps_attribute_video_present_flag(j, haveTextureVideo);

    if (haveGeometryVideo) {
      vps.geometry_information(j).gi_geometry_2d_bit_depth_minus1(9);
    }

    if (haveOccupancyVideo) {
      vps.occupancy_information(j)
          .oi_occupancy_codec_id(0)
          .oi_lossy_occupancy_compression_threshold(0) // set similar to V-PCC
          .oi_occupancy_2d_bit_depth_minus1(
              9) // doing binary lossless coding for now but writing as yuv420p10le files
          .oi_occupancy_MSB_align_flag(false);
    }

    if (haveTextureVideo) {
      vps.attribute_information(j)
          .ai_attribute_count(1)
          .ai_attribute_type_id(0, AiAttributeTypeId::ATTR_TEXTURE)
          .ai_attribute_dimension_minus1(0, 2)
          .ai_attribute_2d_bit_depth_minus1(0, 9);
    }
  }
}

EncoderParams::EncoderParams(const Common::SizeVector &atlasSizes, std::uint8_t textureBitDepth,
                             std::uint8_t occupancyBitDepth, std::uint8_t geometryBitDepth,
                             std::uint8_t transparencyBitDepth) {
  const bool haveTextureVideo = textureBitDepth != 0;
  const bool haveOccupancyVideo = occupancyBitDepth != 0;
  const bool haveGeometryVideo = geometryBitDepth != 0;
  const bool haveTransparencyVideo = transparencyBitDepth != 0;

  vps.profile_tier_level()
      .ptl_level_idc(PtlLevelIdc::Level_3_5)
      .ptl_profile_codec_group_idc(PtlProfileCodecGroupIdc::HEVC_Main10)
      .ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc::MIV_Main);

  vps.profile_tier_level().ptl_profile_toolset_idc(
      getPtlProfilePccToolsetIdc(haveGeometryVideo, haveOccupancyVideo, haveTransparencyVideo));

  ProfileToolsetConstraintsInformation ptci{};
  ptci.ptc_restricted_geometry_flag(haveTransparencyVideo);
  vps.profile_tier_level().ptl_toolset_constraints_present_flag(true);
  vps.profile_tier_level().ptl_profile_toolset_constraints_information(ptci);

  VERIFY_MIVBITSTREAM(!atlasSizes.empty());
  vps.vps_atlas_count_minus1(uint8_t(atlasSizes.size() - 1));

  for (size_t k = 0; k < atlasSizes.size(); ++k) {
    const auto j = AtlasId{uint8_t(k)};
    vps.vps_atlas_id(k, j)
        .vps_frame_width(j, atlasSizes[k].x())
        .vps_frame_height(j, atlasSizes[k].y())
        .vps_geometry_video_present_flag(j, haveGeometryVideo)
        .vps_occupancy_video_present_flag(j, haveOccupancyVideo)
        .vps_attribute_video_present_flag(j, haveTextureVideo || haveTransparencyVideo);

    if (haveGeometryVideo) {
      vps.geometry_information(j).gi_geometry_2d_bit_depth_minus1(geometryBitDepth - 1);
    }

    if (haveOccupancyVideo) {
      vps.occupancy_information(j)
          .oi_occupancy_codec_id(0)
          .oi_lossy_occupancy_compression_threshold(0) // set similar to V-PCC
          .oi_occupancy_2d_bit_depth_minus1(
              occupancyBitDepth -
              1) // doing binary lossless coding for now but writing as yuv420p10le files
          .oi_occupancy_MSB_align_flag(false);
    }

    uint8_t attributeCount =
        static_cast<uint8_t>(haveTextureVideo) + static_cast<uint8_t>(haveTransparencyVideo);
    vps.attribute_information(j).ai_attribute_count(attributeCount);
    uint8_t attributeIndex{};

    if (haveTextureVideo) {
      vps.attribute_information(j)
          .ai_attribute_type_id(attributeIndex, AiAttributeTypeId::ATTR_TEXTURE)
          .ai_attribute_dimension_minus1(attributeIndex, 2)
          .ai_attribute_2d_bit_depth_minus1(attributeIndex, textureBitDepth - 1);

      ++attributeIndex;
    }

    if (haveTransparencyVideo) {
      vps.attribute_information(j)
          .ai_attribute_type_id(attributeIndex, AiAttributeTypeId::ATTR_TRANSPARENCY)
          .ai_attribute_dimension_minus1(attributeIndex, 0)
          .ai_attribute_2d_bit_depth_minus1(attributeIndex, transparencyBitDepth - 1);

      ++attributeIndex;
    }
  }
}

auto EncoderParams::casme() const noexcept -> const CaspsMivExtension & {
  return casps.casps_miv_extension();
}

auto EncoderParams::casme() noexcept -> CaspsMivExtension & {
  return casps.casps_extension_present_flag(true)
      .casps_miv_extension_present_flag(true)
      .casps_miv_extension();
}

auto EncoderParams::vme() const noexcept -> const VpsMivExtension & {
  return vps.vps_miv_extension();
}

auto EncoderParams::vme() noexcept -> VpsMivExtension & {
  return vps.vps_extension_present_flag(true)
      .vps_miv_extension_present_flag(true)
      .vps_miv_extension();
}

auto EncoderParams::operator==(const EncoderParams &other) const -> bool {
  return vps == other.vps && viewingSpace == other.viewingSpace &&
         viewParamsList == other.viewParamsList && atlas == other.atlas &&
         patchParamsList == other.patchParamsList;
}

auto EncoderParams::atlasSizes() const -> Common::SizeVector {
  auto x = Common::SizeVector{};
  x.reserve(atlas.size());

  std::transform(cbegin(atlas), cend(atlas), back_inserter(x), [](const auto &atlas) {
    return Common::Vec2i{atlas.asps.asps_frame_width(), atlas.asps.asps_frame_height()};
  });

  return x;
}
} // namespace TMIV::MivBitstream
