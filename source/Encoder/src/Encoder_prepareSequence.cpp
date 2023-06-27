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

#include <TMIV/Common/verify.h>

#include <cassert>
#include <iostream>

namespace TMIV::Encoder {
namespace {
void runtimeCheck(bool cond, const char *what) {
  if (!cond) {
    throw std::runtime_error(what);
  }
}
} // namespace

void Encoder::prepareSequence(MivBitstream::EncoderParams sourceParams) {
  m_config.blockSize = m_config.blockSizeDepthQualityDependent[static_cast<std::size_t>(
      sourceParams.casme().casme_depth_low_quality_flag())];
  runtimeCheck(2 <= m_config.blockSize, "blockSize should be at least two");
  runtimeCheck((m_config.blockSize & (m_config.blockSize - 1)) == 0,
               "blockSize should be a power of two");

  // TODO(BK): To account for the occupancy maps, the scaling factor needs to be known before this
  // point.
  const auto lumaSamplesPerAtlasSample =
      (m_config.haveTexture ? 1. : 0.) +
      (m_config.haveGeometry ? (m_config.geometryScaleEnabledFlag ? 0.25 : 1.) : 0.);
  const auto numGroups =
      std::max(1.0F, static_cast<float>(sourceParams.vme().group_mapping().gm_group_count()));
  m_config.maxBlockRate = m_config.maxLumaSampleRate /
                          (numGroups * lumaSamplesPerAtlasSample * Common::sqr(m_config.blockSize));
  m_config.maxBlocksPerAtlas = m_config.maxLumaPictureSize / Common::sqr(m_config.blockSize);

  // gcc-9 and gcc-10 give a false alarm here, so suppress that warning on this one line
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif // __GNUC__

  std::optional<MivBitstream::ViewingSpace> viewingSpace = sourceParams.viewingSpace;

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif // __GNUC__

  // Transform source to transport view sequence parameters
  m_transportParams = m_viewOptimizer->optimizeParams(std::move(sourceParams));

  // Calculate nominal atlas frame sizes
  const auto atlasFrameSizes = calculateNominalAtlasFrameSizes(m_transportParams);
  std::cout << "Nominal atlas frame sizes: { ";
  for (const auto &size : atlasFrameSizes) {
    std::cout << ' ' << size;
  }
  std::cout << " }\n";

  // Create IVS with VPS with right number of atlases but copy other parts from input IVS
  m_params = MivBitstream::EncoderParams{atlasFrameSizes, m_config.haveTexture,
                                         m_config.haveGeometry, m_config.haveOccupancy};

  m_params.vme() = m_transportParams.vme();
  m_params.casme() = m_transportParams.casme();

  if (0 < m_params.vme().group_mapping().gm_group_count()) {
    // Group atlases together to restrict atlas-level sub-bitstream access
    for (std::size_t i = 0; i < atlasFrameSizes.size(); ++i) {
      m_params.vme().group_mapping().gm_group_id(i, 0);
    }
  }

  m_params.viewParamsList = m_transportParams.viewParamsList;
  m_params.frameRate = m_transportParams.frameRate;
  m_params.lengthsInMeters = m_transportParams.lengthsInMeters;
  m_params.maxEntityId = m_transportParams.maxEntityId;
  m_params.casps.casps_extension_present_flag(true)
      .casps_miv_extension_present_flag(true)
      .casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4())
      .casps_miv_extension()
      .casme_depth_quantization_params_present_flag(m_transportParams.dqParamsPresentFlag)
      .casme_vui_params_present_flag(true)
      .vui_parameters(vuiParameters());
  m_params.viewingSpace = std::move(viewingSpace);

  m_params.randomAccess = m_transportParams.randomAccess;

  setGiGeometry3dCoordinatesBitdepthMinus1();

  // Register pruning relation
  m_pruner->prepareSequence(m_params);

  // Turn on occupancy coding per view
  enableOccupancyPerView();

  // Set-up ASPS and AFPS
  prepareIvau();
}

// Calculate atlas frame sizes [MPEG/M52994 v2]
auto Encoder::calculateNominalAtlasFrameSizes(const MivBitstream::EncoderParams &params) const
    -> Common::SizeVector {
  if (m_config.oneViewPerAtlasFlag) {
    // No constraints: one atlas per transport view
    auto result = Common::SizeVector(params.viewParamsList.size());
    std::transform(std::cbegin(params.viewParamsList), std::cend(params.viewParamsList),
                   std::begin(result),
                   [](const MivBitstream::ViewParams &x) { return x.ci.projectionPlaneSize(); });
    return result;
  }

  if (!m_config.overrideAtlasFrameSizes.empty()) {
    std::cout
        << "WARNING: When overriding nominal atlas frame sizes, constraints are not checked.\n";
    return m_config.overrideAtlasFrameSizes;
  }

  // Translate block rate into a maximum number of blocks
  const auto maxBlocks = static_cast<int>(m_config.maxBlockRate / params.frameRate);

  // Calculate the number of atlases
  auto numAtlases = (maxBlocks + m_config.maxBlocksPerAtlas - 1) / m_config.maxBlocksPerAtlas;
  if (numAtlases > m_config.maxAtlases) {
    std::cout << "The maxAtlases constraint is a limiting factor.\n";
    numAtlases = m_config.maxAtlases;
  }

  // Calculate the number of blocks per atlas
  auto maxBlocksPerAtlas = maxBlocks / numAtlases;
  if (maxBlocksPerAtlas > m_config.maxBlocksPerAtlas) {
    std::cout << "The maxLumaPictureSize constraint is a limiting factor.\n";
    maxBlocksPerAtlas = m_config.maxBlocksPerAtlas;
  }

  // Take the smallest reasonable width
  const auto viewGridSize = calculateViewGridSize(params);
  const auto atlasGridWidth = viewGridSize.x();
  const auto atlasGridHeight = maxBlocksPerAtlas / atlasGridWidth;

  // Warn if the aspect ratio is outside of HEVC limits (unlikely)
  if (atlasGridWidth * 8 < atlasGridHeight || atlasGridHeight * 8 < atlasGridWidth) {
    std::cout << "WARNING: Atlas aspect ratio is outside of HEVC general tier and level limits\n";
  }

  return Common::SizeVector(
      numAtlases, {atlasGridWidth * m_config.blockSize, atlasGridHeight * m_config.blockSize});
}

auto Encoder::calculateViewGridSize(const MivBitstream::EncoderParams &params) const
    -> Common::Vec2i {
  int x{};
  int y{};

  for (const auto &viewParams : params.viewParamsList) {
    x = std::max(x, (viewParams.ci.ci_projection_plane_width_minus1() + m_config.blockSize) /
                        m_config.blockSize);
    y = std::max(y, (viewParams.ci.ci_projection_plane_height_minus1() + m_config.blockSize) /
                        m_config.blockSize);
  }

  return {x, y};
}

auto Encoder::vuiParameters() const -> MivBitstream::VuiParameters {
  auto numUnitsInTick = 1;
  auto timeScale = static_cast<int>(numUnitsInTick * m_params.frameRate);
  LIMITATION(timeScale == numUnitsInTick * m_params.frameRate);

  auto vui = MivBitstream::VuiParameters{};
  vui.vui_timing_info_present_flag(true)
      .vui_num_units_in_tick(numUnitsInTick)
      .vui_time_scale(timeScale)
      .vui_poc_proportional_to_timing_flag(false)
      .vui_hrd_parameters_present_flag(false);
  vui.vui_unit_in_metres_flag(m_params.lengthsInMeters);
  vui.vui_coordinate_system_parameters_present_flag(true).coordinate_system_parameters() = {};
  return vui;
}

void Encoder::setGiGeometry3dCoordinatesBitdepthMinus1() {
  uint8_t numBitsMinus1 = 9; // Main 10
  for (auto &vp : m_params.viewParamsList) {
    const auto size = std::max(vp.ci.ci_projection_plane_width_minus1() + 1,
                               vp.ci.ci_projection_plane_height_minus1() + 1);
    numBitsMinus1 = std::max(numBitsMinus1, static_cast<uint8_t>(Common::ceilLog2(size) - 1));
  }
  for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_params.vps.vps_atlas_id(k);
    if (m_params.vps.vps_geometry_video_present_flag(j)) {
      m_params.vps.geometry_information(j).gi_geometry_3d_coordinates_bit_depth_minus1(
          numBitsMinus1);
    }
  }
}

void Encoder::enableOccupancyPerView() {
  for (size_t viewId = 0; viewId < m_params.viewParamsList.size(); ++viewId) {
    if (!m_params.viewParamsList[viewId].isBasicView || 0 < m_params.maxEntityId) {
      m_params.viewParamsList[viewId].hasOccupancy = true;
    }
  }
}

void Encoder::prepareIvau() {
  m_params.atlas.resize(m_params.vps.vps_atlas_count_minus1() + size_t{1});

  for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    auto &atlas = m_params.atlas[k];
    const auto j = m_params.vps.vps_atlas_id(k);

    // Set ASPS parameters
    atlas.asps.asps_frame_width(m_params.vps.vps_frame_width(j))
        .asps_frame_height(m_params.vps.vps_frame_height(j))
        .asps_log2_max_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4())
        .asps_use_eight_orientations_flag(true)
        .asps_extended_projection_enabled_flag(true)
        .asps_normal_axis_limits_quantization_enabled_flag(true)
        .asps_max_number_projections_minus1(
            static_cast<uint16_t>(m_params.viewParamsList.size() - 1))
        .asps_log2_patch_packing_block_size(Common::ceilLog2(m_config.blockSize));

    atlas.asme().asme_max_entity_id(m_params.maxEntityId);

    const auto psq = patchSizeQuantizers();
    atlas.asps.asps_patch_size_quantizer_present_flag(psq.x() != m_config.blockSize ||
                                                      psq.y() != m_config.blockSize);

    if (m_params.vps.vps_geometry_video_present_flag(j)) {
      const auto &gi = m_params.vps.geometry_information(j);
      atlas.asps.asps_geometry_3d_bit_depth_minus1(gi.gi_geometry_3d_coordinates_bit_depth_minus1())
          .asps_geometry_2d_bit_depth_minus1(gi.gi_geometry_2d_bit_depth_minus1());
    }

    // Signalling pdu_entity_id requires ASME to be present
    if (m_params.vps.vps_miv_extension_present_flag() && 0 < m_params.maxEntityId) {
      // There is nothing entity-related in ASME so a reference is obtained but discarded
      static_cast<void>(atlas.asme());
    }

    // Set ATH parameters
    atlas.ath.ath_ref_atlas_frame_list_asps_flag(true);
    if (atlas.asps.asps_patch_size_quantizer_present_flag()) {
      atlas.ath.ath_patch_size_x_info_quantizer(Common::ceilLog2(psq.x()));
      atlas.ath.ath_patch_size_y_info_quantizer(Common::ceilLog2(psq.y()));
    }
    atlas.ath.ath_pos_min_d_quantizer( // make pdu_3d_offset_d u(0)
        atlas.asps.asps_geometry_3d_bit_depth_minus1() + 1);
  }
}

auto Encoder::log2FocLsbMinus4() const -> std::uint8_t {
  // Avoid confusion but test MSB/LSB logic in decoder
  return std::max(4U, Common::ceilLog2(m_config.intraPeriod) + 1U) - 4U;
}

auto Encoder::patchSizeQuantizers() const -> Common::Vec2i {
  auto quantizer = m_config.blockSize;

  for (const auto &vp : m_params.viewParamsList) {
    quantizer = std::gcd(quantizer, vp.ci.ci_projection_plane_width_minus1() + 1);
    quantizer = std::gcd(quantizer, vp.ci.ci_projection_plane_height_minus1() + 1);
  }

  // NOTE(BK): There may be rotated patches of full width or height: same quantizer for x and y
  return {quantizer, quantizer};
}
} // namespace TMIV::Encoder
