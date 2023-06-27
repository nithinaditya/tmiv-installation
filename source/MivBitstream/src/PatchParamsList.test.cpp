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

#include <catch2/catch.hpp>

#include <TMIV/Common/LinAlg.h>
#include <TMIV/Common/Matrix.h>

#include <TMIV/MivBitstream/PatchParamsList.h>

using Mat2x2d = TMIV::Common::stack::Matrix<double, 2, 2>;
using TMIV::Common::Mat2x2i;
using TMIV::Common::Vec2d;
using TMIV::Common::Vec2i;
using TMIV::Common::Vec3i;
using TMIV::Common::Vec3w;
using TMIV::MivBitstream::AtlasFrameParameterSetRBSP;
using TMIV::MivBitstream::AtlasSequenceParameterSetRBSP;
using TMIV::MivBitstream::AtlasTileHeader;
using TMIV::MivBitstream::FlexiblePatchOrientation;
using TMIV::MivBitstream::PatchDataUnit;
using TMIV::MivBitstream::PatchParams;
// To compare against the efficient implementation, we directly implement V3C 2E
// [ISO/IEC JTC 1/SC 29/WG 07 N 0003 clause 9.5.2.1]
namespace direct {
// [ISO/IEC JTC 1/SC 29/WG 07 N 0003 Table 11]
auto matrixRo(FlexiblePatchOrientation fpo) noexcept -> Mat2x2i {
  switch (fpo) {
  case FlexiblePatchOrientation::FPO_NULL:
    return {1, 0, 0, 1};
  case FlexiblePatchOrientation::FPO_SWAP:
    return {0, 1, 1, 0};
  case FlexiblePatchOrientation::FPO_ROT90:
    // NOTE(#436): Fix to be included in V3C 2E DIS
    return {0, 1, -1, 0};
  case FlexiblePatchOrientation::FPO_ROT180:
    return {-1, 0, 0, -1};
  case FlexiblePatchOrientation::FPO_ROT270:
    // NOTE(#436): Fix to be included in V3C 2E DIS
    return {0, -1, 1, 0};
  case FlexiblePatchOrientation::FPO_MIRROR:
    return {-1, 0, 0, 1};
  case FlexiblePatchOrientation::FPO_MROT90:
    return {0, -1, -1, 0};
  case FlexiblePatchOrientation::FPO_MROT180:
    return {1, 0, 0, -1};
  default:
    abort();
  }
}

// [ISO/IEC JTC 1/SC 29/WG 07 N 0003 Table 11]
auto matrixRs(FlexiblePatchOrientation fpo) noexcept -> Mat2x2i {
  switch (fpo) {
  case FlexiblePatchOrientation::FPO_NULL:
  case FlexiblePatchOrientation::FPO_SWAP:
    return {0, 0, 0, 0};
  case FlexiblePatchOrientation::FPO_ROT90:
    return {0, 0, 1, 0};
  case FlexiblePatchOrientation::FPO_ROT180:
    return {1, 0, 0, 1};
  case FlexiblePatchOrientation::FPO_ROT270:
    return {0, 1, 0, 0};
  case FlexiblePatchOrientation::FPO_MIRROR:
    return {1, 0, 0, 0};
  case FlexiblePatchOrientation::FPO_MROT90:
    return {0, 1, 1, 0};
  case FlexiblePatchOrientation::FPO_MROT180:
    return {0, 0, 0, 1};
  default:
    abort();
  }
}

// Forward function [WG 07 N 0003 Eq. (49)]
auto atlasToView(const PatchParams &pp, Vec2i xy) -> Vec2i {
  const auto pos = Vec2i{pp.atlasPatch2dPosX(), pp.atlasPatch2dPosY()};
  const auto size = Vec2i{pp.atlasPatch2dSizeX(), pp.atlasPatch2dSizeY()};
  const auto lod = Mat2x2i{pp.atlasPatchLoDScaleX(), 0, 0, pp.atlasPatchLoDScaleY()};
  const auto Ro = matrixRo(pp.atlasPatchOrientationIndex());
  const auto Rs = matrixRs(pp.atlasPatchOrientationIndex());
  const auto offset = Vec2i{pp.atlasPatch3dOffsetU(), pp.atlasPatch3dOffsetV()};

  return offset + Ro * lod * (xy - pos) + Rs * lod * (size - Vec2i{1, 1});
}
} // namespace direct

TEST_CASE("TMIV::MivBitstream::PatchParams") {
  SECTION("Decode patch data unit") {
    SECTION("Minimal example") {
      const auto unit = PatchParams::decodePdu({}, {}, {}, {});

      REQUIRE(unit.atlasPatch2dPosX() == 0);
      REQUIRE(unit.atlasPatch2dPosY() == 0);
      REQUIRE(unit.atlasPatch2dSizeX() == 1);
      REQUIRE(unit.atlasPatch2dSizeY() == 1);
      REQUIRE(unit.atlasPatch3dOffsetU() == 0);
      REQUIRE(unit.atlasPatch3dOffsetV() == 0);
      REQUIRE(unit.atlasPatch3dOffsetD() == 0);
      REQUIRE(unit.atlasPatch3dRangeD() == 1);
      REQUIRE(unit.atlasPatchProjectionId() == 0);
      REQUIRE(unit.atlasPatchOrientationIndex() == FlexiblePatchOrientation::FPO_NULL);
      REQUIRE(unit.atlasPatchLoDScaleX() == 1);
      REQUIRE(unit.atlasPatchLoDScaleY() == 1);
      REQUIRE(unit.atlasPatchEntityId() == std::nullopt);
      REQUIRE(unit.atlasPatchDepthOccMapThreshold() == std::nullopt);
      REQUIRE_THROWS(unit.atlasPatchAttributeOffset());
      REQUIRE(!unit.atlasPatchInpaintFlag());
    }

    SECTION("Typical example") {
      auto pdu = PatchDataUnit{};
      pdu.pdu_2d_pos_x(1)
          .pdu_2d_pos_y(2)
          .pdu_2d_size_x_minus1(3)
          .pdu_2d_size_y_minus1(4)
          .pdu_3d_offset_u(5)
          .pdu_3d_offset_v(6)
          .pdu_projection_id(7)
          .pdu_orientation_index(FlexiblePatchOrientation::FPO_ROT270)
          .pdu_lod_enabled_flag(true)
          .pdu_lod_scale_x_minus1(0)
          .pdu_lod_scale_y_idc(1);

      auto asps = AtlasSequenceParameterSetRBSP{};
      asps.asps_log2_patch_packing_block_size(3)
          .asps_geometry_2d_bit_depth_minus1(4)
          .asps_geometry_3d_bit_depth_minus1(5);

      const auto unit = PatchParams::decodePdu(pdu, asps, {}, {});

      REQUIRE(unit.atlasPatch2dPosX() == 8);
      REQUIRE(unit.atlasPatch2dPosY() == 16);
      REQUIRE(unit.atlasPatch2dSizeX() == 32);
      REQUIRE(unit.atlasPatch2dSizeY() == 40);
      REQUIRE(unit.atlasPatch3dOffsetU() == 5);
      REQUIRE(unit.atlasPatch3dOffsetV() == 6);
      REQUIRE(unit.atlasPatch3dOffsetD() == 0);
      REQUIRE(unit.atlasPatch3dRangeD() == 31);
      REQUIRE(unit.atlasPatchProjectionId() == 7);
      REQUIRE(unit.atlasPatchOrientationIndex() == FlexiblePatchOrientation::FPO_ROT270);
      REQUIRE(unit.atlasPatchLoDScaleX() == 1);
      REQUIRE(unit.atlasPatchLoDScaleY() == 3);
      REQUIRE(unit.atlasPatchEntityId() == std::nullopt);
      REQUIRE(unit.atlasPatchDepthOccMapThreshold() == std::nullopt);
      REQUIRE_THROWS(unit.atlasPatchAttributeOffset());
      REQUIRE(!unit.atlasPatchInpaintFlag());
    }

    SECTION("Elaborate example") {
      auto pdu = PatchDataUnit{};
      pdu.pdu_2d_pos_x(1)
          .pdu_2d_pos_y(2)
          .pdu_2d_size_x_minus1(3)
          .pdu_2d_size_y_minus1(4)
          .pdu_3d_offset_u(5)
          .pdu_3d_offset_v(6)
          .pdu_3d_range_d(7)
          .pdu_projection_id(8)
          .pdu_orientation_index(FlexiblePatchOrientation::FPO_ROT270)
          .pdu_lod_enabled_flag(true)
          .pdu_lod_scale_x_minus1(2)
          .pdu_lod_scale_y_idc(0)
          .pdu_miv_extension()
          .pdu_depth_occ_threshold(9)
          .pdu_attribute_offset(Vec3w{10, 11, 12})
          .pdu_entity_id(13);

      auto asps = AtlasSequenceParameterSetRBSP{};
      asps.asps_log2_patch_packing_block_size(3)
          .asps_geometry_2d_bit_depth_minus1(4)
          .asps_geometry_3d_bit_depth_minus1(5)
          .asps_normal_axis_max_delta_value_enabled_flag(true)
          .asps_patch_size_quantizer_present_flag(true)
          .asps_extension_present_flag(true)
          .asps_miv_extension_present_flag(true)
          .asps_miv_extension()
          .asme_embedded_occupancy_enabled_flag(true)
          .asme_depth_occ_threshold_flag(true)
          .asme_patch_attribute_offset_enabled_flag(true);

      auto ath = AtlasTileHeader();
      ath.ath_pos_min_d_quantizer(6)
          .ath_pos_delta_max_d_quantizer(7)
          .ath_patch_size_x_info_quantizer(2)
          .ath_patch_size_y_info_quantizer(1);

      const auto unit = PatchParams::decodePdu(pdu, asps, {}, ath);

      REQUIRE(unit.atlasPatch2dPosX() == 8);
      REQUIRE(unit.atlasPatch2dPosY() == 16);
      REQUIRE(unit.atlasPatch2dSizeX() == 16);
      REQUIRE(unit.atlasPatch2dSizeY() == 10);
      REQUIRE(unit.atlasPatch3dOffsetU() == 5);
      REQUIRE(unit.atlasPatch3dOffsetV() == 6);
      REQUIRE(unit.atlasPatch3dOffsetD() == 0);
      REQUIRE(unit.atlasPatch3dRangeD() == 895);
      REQUIRE(unit.atlasPatchProjectionId() == 8);
      REQUIRE(unit.atlasPatchOrientationIndex() == FlexiblePatchOrientation::FPO_ROT270);
      REQUIRE(unit.atlasPatchLoDScaleX() == 3);
      REQUIRE(unit.atlasPatchLoDScaleY() == 1);
      REQUIRE(unit.atlasPatchEntityId() == 13);
      REQUIRE(unit.atlasPatchDepthOccMapThreshold() == 9);
      REQUIRE(unit.atlasPatchAttributeOffset() == Vec3w{10, 11, 12});
      REQUIRE(!unit.atlasPatchInpaintFlag());
    }
  }

  SECTION("Inpaint patch") {
    auto pdu = PatchDataUnit{};
    pdu.pdu_miv_extension().pdu_inpaint_flag(true);

    auto asps = AtlasSequenceParameterSetRBSP{};
    asps.asps_extension_present_flag(true)
        .asps_miv_extension_present_flag(true)
        .asps_miv_extension()
        .asme_inpaint_enabled_flag(true);

    auto afps = AtlasFrameParameterSetRBSP{};
    afps.afps_extension_present_flag(true)
        .afps_miv_extension_present_flag(true)
        .afps_miv_extension()
        .afme_inpaint_lod_enabled_flag(true)
        .afme_inpaint_lod_scale_x_minus1(3)
        .afme_inpaint_lod_scale_y_idc(3);

    const auto unit = PatchParams::decodePdu(pdu, asps, afps, {});

    REQUIRE(unit.atlasPatchLoDScaleX() == 4);
    REQUIRE(unit.atlasPatchLoDScaleY() == 4);
    REQUIRE(unit.atlasPatchInpaintFlag());
  }

  SECTION("Encode patch data unit") {
    SECTION("Minimal example") {
      auto unit = PatchParams{};
      unit.atlasPatch2dSizeX(1).atlasPatch2dSizeY(1).atlasPatchOrientationIndex(
          FlexiblePatchOrientation::FPO_NULL);

      const auto pdu = unit.encodePdu({}, {}, {});

      REQUIRE(pdu.pdu_2d_pos_x() == 0);
      REQUIRE(pdu.pdu_2d_pos_y() == 0);
      REQUIRE(pdu.pdu_2d_size_x_minus1() == 0);
      REQUIRE(pdu.pdu_2d_size_y_minus1() == 0);
      REQUIRE(pdu.pdu_3d_offset_u() == 0);
      REQUIRE(pdu.pdu_3d_offset_v() == 0);
      REQUIRE(pdu.pdu_3d_offset_d() == 0);
      REQUIRE_THROWS(pdu.pdu_3d_range_d());
      REQUIRE(pdu.pdu_projection_id() == 0);
      REQUIRE(pdu.pdu_orientation_index() == FlexiblePatchOrientation::FPO_NULL);
      REQUIRE(!pdu.pdu_lod_enabled_flag());
      REQUIRE(pdu.pdu_lod_scale_x_minus1() == 0);
      REQUIRE(pdu.pdu_lod_scale_y_idc() == 0);
      REQUIRE(pdu.pdu_miv_extension().pdu_attribute_offset() == Vec3w{});
      REQUIRE(pdu.pdu_miv_extension().pdu_entity_id() == 0);
      REQUIRE_THROWS(pdu.pdu_miv_extension().pdu_depth_occ_threshold());
    }

    SECTION("Typical example") {
      auto asps = AtlasSequenceParameterSetRBSP{};
      asps.asps_log2_patch_packing_block_size(3)
          .asps_geometry_2d_bit_depth_minus1(4)
          .asps_geometry_3d_bit_depth_minus1(5);

      auto afps = AtlasFrameParameterSetRBSP{};
      afps.afps_lod_mode_enabled_flag(true);

      auto unit = PatchParams{};
      unit.atlasPatch2dPosX(8)
          .atlasPatch2dPosY(16)
          .atlasPatch2dSizeX(32)
          .atlasPatch2dSizeY(40)
          .atlasPatch3dOffsetU(5)
          .atlasPatch3dOffsetV(6)
          .atlasPatch3dRangeD(31)
          .atlasPatchProjectionId(7)
          .atlasPatchLoDScaleX(1)
          .atlasPatchLoDScaleY(3)
          .atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_ROT270);

      const auto pdu = unit.encodePdu(asps, afps, {});

      REQUIRE(pdu.pdu_2d_pos_x() == 1);
      REQUIRE(pdu.pdu_2d_pos_y() == 2);
      REQUIRE(pdu.pdu_2d_size_x_minus1() == 3);
      REQUIRE(pdu.pdu_2d_size_y_minus1() == 4);
      REQUIRE(pdu.pdu_3d_offset_u() == 5);
      REQUIRE(pdu.pdu_3d_offset_v() == 6);
      REQUIRE(pdu.pdu_projection_id() == 7);
      REQUIRE(pdu.pdu_orientation_index() == FlexiblePatchOrientation::FPO_ROT270);
      REQUIRE(pdu.pdu_lod_enabled_flag());
      REQUIRE(pdu.pdu_lod_scale_x_minus1() == 0);
      REQUIRE(pdu.pdu_lod_scale_y_idc() == 1);
    }

    SECTION("Elaborate example") {
      auto unit = PatchParams{};
      unit.atlasPatch2dPosX(8)
          .atlasPatch2dPosY(16)
          .atlasPatch2dSizeX(16)
          .atlasPatch2dSizeY(10)
          .atlasPatch3dOffsetU(5)
          .atlasPatch3dOffsetV(6)
          .atlasPatch3dOffsetD(0)
          .atlasPatch3dRangeD(895)
          .atlasPatchProjectionId(8)
          .atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_ROT270)
          .atlasPatchLoDScaleX(3)
          .atlasPatchLoDScaleY(1)
          .atlasPatchEntityId(13)
          .atlasPatchDepthOccMapThreshold(9)
          .atlasPatchAttributeOffset(Vec3w{10, 11, 12});

      auto asps = AtlasSequenceParameterSetRBSP{};
      asps.asps_log2_patch_packing_block_size(3)
          .asps_geometry_2d_bit_depth_minus1(4)
          .asps_geometry_3d_bit_depth_minus1(5)
          .asps_normal_axis_max_delta_value_enabled_flag(true)
          .asps_patch_size_quantizer_present_flag(true)
          .asps_extension_present_flag(true)
          .asps_miv_extension_present_flag(true)
          .asps_miv_extension()
          .asme_embedded_occupancy_enabled_flag(true)
          .asme_depth_occ_threshold_flag(true)
          .asme_patch_attribute_offset_enabled_flag(true);

      auto afps = AtlasFrameParameterSetRBSP{};
      afps.afps_lod_mode_enabled_flag(true);

      auto ath = AtlasTileHeader();
      ath.ath_pos_min_d_quantizer(6)
          .ath_pos_delta_max_d_quantizer(7)
          .ath_patch_size_x_info_quantizer(2)
          .ath_patch_size_y_info_quantizer(1);

      const auto pdu = unit.encodePdu(asps, afps, ath);

      REQUIRE(pdu.pdu_2d_pos_x() == 1);
      REQUIRE(pdu.pdu_2d_pos_y() == 2);
      REQUIRE(pdu.pdu_2d_size_x_minus1() == 3);
      REQUIRE(pdu.pdu_2d_size_y_minus1() == 4);
      REQUIRE(pdu.pdu_3d_offset_u() == 5);
      REQUIRE(pdu.pdu_3d_offset_v() == 6);
      REQUIRE(pdu.pdu_3d_range_d() == 7);
      REQUIRE(pdu.pdu_projection_id() == 8);
      REQUIRE(pdu.pdu_orientation_index() == FlexiblePatchOrientation::FPO_ROT270);
      REQUIRE(pdu.pdu_lod_enabled_flag());
      REQUIRE(pdu.pdu_lod_scale_x_minus1() == 2);
      REQUIRE(pdu.pdu_lod_scale_y_idc() == 0);
      REQUIRE(pdu.pdu_miv_extension().pdu_depth_occ_threshold() == 9);
      REQUIRE(pdu.pdu_miv_extension().pdu_attribute_offset() == Vec3w{10, 11, 12});
      REQUIRE(pdu.pdu_miv_extension().pdu_entity_id() == 13);
    }

    SECTION("Inpaint patch") {
      auto unit = PatchParams{};
      unit.atlasPatchLoDScaleX(4).atlasPatchLoDScaleY(4).atlasPatchInpaintFlag(true);

      auto asps = AtlasSequenceParameterSetRBSP{};
      asps.asps_extension_present_flag(true)
          .asps_miv_extension_present_flag(true)
          .asps_miv_extension()
          .asme_inpaint_enabled_flag(true);

      auto afps = AtlasFrameParameterSetRBSP{};
      afps.afps_extension_present_flag(true)
          .afps_miv_extension_present_flag(true)
          .afps_miv_extension()
          .afme_inpaint_lod_enabled_flag(true)
          .afme_inpaint_lod_scale_x_minus1(3)
          .afme_inpaint_lod_scale_y_idc(3);

      const auto pdu = unit.encodePdu(asps, afps, {});

      REQUIRE(pdu.pdu_miv_extension().pdu_inpaint_flag());
    }
  }

  SECTION("View-atlas coordinate transformations") {
    const int32_t posX = GENERATE(8, 19);
    const int32_t posY = GENERATE(3, 16);
    const int32_t sizeX = GENERATE(16, 25);
    const int32_t sizeY = GENERATE(10, 23);
    const int32_t offsetU = 5;
    const int32_t offsetV = 45;
    const auto orientation =
        GENERATE(FlexiblePatchOrientation::FPO_NULL, FlexiblePatchOrientation::FPO_SWAP,
                 FlexiblePatchOrientation::FPO_ROT90, FlexiblePatchOrientation::FPO_ROT180,
                 FlexiblePatchOrientation::FPO_ROT270, FlexiblePatchOrientation::FPO_MIRROR,
                 FlexiblePatchOrientation::FPO_MROT90, FlexiblePatchOrientation::FPO_MROT180);

    CAPTURE(posX, posY, sizeX, sizeY, offsetU, offsetV, orientation);

    auto unit = PatchParams{};
    unit.atlasPatch2dPosX(posX)
        .atlasPatch2dPosY(posY)
        .atlasPatch2dSizeX(sizeX)
        .atlasPatch2dSizeY(sizeY)
        .atlasPatch3dOffsetU(offsetU)
        .atlasPatch3dOffsetV(offsetV)
        .atlasPatchOrientationIndex(orientation);

    SECTION("Transformation matrices") {
      const auto lodX = GENERATE(1, 2, 3);
      const auto lodY = GENERATE(1, 2, 5);

      unit.atlasPatchLoDScaleX(lodX).atlasPatchLoDScaleY(lodY);

      const auto M = unit.atlasToViewTransform();
      const auto invM = unit.viewToAtlasTransform();
      const auto d = std::lcm(lodX, lodY);

      REQUIRE(invM(2, 2) == d);
      REQUIRE(M * invM == d * TMIV::Common::Mat3x3i::eye());
    }

    SECTION("View to atlas transformation") {
      const auto u = GENERATE(13, 78);
      const auto v = GENERATE(16, 17);
      CAPTURE(u, v);
      const auto xy = unit.viewToAtlas({u, v});
      REQUIRE(direct::atlasToView(unit, xy) == Vec2i{u, v});
    }

    SECTION("Atlas to view transformation") {
      const auto x = GENERATE(13, 78);
      const auto y = GENERATE(16, 17);
      CAPTURE(x, y);
      const auto actual = unit.atlasToView({x, y});
      const auto reference = direct::atlasToView(unit, {x, y});
      REQUIRE(actual == reference);

      SECTION("Level of Detail") {
        unit.atlasPatchLoDScaleX(2).atlasPatchLoDScaleY(3);
        const auto actual = unit.atlasToView({x, y});
        const auto reference = direct::atlasToView(unit, {x, y});
        REQUIRE(actual == reference);
      }
    }
  }
}
