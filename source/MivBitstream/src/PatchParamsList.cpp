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

#include <TMIV/MivBitstream/PatchParamsList.h>

#include <TMIV/Common/verify.h>

namespace TMIV::MivBitstream {
auto PatchParams::decodePdu(const PatchDataUnit &pdu, const AtlasSequenceParameterSetRBSP &asps,
                            const AtlasFrameParameterSetRBSP &afps, const AtlasTileHeader &ath)
    -> PatchParams {
  auto pp = PatchParams{};

  const auto patchPackingBlockSize = 1U << asps.asps_log2_patch_packing_block_size();
  pp.atlasPatch2dPosX(pdu.pdu_2d_pos_x() * patchPackingBlockSize);
  pp.atlasPatch2dPosY(pdu.pdu_2d_pos_y() * patchPackingBlockSize);

  pp.atlasPatch3dOffsetU(pdu.pdu_3d_offset_u());
  pp.atlasPatch3dOffsetV(pdu.pdu_3d_offset_v());

  const auto offsetDQuantizer = 1U << ath.ath_pos_min_d_quantizer();
  pp.atlasPatch3dOffsetD(pdu.pdu_3d_offset_d() * offsetDQuantizer);

  if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
    const auto rangeDQuantizer = 1U << ath.ath_pos_delta_max_d_quantizer();
    pp.atlasPatch3dRangeD(pdu.pdu_3d_range_d() == 0 ? 0
                                                    : (pdu.pdu_3d_range_d() * rangeDQuantizer) - 1);
  } else {
    const auto rangeDBitDepth = std::min(asps.asps_geometry_2d_bit_depth_minus1() + 1U,
                                         asps.asps_geometry_3d_bit_depth_minus1() + 1U);
    const auto rangeD = 1U << rangeDBitDepth;
    pp.atlasPatch3dRangeD(rangeD - 1);
  }

  pp.atlasPatchProjectionId(pdu.pdu_projection_id());
  pp.atlasPatchOrientationIndex(pdu.pdu_orientation_index());

  if (pdu.pdu_lod_enabled_flag()) {
    pp.atlasPatchLoDScaleX(pdu.pdu_lod_scale_x_minus1() + 1U);
    pp.atlasPatchLoDScaleY((pdu.pdu_lod_scale_x_minus1() == 0 ? 2U : 1U) +
                           pdu.pdu_lod_scale_y_idc());
  } else if (afps.afps_miv_extension_present_flag()) {
    const auto &afme = afps.afps_miv_extension();
    if (afme.afme_inpaint_lod_enabled_flag()) {
      pp.atlasPatchLoDScaleX(afme.afme_inpaint_lod_scale_x_minus1() + 1U);
      pp.atlasPatchLoDScaleY((afme.afme_inpaint_lod_scale_x_minus1() == 0 ? 2U : 1U) +
                             afme.afme_inpaint_lod_scale_y_idc());
    }
  }

  const auto patchSizeXQuantizer = asps.asps_patch_size_quantizer_present_flag()
                                       ? 1U << ath.ath_patch_size_x_info_quantizer()
                                       : patchPackingBlockSize;
  pp.atlasPatch2dSizeX((pdu.pdu_2d_size_x_minus1() + 1) * patchSizeXQuantizer);

  const auto patchSizeYQuantizer = asps.asps_patch_size_quantizer_present_flag()
                                       ? 1U << ath.ath_patch_size_y_info_quantizer()
                                       : patchPackingBlockSize;
  pp.atlasPatch2dSizeY((pdu.pdu_2d_size_y_minus1() + 1) * patchSizeYQuantizer);

  if (asps.asps_miv_extension_present_flag()) {
    const auto &asme = asps.asps_miv_extension();

    pp.atlasPatchEntityId(pdu.pdu_miv_extension().pdu_entity_id());

    if (asme.asme_depth_occ_threshold_flag()) {
      pp.atlasPatchDepthOccMapThreshold(pdu.pdu_miv_extension().pdu_depth_occ_threshold());
    }
    if (asme.asme_patch_attribute_offset_enabled_flag()) {
      pp.atlasPatchAttributeOffset(pdu.pdu_miv_extension().pdu_attribute_offset());
    }
    if (asme.asme_inpaint_enabled_flag()) {
      pp.atlasPatchInpaintFlag(pdu.pdu_miv_extension().pdu_inpaint_flag());
    }
  }

  return pp;
}

auto PatchParams::encodePdu(const AtlasSequenceParameterSetRBSP &asps,
                            const AtlasFrameParameterSetRBSP &afps,
                            const AtlasTileHeader &ath) const -> MivBitstream::PatchDataUnit {
  auto pdu = MivBitstream::PatchDataUnit{};

  const auto patchPackingBlockSize = 1U << asps.asps_log2_patch_packing_block_size();
  VERIFY_MIVBITSTREAM(atlasPatch2dPosX() % patchPackingBlockSize == 0);
  VERIFY_MIVBITSTREAM(atlasPatch2dPosY() % patchPackingBlockSize == 0);
  pdu.pdu_2d_pos_x(atlasPatch2dPosX() / patchPackingBlockSize);
  pdu.pdu_2d_pos_y(atlasPatch2dPosY() / patchPackingBlockSize);

  pdu.pdu_3d_offset_u(atlasPatch3dOffsetU());
  pdu.pdu_3d_offset_v(atlasPatch3dOffsetV());

  const auto offsetDQuantizer = 1U << ath.ath_pos_min_d_quantizer();
  VERIFY_MIVBITSTREAM(atlasPatch3dOffsetD() % offsetDQuantizer == 0);
  pdu.pdu_3d_offset_d(atlasPatch3dOffsetD() / offsetDQuantizer);

  if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
    const auto rangeDQuantizer = 1U << ath.ath_pos_delta_max_d_quantizer();
    VERIFY_MIVBITSTREAM((atlasPatch3dRangeD() + 1) % rangeDQuantizer == 0);
    pdu.pdu_3d_range_d(atlasPatch3dRangeD() / rangeDQuantizer + 1);
  }

  pdu.pdu_projection_id(atlasPatchProjectionId());
  pdu.pdu_orientation_index(atlasPatchOrientationIndex());

  if (atlasPatchLoDScaleX() != 1 || atlasPatchLoDScaleY() != 1) {
    if (afps.afps_lod_mode_enabled_flag()) {
      pdu.pdu_lod_enabled_flag(true);
      pdu.pdu_lod_scale_x_minus1(atlasPatchLoDScaleX() - 1U);
      pdu.pdu_lod_scale_y_idc(atlasPatchLoDScaleY() -
                              (pdu.pdu_lod_scale_x_minus1() == 0 ? 2U : 1U));
    } else {
      const auto &afme = afps.afps_miv_extension();
      VERIFY_MIVBITSTREAM(afme.afme_inpaint_lod_enabled_flag());
      VERIFY_MIVBITSTREAM(atlasPatchLoDScaleX() ==
                          static_cast<int32_t>(afme.afme_inpaint_lod_scale_x_minus1()) + 1);
      VERIFY_MIVBITSTREAM(
          atlasPatchLoDScaleY() ==
          static_cast<int32_t>((afme.afme_inpaint_lod_scale_x_minus1() == 0 ? 2U : 1U) +
                               afme.afme_inpaint_lod_scale_y_idc()));
    }
  }

  const auto patchSizeXQuantizer = asps.asps_patch_size_quantizer_present_flag()
                                       ? 1U << ath.ath_patch_size_x_info_quantizer()
                                       : patchPackingBlockSize;
  VERIFY_MIVBITSTREAM(atlasPatch2dSizeX() % patchSizeXQuantizer == 0);
  pdu.pdu_2d_size_x_minus1(atlasPatch2dSizeX() / patchSizeXQuantizer - 1);

  const auto patchSizeYQuantizer = asps.asps_patch_size_quantizer_present_flag()
                                       ? 1U << ath.ath_patch_size_y_info_quantizer()
                                       : patchPackingBlockSize;
  VERIFY_MIVBITSTREAM(atlasPatch2dSizeY() % patchSizeYQuantizer == 0);
  pdu.pdu_2d_size_y_minus1(atlasPatch2dSizeY() / patchSizeYQuantizer - 1);

  if (atlasPatchEntityId()) {
    pdu.pdu_miv_extension().pdu_entity_id(*atlasPatchEntityId());
  }
  if (atlasPatchDepthOccMapThreshold()) {
    pdu.pdu_miv_extension().pdu_depth_occ_threshold(*atlasPatchDepthOccMapThreshold());
  }
  if (asps.asps_miv_extension_present_flag() &&
      asps.asps_miv_extension().asme_patch_attribute_offset_enabled_flag()) {
    pdu.pdu_miv_extension().pdu_attribute_offset(atlasPatchAttributeOffset());
  }
  if (asps.asps_miv_extension_present_flag() &&
      asps.asps_miv_extension().asme_inpaint_enabled_flag()) {
    pdu.pdu_miv_extension().pdu_inpaint_flag(atlasPatchInpaintFlag());
  }

  return pdu;
}

auto PatchParams::atlasToViewTransform() const noexcept -> Common::Mat3x3i {
  const auto xS = atlasPatch2dSizeX();
  const auto yS = atlasPatch2dSizeY();
  const auto x0 = atlasPatch2dPosX();
  const auto y0 = atlasPatch2dPosY();
  const auto lodX = atlasPatchLoDScaleX();
  const auto lodY = atlasPatchLoDScaleY();
  const auto u0 = atlasPatch3dOffsetU();
  const auto v0 = atlasPatch3dOffsetV();

  // (u, v) == M.(x, y, 1)

  switch (atlasPatchOrientationIndex()) {
  case FlexiblePatchOrientation::FPO_NULL:            // (x, y)
    return {lodX, 0,    u0 - lodX * x0,               //
            0,    lodY, v0 - lodY * y0,               //
            0,    0,    1};                           //
  case FlexiblePatchOrientation::FPO_SWAP:            // (y, x)
    return {0,    lodY, u0 - lodY * y0,               //
            lodX, 0,    v0 - lodX * x0,               //
            0,    0,    1};                           //
  case FlexiblePatchOrientation::FPO_ROT90:           // (y, -x)
    return {0,     lodY, u0 - lodY * y0,              //
            -lodX, 0,    v0 + lodX * (x0 + xS - 1),   //
            0,     0,    1};                          //
  case FlexiblePatchOrientation::FPO_ROT180:          // (-x, -y)
    return {-lodX, 0,     u0 + lodX * (-1 + x0 + xS), //
            0,     -lodY, v0 + lodY * (-1 + y0 + yS), //
            0,     0,     1};                         //
  case FlexiblePatchOrientation::FPO_ROT270:          // (-y, x)
    return {0,    -lodY, u0 + lodY * (y0 + yS - 1),   //
            lodX, 0,     v0 - lodX * x0,              //
            0,    0,     1};                          //
  case FlexiblePatchOrientation::FPO_MIRROR:          // (-x, y)
    return {-lodX, 0,    u0 + lodX * (x0 + xS - 1),   //
            0,     lodY, v0 - lodY * y0,              //
            0,     0,    1};                          //
  case FlexiblePatchOrientation::FPO_MROT90:          // (-y, -x)
    return {0,     -lodY, u0 + lodY * (y0 + yS - 1),  //
            -lodX, 0,     v0 + lodX * (x0 + xS - 1),  //
            0,     0,     1};                         //
  case FlexiblePatchOrientation::FPO_MROT180:         // (x, -y)
    return {lodX, 0,     u0 - lodX * x0,              //
            0,    -lodY, v0 + lodY * (y0 + yS - 1),   //
            0,    0,     1};                          //
  default:
    abort();
  }
}

auto PatchParams::viewToAtlasTransform() const noexcept -> Common::Mat3x3i {
  const auto m = atlasToViewTransform();
  const auto d = std::lcm(atlasPatchLoDScaleX(), atlasPatchLoDScaleY());

  // (u, v, 1) == M.(x, y, 1) ==> (x, y, 1) == invM.(u, v, 1), where '.' denots the dot product
  if (isRotated()) {
    return {0,           d / m(1, 0), -d * m(1, 2) / m(1, 0), //
            d / m(0, 1), 0,           -d * m(0, 2) / m(0, 1), //
            0,           0,           d};                     //
  }
  return {d / m(0, 0), 0,           -d * m(0, 2) / m(0, 0), //
          0,           d / m(1, 1), -d * m(1, 2) / m(1, 1), //
          0,           0,           d};                     //
}
} // namespace TMIV::MivBitstream
