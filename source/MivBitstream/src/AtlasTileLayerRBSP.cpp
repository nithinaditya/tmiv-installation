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

#include <TMIV/MivBitstream/AtlasTileLayerRBSP.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/verify.h>

#include <fmt/ostream.h>

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, AthType x) -> std::ostream & {
  switch (x) {
  case AthType::P_TILE:
    return stream << "P_TILE";
  case AthType::I_TILE:
    return stream << "I_TILE";
  case AthType::SKIP_TILE:
    return stream << "SKIP_TILE";
  default:
    return stream << "[unknown:" << static_cast<int>(x) << "]";
  }
}

auto operator<<(std::ostream &stream, FlexiblePatchOrientation x) -> std::ostream & {
  switch (x) {
  case FlexiblePatchOrientation::FPO_NULL:
    return stream << "FPO_NULL";
  case FlexiblePatchOrientation::FPO_SWAP:
    return stream << "FPO_SWAP";
  case FlexiblePatchOrientation::FPO_ROT90:
    return stream << "FPO_ROT90";
  case FlexiblePatchOrientation::FPO_ROT180:
    return stream << "FPO_ROT180";
  case FlexiblePatchOrientation::FPO_ROT270:
    return stream << "FPO_ROT270";
  case FlexiblePatchOrientation::FPO_MIRROR:
    return stream << "FPO_MIRROR";
  case FlexiblePatchOrientation::FPO_MROT90:
    return stream << "FPO_MROT90";
  case FlexiblePatchOrientation::FPO_MROT180:
    return stream << "FPO_MROT180";
  default:
    return stream << "[unknown:" << static_cast<int>(x) << "]";
  }
}

auto printTo(std::ostream &stream, AtduPatchMode x, AthType ath_type) -> std::ostream & {
  switch (ath_type) {
  case AthType::I_TILE:
    switch (x) {
    case AtduPatchMode::I_INTRA:
      return stream << "I_INTRA";
    case AtduPatchMode::I_RAW:
      return stream << "I_RAW";
    case AtduPatchMode::I_EOM:
      return stream << "I_EOM";
    case AtduPatchMode::I_END:
      return stream << "I_END";
    default:
      return stream << "[unknown:" << static_cast<int>(x) << "]";
    }
  case AthType::P_TILE:
    switch (x) {
    case AtduPatchMode::P_SKIP:
      return stream << "P_SKIP";
    case AtduPatchMode::P_MERGE:
      return stream << "P_MERGE";
    case AtduPatchMode::P_INTER:
      return stream << "P_INTER";
    case AtduPatchMode::P_INTRA:
      return stream << "P_INTRA";
    case AtduPatchMode::P_RAW:
      return stream << "P_RAW";
    case AtduPatchMode::P_EOM:
      return stream << "P_EOM";
    case AtduPatchMode::P_END:
      return stream << "P_END";
    default:
      return stream << "[unknown:" << static_cast<int>(x) << "]";
    }
  case AthType::SKIP_TILE:
    switch (x) {
    case AtduPatchMode::P_SKIP:
      return stream << "P_SKIP";
    default:
      return stream << "[unknown:" << static_cast<int>(x) << "]";
    }
  default:
    return stream << "[unknown:" << static_cast<int>(x) << "]";
  }
}

auto AtlasTileHeader::ath_atlas_output_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(m_ath_atlas_output_flag.has_value());
  return *m_ath_atlas_output_flag;
}

auto AtlasTileHeader::ath_patch_size_x_info_quantizer() const noexcept -> uint8_t {
  VERIFY_V3CBITSTREAM(ath_type() != AthType::SKIP_TILE);
  VERIFY_V3CBITSTREAM(m_ath_patch_size_x_info_quantizer.has_value());
  return *m_ath_patch_size_x_info_quantizer;
}

auto AtlasTileHeader::ath_patch_size_y_info_quantizer() const noexcept -> uint8_t {
  VERIFY_V3CBITSTREAM(ath_type() != AthType::SKIP_TILE);
  VERIFY_V3CBITSTREAM(m_ath_patch_size_x_info_quantizer.has_value());
  return *m_ath_patch_size_y_info_quantizer;
}

auto AtlasTileHeader::ath_patch_size_x_info_quantizer(const uint8_t value) noexcept
    -> AtlasTileHeader & {
  VERIFY_V3CBITSTREAM(ath_type() != AthType::SKIP_TILE);
  m_ath_patch_size_x_info_quantizer = value;
  return *this;
}

auto AtlasTileHeader::ath_patch_size_y_info_quantizer(const uint8_t value) noexcept
    -> AtlasTileHeader & {
  VERIFY_V3CBITSTREAM(ath_type() != AthType::SKIP_TILE);
  m_ath_patch_size_y_info_quantizer = value;
  return *this;
}

auto operator<<(std::ostream &stream, const AtlasTileHeader &x) -> std::ostream & {
  if (x.m_ath_no_output_of_prior_atlas_frames_flag.has_value()) {
    stream << "ath_no_output_of_prior_atlas_frames_flag=" << std::boolalpha
           << x.ath_no_output_of_prior_atlas_frames_flag() << '\n';
  }
  stream << "ath_atlas_frame_parameter_set_id=" << int{x.ath_atlas_frame_parameter_set_id()}
         << '\n';
  stream << "ath_atlas_adaptation_parameter_set_id=" << int{x.m_ath_adaptation_parameter_set_id}
         << '\n';
  stream << "ath_id=" << int{x.ath_id()} << '\n';
  stream << "ath_type=" << x.ath_type() << '\n';
  if (x.m_ath_atlas_output_flag) {
    stream << "ath_atlas_output_flag=" << std::boolalpha << *x.m_ath_atlas_output_flag << '\n';
  }
  stream << "ath_atlas_frm_order_cnt_lsb=" << int{x.ath_atlas_frm_order_cnt_lsb()} << '\n';
  if (x.m_ath_ref_atlas_frame_list_asps_flag) {
    stream << "ath_ref_atlas_frame_list_asps_flag=" << std::boolalpha
           << *x.m_ath_ref_atlas_frame_list_asps_flag << '\n';
  }
  if (x.ath_type() != AthType::SKIP_TILE) {
    if (x.m_ath_pos_min_d_quantizer) {
      stream << "ath_pos_min_d_quantizer=" << int{*x.m_ath_pos_min_d_quantizer} << '\n';
      if (x.m_ath_pos_delta_max_d_quantizer) {
        stream << "ath_pos_delta_max_d_quantizer=" << int{*x.m_ath_pos_delta_max_d_quantizer}
               << '\n';
      }
    }
    if (x.m_ath_patch_size_x_info_quantizer || x.m_ath_patch_size_y_info_quantizer) {
      stream << "ath_patch_size_x_info_quantizer=" << int{x.ath_patch_size_x_info_quantizer()}
             << '\n';
      stream << "ath_patch_size_y_info_quantizer=" << int{x.ath_patch_size_y_info_quantizer()}
             << '\n';
    }
  }
  return stream;
}

auto AtlasTileHeader::decodeFrom(Common::InputBitstream &bitstream, const NalUnitHeader &nuh,
                                 const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                                 const std::vector<AtlasFrameParameterSetRBSP> &afpsV)
    -> AtlasTileHeader {
  auto x = AtlasTileHeader{};

  if (NalUnitType::NAL_BLA_W_LP <= nuh.nal_unit_type() &&
      nuh.nal_unit_type() <= NalUnitType::NAL_RSV_IRAP_ACL_29) {
    x.ath_no_output_of_prior_atlas_frames_flag(bitstream.getFlag());
  }

  x.ath_atlas_frame_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());
  VERIFY_V3CBITSTREAM(x.ath_atlas_frame_parameter_set_id() <= 63);
  const auto &afps = afpsById(afpsV, x.ath_atlas_frame_parameter_set_id());
  const auto &asps = aspsById(aspsV, afps.afps_atlas_sequence_parameter_set_id());

  x.ath_atlas_adaptation_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());

  VERIFY_MIVBITSTREAM(afps.atlas_frame_tile_information().afti_single_tile_in_atlas_frame_flag());
  x.ath_id(0);

  x.ath_type(AthType(bitstream.getUExpGolomb<uint8_t>()));
  VERIFY_MIVBITSTREAM(x.ath_type() == AthType::I_TILE || x.ath_type() == AthType::SKIP_TILE);

  if (afps.afps_output_flag_present_flag()) {
    x.ath_atlas_output_flag(bitstream.getFlag());
  }

  x.ath_atlas_frm_order_cnt_lsb(
      bitstream.readBits<uint16_t>(asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4));

  if (asps.asps_num_ref_atlas_frame_lists_in_asps() > 0) {
    x.ath_ref_atlas_frame_list_asps_flag(bitstream.getFlag());
  }

  LIMITATION(x.ath_ref_atlas_frame_list_asps_flag());
  LIMITATION(asps.ref_list_struct(0).num_ref_entries() <= 1);

  if (x.ath_type() != AthType::SKIP_TILE) {
    if (asps.asps_normal_axis_limits_quantization_enabled_flag()) {
      x.ath_pos_min_d_quantizer(bitstream.readBits<uint8_t>(5));
      if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
        x.ath_pos_delta_max_d_quantizer(bitstream.readBits<uint8_t>(5));
      }
    }
    if (asps.asps_patch_size_quantizer_present_flag()) {
      x.ath_patch_size_x_info_quantizer(bitstream.readBits<uint8_t>(3));
      VERIFY_V3CBITSTREAM(x.ath_patch_size_x_info_quantizer() <=
                          asps.asps_log2_patch_packing_block_size());

      x.ath_patch_size_y_info_quantizer(bitstream.readBits<uint8_t>(3));
      VERIFY_V3CBITSTREAM(x.ath_patch_size_y_info_quantizer() <=
                          asps.asps_log2_patch_packing_block_size());
    }

    VERIFY_MIVBITSTREAM(!afps.afps_raw_3d_offset_bit_count_explicit_mode_flag());
  }

  bitstream.byteAlignment();

  return x;
}

void AtlasTileHeader::encodeTo(Common::OutputBitstream &bitstream, const NalUnitHeader &nuh,
                               const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                               const std::vector<AtlasFrameParameterSetRBSP> &afpsV) const {
  if (NalUnitType::NAL_BLA_W_LP <= nuh.nal_unit_type() &&
      nuh.nal_unit_type() <= NalUnitType::NAL_RSV_IRAP_ACL_29) {
    bitstream.putFlag(ath_no_output_of_prior_atlas_frames_flag());
  }

  VERIFY_V3CBITSTREAM(ath_atlas_frame_parameter_set_id() <= 63);
  bitstream.putUExpGolomb(ath_atlas_frame_parameter_set_id());

  const auto &afps = afpsById(afpsV, ath_atlas_frame_parameter_set_id());
  const auto &asps = aspsById(aspsV, afps.afps_atlas_sequence_parameter_set_id());

  bitstream.putUExpGolomb(ath_atlas_adaptation_parameter_set_id());

  VERIFY_MIVBITSTREAM(afps.atlas_frame_tile_information().afti_single_tile_in_atlas_frame_flag());
  VERIFY_V3CBITSTREAM(ath_id() == 0);

  VERIFY_MIVBITSTREAM(ath_type() == AthType::I_TILE || ath_type() == AthType::SKIP_TILE);
  bitstream.putUExpGolomb(ath_type());

  if (afps.afps_output_flag_present_flag()) {
    bitstream.putFlag(ath_atlas_output_flag());
  }

  bitstream.writeBits(ath_atlas_frm_order_cnt_lsb(),
                      asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4);

  LIMITATION(ath_ref_atlas_frame_list_asps_flag());
  LIMITATION(asps.ref_list_struct(0).num_ref_entries() <= 1);

  VERIFY_V3CBITSTREAM(asps.asps_num_ref_atlas_frame_lists_in_asps() > 0 ||
                      !ath_ref_atlas_frame_list_asps_flag());
  if (asps.asps_num_ref_atlas_frame_lists_in_asps() > 0) {
    bitstream.putFlag(ath_ref_atlas_frame_list_asps_flag());
  }

  if (ath_type() != AthType::SKIP_TILE) {
    if (asps.asps_normal_axis_limits_quantization_enabled_flag()) {
      bitstream.writeBits(ath_pos_min_d_quantizer(), 5);
      if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
        bitstream.writeBits(ath_pos_delta_max_d_quantizer(), 5);
      }
    }
    if (asps.asps_patch_size_quantizer_present_flag()) {
      VERIFY_V3CBITSTREAM(ath_patch_size_x_info_quantizer() <=
                          asps.asps_log2_patch_packing_block_size());
      bitstream.writeBits(ath_patch_size_x_info_quantizer(), 3);

      VERIFY_V3CBITSTREAM(ath_patch_size_y_info_quantizer() <=
                          asps.asps_log2_patch_packing_block_size());
      bitstream.writeBits(ath_patch_size_y_info_quantizer(), 3);
    }

    VERIFY_MIVBITSTREAM(!afps.afps_raw_3d_offset_bit_count_explicit_mode_flag());
  }

  bitstream.byteAlignment();
}

auto operator<<(std::ostream &stream, const SkipPatchDataUnit & /* x */) -> std::ostream & {
  return stream;
}

auto PduMivExtension::pdu_depth_occ_threshold() const -> uint32_t {
  VERIFY_MIVBITSTREAM(m_pdu_depth_occ_threshold.has_value());
  return *m_pdu_depth_occ_threshold;
}

auto PduMivExtension::pdu_attribute_offset() const -> Common::Vec3w {
  return m_pdu_attribute_offset.value_or(Common::Vec3w{});
}

auto PduMivExtension::printTo(std::ostream &stream, unsigned tileId, size_t patchIdx) const
    -> std::ostream & {
  if (m_pdu_entity_id) {
    fmt::print(stream, "pdu_entity_id[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_entity_id());
  }
  if (m_pdu_depth_occ_threshold) {
    fmt::print(stream, "pdu_depth_occ_threshold[ {} ][ {} ]={}\n", tileId, patchIdx,
               pdu_depth_occ_threshold());
  }
  if (m_pdu_attribute_offset) {
    for (int c = 0; c < 3; ++c) {
      fmt::print(stream, "pdu_attribute_offset[ {} ][ {} ][ {} ]={}\n", tileId, patchIdx, c,
                 pdu_attribute_offset()[c]);
    }
  }
  if (m_pdu_inpaint_flag) {
    fmt::print(stream, "pdu_inpaint_flag[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_inpaint_flag());
  }
  return stream;
}

auto PduMivExtension::operator==(const PduMivExtension &other) const noexcept -> bool {
  return pdu_entity_id() == other.pdu_entity_id() &&
         m_pdu_depth_occ_threshold == other.m_pdu_depth_occ_threshold &&
         pdu_attribute_offset() == other.pdu_attribute_offset() &&
         pdu_inpaint_flag() == other.pdu_inpaint_flag();
}

auto PduMivExtension::operator!=(const PduMivExtension &other) const noexcept -> bool {
  return !operator==(other);
}

auto PduMivExtension::decodeFrom(Common::InputBitstream &bitstream,
                                 const AtlasSequenceParameterSetRBSP &asps) -> PduMivExtension {
  auto x = PduMivExtension{};

  if (asps.asps_miv_extension_present_flag()) {
    const auto &asme = asps.asps_miv_extension();
    if (0 < asme.asme_max_entity_id()) {
      x.pdu_entity_id(bitstream.getUVar<uint32_t>(asme.asme_max_entity_id()));
    }
    if (asme.asme_depth_occ_threshold_flag()) {
      x.pdu_depth_occ_threshold(
          bitstream.readBits<uint32_t>(asps.asps_geometry_2d_bit_depth_minus1() + 1));
    }
    if (asme.asme_patch_attribute_offset_enabled_flag()) {
      int bits = asps.asps_miv_extension().asme_patch_attribute_offset_bit_depth_minus1() + 1;
      x.pdu_attribute_offset({bitstream.readBits<uint16_t>(bits),
                              bitstream.readBits<uint16_t>(bits),
                              bitstream.readBits<uint16_t>(bits)});
    }
    if (asme.asme_inpaint_enabled_flag()) {
      x.pdu_inpaint_flag(bitstream.getFlag());
    }
  }
  return x;
}

void PduMivExtension::encodeTo(Common::OutputBitstream &bitstream,
                               const AtlasSequenceParameterSetRBSP &asps) const {
  if (asps.asps_extension_present_flag() && asps.asps_miv_extension_present_flag()) {
    const auto &asme = asps.asps_miv_extension();
    if (0 < asme.asme_max_entity_id()) {
      bitstream.putUVar(pdu_entity_id(), asme.asme_max_entity_id());
    } else {
      VERIFY_MIVBITSTREAM(!m_pdu_entity_id.has_value());
    }
    if (asme.asme_depth_occ_threshold_flag()) {
      bitstream.writeBits(pdu_depth_occ_threshold(), asps.asps_geometry_2d_bit_depth_minus1() + 1);
    } else {
      VERIFY_MIVBITSTREAM(!m_pdu_depth_occ_threshold.has_value());
    }
    if (asme.asme_patch_attribute_offset_enabled_flag()) {
      const auto bits =
          asps.asps_miv_extension().asme_patch_attribute_offset_bit_depth_minus1() + 1;
      VERIFY_MIVBITSTREAM(m_pdu_attribute_offset.has_value());
      bitstream.writeBits(uint16_t(pdu_attribute_offset().x()), bits);
      bitstream.writeBits(uint16_t(pdu_attribute_offset().y()), bits);
      bitstream.writeBits(uint16_t(pdu_attribute_offset().z()), bits);
    }
    if (asme.asme_inpaint_enabled_flag()) {
      VERIFY_MIVBITSTREAM(m_pdu_inpaint_flag.has_value());
      bitstream.putFlag(pdu_inpaint_flag());
    }
  }
}

auto PatchDataUnit::pdu_3d_range_d() const -> uint32_t {
  VERIFY_V3CBITSTREAM(m_pdu_3d_range_d.has_value());
  return *m_pdu_3d_range_d;
}

auto PatchDataUnit::pdu_miv_extension(const PduMivExtension &value) noexcept -> PatchDataUnit & {
  m_pdu_miv_extension = value;
  return *this;
}

auto PatchDataUnit::printTo(std::ostream &stream, unsigned tileId, size_t patchIdx) const
    -> std::ostream & {
  fmt::print(stream, "pdu_2d_pos_x[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_2d_pos_x());
  fmt::print(stream, "pdu_2d_pos_y[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_2d_pos_y());
  fmt::print(stream, "pdu_2d_size_x_minus1[ {} ][ {} ]={}\n", tileId, patchIdx,
             pdu_2d_size_x_minus1());
  fmt::print(stream, "pdu_2d_size_y_minus1[ {} ][ {} ]={}\n", tileId, patchIdx,
             pdu_2d_size_y_minus1());
  fmt::print(stream, "pdu_3d_offset_u[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_3d_offset_u());
  fmt::print(stream, "pdu_3d_offset_v[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_3d_offset_v());
  fmt::print(stream, "pdu_3d_offset_d[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_3d_offset_d());

  if (m_pdu_3d_range_d) {
    fmt::print(stream, "pdu_3d_range_d[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_3d_range_d());
  }

  fmt::print(stream, "pdu_projection_id[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_projection_id());
  fmt::print(stream, "pdu_orientation_index[ {} ][ {} ]={}\n", tileId, patchIdx,
             pdu_orientation_index());

  if (m_pdu_lod_enabled_flag) {
    fmt::print(stream, "pdu_lod_enabled_flag[ {} ][ {} ]={}\n", tileId, patchIdx,
               pdu_lod_enabled_flag());

    if (pdu_lod_enabled_flag()) {
      fmt::print(stream, "pdu_lod_scale_x_minus1[ {} ][ {} ]={}\n", tileId, patchIdx,
                 pdu_lod_scale_x_minus1());
      fmt::print(stream, "pdu_lod_scale_y_idc[ {} ][ {} ]={}\n", tileId, patchIdx,
                 pdu_lod_scale_y_idc());
    }
  }
  if (m_pdu_miv_extension) {
    m_pdu_miv_extension->printTo(stream, tileId, patchIdx);
  }
  return stream;
}

auto PatchDataUnit::decodeFrom(Common::InputBitstream &bitstream,
                               const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                               const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                               const AtlasTileHeader &ath) -> PatchDataUnit {
  auto x = PatchDataUnit{};

  const auto &afps = afpsById(afpsV, ath.ath_atlas_frame_parameter_set_id());
  const auto &asps = aspsById(aspsV, afps.afps_atlas_sequence_parameter_set_id());

  const auto pdu3dOffsetUVNumBits = asps.asps_geometry_3d_bit_depth_minus1() + 1U;
  const auto pdu3dOffsetDNumBits =
      asps.asps_geometry_3d_bit_depth_minus1() - ath.ath_pos_min_d_quantizer() + 1;
  const auto rangeDBitDepth = std::min(asps.asps_geometry_2d_bit_depth_minus1() + 1,
                                       asps.asps_geometry_3d_bit_depth_minus1() + 1);
  const auto pdu3dRangeDNumBits = rangeDBitDepth - ath.ath_pos_delta_max_d_quantizer();
  const auto pduProjectionIdNumBits =
      Common::ceilLog2(asps.asps_max_number_projections_minus1() + 1ULL);
  const auto pduOrientationIndexNumBits = asps.asps_use_eight_orientations_flag() ? 3 : 1;

  x.pdu_2d_pos_x(bitstream.getUExpGolomb<uint32_t>());
  x.pdu_2d_pos_y(bitstream.getUExpGolomb<uint32_t>());
  x.pdu_2d_size_x_minus1(bitstream.getUExpGolomb<uint32_t>());
  x.pdu_2d_size_y_minus1(bitstream.getUExpGolomb<uint32_t>());
  x.pdu_3d_offset_u(bitstream.readBits<uint32_t>(pdu3dOffsetUVNumBits));
  x.pdu_3d_offset_v(bitstream.readBits<uint32_t>(pdu3dOffsetUVNumBits));

  VERIFY_V3CBITSTREAM(pdu3dOffsetDNumBits >= 0);
  x.pdu_3d_offset_d(bitstream.readBits<uint32_t>(pdu3dOffsetDNumBits));

  if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
    VERIFY_V3CBITSTREAM(pdu3dRangeDNumBits >= 0);
    x.pdu_3d_range_d(bitstream.readBits<uint32_t>(pdu3dRangeDNumBits));
  }

  x.pdu_projection_id(bitstream.readBits<uint16_t>(pduProjectionIdNumBits));
  x.pdu_orientation_index(bitstream.readBits<FlexiblePatchOrientation>(pduOrientationIndexNumBits));

  if (afps.afps_lod_mode_enabled_flag()) {
    x.pdu_lod_enabled_flag(bitstream.getFlag());
    if (x.pdu_lod_enabled_flag()) {
      x.pdu_lod_scale_x_minus1(bitstream.getUExpGolomb<unsigned>());
      x.pdu_lod_scale_y_idc(bitstream.getUExpGolomb<unsigned>());
    }
  }

  VERIFY_MIVBITSTREAM(!asps.asps_plr_enabled_flag());

  if (asps.asps_miv_extension_present_flag()) {
    x.pdu_miv_extension(PduMivExtension::decodeFrom(bitstream, asps));
  }
  return x;
}

void PatchDataUnit::encodeTo(Common::OutputBitstream &bitstream,
                             const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                             const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                             const AtlasTileHeader &ath) const {
  const auto &afps = afpsById(afpsV, ath.ath_atlas_frame_parameter_set_id());
  const auto &asps = aspsById(aspsV, afps.afps_atlas_sequence_parameter_set_id());

  const auto pdu3dOffsetUVNumBits = asps.asps_geometry_3d_bit_depth_minus1() + 1U;
  const auto pdu3dOffsetDNumBits =
      asps.asps_geometry_3d_bit_depth_minus1() - ath.ath_pos_min_d_quantizer() + 1;
  const auto rangeDBitDepth = std::min(asps.asps_geometry_2d_bit_depth_minus1() + 1,
                                       asps.asps_geometry_3d_bit_depth_minus1() + 1);
  const auto pdu3dRangeDNumBits = rangeDBitDepth - ath.ath_pos_delta_max_d_quantizer();
  const auto pduProjectionIdNumBits =
      Common::ceilLog2(asps.asps_max_number_projections_minus1() + 1ULL);
  const auto pduOrientationIndexNumBits = asps.asps_use_eight_orientations_flag() ? 3 : 1;

  bitstream.putUExpGolomb(pdu_2d_pos_x());
  bitstream.putUExpGolomb(pdu_2d_pos_y());
  bitstream.putUExpGolomb(pdu_2d_size_x_minus1());
  bitstream.putUExpGolomb(pdu_2d_size_y_minus1());
  bitstream.writeBits(pdu_3d_offset_u(), pdu3dOffsetUVNumBits);
  bitstream.writeBits(pdu_3d_offset_v(), pdu3dOffsetUVNumBits);

  VERIFY_V3CBITSTREAM(pdu3dOffsetDNumBits >= 0);
  bitstream.writeBits(pdu_3d_offset_d(), pdu3dOffsetDNumBits);

  if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
    VERIFY_V3CBITSTREAM(pdu3dRangeDNumBits >= 0);
    bitstream.writeBits(pdu_3d_range_d(), pdu3dRangeDNumBits);
  }

  bitstream.writeBits(pdu_projection_id(), pduProjectionIdNumBits);
  bitstream.writeBits(pdu_orientation_index(), pduOrientationIndexNumBits);

  if (afps.afps_lod_mode_enabled_flag()) {
    bitstream.putFlag(pdu_lod_enabled_flag());
    if (pdu_lod_enabled_flag()) {
      bitstream.putUExpGolomb(pdu_lod_scale_x_minus1());
      bitstream.putUExpGolomb(pdu_lod_scale_y_idc());
    }
  }

  VERIFY_MIVBITSTREAM(!asps.asps_plr_enabled_flag());

  if (asps.asps_miv_extension_present_flag()) {
    pdu_miv_extension().encodeTo(bitstream, asps);
  } else {
    VERIFY_V3CBITSTREAM(!m_pdu_miv_extension);
  }
}

auto PatchInformationData::skip_patch_data_unit() const noexcept -> const SkipPatchDataUnit & {
  VERIFY_V3CBITSTREAM(std::holds_alternative<SkipPatchDataUnit>(m_data));
  return *std::get_if<SkipPatchDataUnit>(&m_data);
}

auto PatchInformationData::patch_data_unit() const noexcept -> const PatchDataUnit & {
  VERIFY_V3CBITSTREAM(std::holds_alternative<PatchDataUnit>(m_data));
  return *std::get_if<PatchDataUnit>(&m_data);
}

auto PatchInformationData::printTo(std::ostream &stream, unsigned tileId, size_t patchIdx) const
    -> std::ostream & {
  visit(Common::overload([&](const std::monostate & /* unused */) { stream << "[unknown]\n"; },
                         [&](const SkipPatchDataUnit &x) { stream << x; },
                         [&](const PatchDataUnit &x) { x.printTo(stream, tileId, patchIdx); }),
        m_data);
  return stream;
}

auto PatchInformationData::operator==(const PatchInformationData &other) const noexcept -> bool {
  return data() == other.data();
}

auto PatchInformationData::operator!=(const PatchInformationData &other) const noexcept -> bool {
  return !operator==(other);
}

auto PatchInformationData::decodeFrom(Common::InputBitstream &bitstream,
                                      const std::vector<AtlasSequenceParameterSetRBSP> &asps,
                                      const std::vector<AtlasFrameParameterSetRBSP> &afps,
                                      const AtlasTileHeader &ath, AtduPatchMode patchMode)
    -> PatchInformationData {
  if (ath.ath_type() == AthType::I_TILE) {
    VERIFY_V3CBITSTREAM(patchMode == AtduPatchMode::I_INTRA);
    return PatchInformationData{PatchDataUnit::decodeFrom(bitstream, asps, afps, ath)};
  }
  if (ath.ath_type() == AthType::SKIP_TILE) {
    VERIFY_V3CBITSTREAM(patchMode == AtduPatchMode::P_SKIP);
    return PatchInformationData{SkipPatchDataUnit::decodeFrom(bitstream)};
  }
  V3CBITSTREAM_ERROR("Unknown or unsupported tile/patch mode combination");
}

void PatchInformationData::encodeTo(Common::OutputBitstream &bitstream,
                                    const std::vector<AtlasSequenceParameterSetRBSP> &asps,
                                    const std::vector<AtlasFrameParameterSetRBSP> &afps,
                                    const AtlasTileHeader &ath, AtduPatchMode patchMode) const {
  if (ath.ath_type() == AthType::I_TILE) {
    VERIFY_V3CBITSTREAM(patchMode == AtduPatchMode::I_INTRA);
    return patch_data_unit().encodeTo(bitstream, asps, afps, ath);
  }
  if (ath.ath_type() == AthType::SKIP_TILE) {
    VERIFY_V3CBITSTREAM(patchMode == AtduPatchMode::P_SKIP);
    return skip_patch_data_unit().encodeTo(bitstream);
  }
  V3CBITSTREAM_ERROR("Unknown or unsupported tile/patch mode combination");
}

auto AtlasTileDataUnit::atduTotalNumberOfPatches() const noexcept -> size_t {
  return m_vector.size();
}

auto AtlasTileDataUnit::atdu_patch_mode(size_t p) const -> AtduPatchMode {
  VERIFY_V3CBITSTREAM(p < m_vector.size());
  return m_vector[p].first;
}

auto AtlasTileDataUnit::patch_information_data(size_t p) const -> const PatchInformationData & {
  VERIFY_V3CBITSTREAM(p < m_vector.size());
  return m_vector[p].second;
}

auto AtlasTileDataUnit::printTo(std::ostream &stream, const AtlasTileHeader &ath) const
    -> std::ostream & {
  visit([&](const auto p, const AtduPatchMode patch_mode,
            const PatchInformationData &patch_information_data) {
    stream << "atdu_patch_mode[ " << p << " ]=";
    MivBitstream::printTo(stream, patch_mode, ath.ath_type()) << '\n';
    patch_information_data.printTo(stream, ath.ath_id(), p);
  });
  return stream;
}

auto AtlasTileDataUnit::operator==(const AtlasTileDataUnit &other) const -> bool {
  return m_vector == other.m_vector;
}

auto AtlasTileDataUnit::operator!=(const AtlasTileDataUnit &other) const -> bool {
  return !operator==(other);
}

auto AtlasTileDataUnit::decodeFrom(Common::InputBitstream &bitstream,
                                   const std::vector<AtlasSequenceParameterSetRBSP> &asps,
                                   const std::vector<AtlasFrameParameterSetRBSP> &afps,
                                   const AtlasTileHeader &ath) -> AtlasTileDataUnit {
  VERIFY_MIVBITSTREAM(ath.ath_type() == AthType::I_TILE || ath.ath_type() == AthType::SKIP_TILE);

  if (ath.ath_type() == AthType::SKIP_TILE) {
    return {};
  }

  auto x = AtlasTileDataUnit::Vector{};
  auto patch_mode = bitstream.getUExpGolomb<AtduPatchMode>();

  while (patch_mode != AtduPatchMode::I_END) {
    x.emplace_back(patch_mode,
                   PatchInformationData::decodeFrom(bitstream, asps, afps, ath, patch_mode));
    VERIFY_MIVBITSTREAM(patch_mode == AtduPatchMode::I_INTRA);
    patch_mode = bitstream.getUExpGolomb<AtduPatchMode>();
  }

  return AtlasTileDataUnit{x};
}

void AtlasTileDataUnit::encodeTo(Common::OutputBitstream &bitstream,
                                 const std::vector<AtlasSequenceParameterSetRBSP> &asps,
                                 const std::vector<AtlasFrameParameterSetRBSP> &afps,
                                 const AtlasTileHeader &ath) const {
  VERIFY_MIVBITSTREAM(ath.ath_type() == AthType::I_TILE || ath.ath_type() == AthType::SKIP_TILE);

  if (ath.ath_type() == AthType::I_TILE) {
    visit([&](const auto /* p */, const AtduPatchMode patch_mode,
              const PatchInformationData &patch_information_data) {
      bitstream.putUExpGolomb(patch_mode);
      patch_information_data.encodeTo(bitstream, asps, afps, ath, patch_mode);
    });

    bitstream.putUExpGolomb(AtduPatchMode::I_END);
  }
}

auto operator<<(std::ostream &stream, const AtlasTileLayerRBSP &x) -> std::ostream & {
  stream << x.atlas_tile_header();
  x.atlas_tile_data_unit().printTo(stream, x.atlas_tile_header());
  return stream;
}

auto AtlasTileLayerRBSP::operator==(const AtlasTileLayerRBSP &other) const noexcept -> bool {
  return atlas_tile_header() == other.atlas_tile_header() &&
         atlas_tile_data_unit() == other.atlas_tile_data_unit();
}

auto AtlasTileLayerRBSP::operator!=(const AtlasTileLayerRBSP &other) const noexcept -> bool {
  return !operator==(other);
}

auto AtlasTileLayerRBSP::decodeFrom(std::istream &stream, const NalUnitHeader &nuh,
                                    const std::vector<AtlasSequenceParameterSetRBSP> &asps,
                                    const std::vector<AtlasFrameParameterSetRBSP> &afps)
    -> AtlasTileLayerRBSP {
  Common::InputBitstream bitstream{stream};

  auto atl = AtlasTileLayerRBSP{};
  atl.atlas_tile_header() = AtlasTileHeader::decodeFrom(bitstream, nuh, asps, afps);
  atl.atlas_tile_data_unit() =
      AtlasTileDataUnit::decodeFrom(bitstream, asps, afps, atl.atlas_tile_header());
  bitstream.rbspTrailingBits();

  return atl;
}

void AtlasTileLayerRBSP::encodeTo(std::ostream &stream, const NalUnitHeader &nuh,
                                  const std::vector<AtlasSequenceParameterSetRBSP> &asps,
                                  const std::vector<AtlasFrameParameterSetRBSP> &afps) const {
  Common::OutputBitstream bitstream{stream};

  atlas_tile_header().encodeTo(bitstream, nuh, asps, afps);
  atlas_tile_data_unit().encodeTo(bitstream, asps, afps, atlas_tile_header());
  bitstream.rbspTrailingBits();
}
} // namespace TMIV::MivBitstream
