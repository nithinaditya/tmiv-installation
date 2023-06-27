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

#ifndef _TMIV_MIVBITSTREAM_ATLASTILELAYERRBSP_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto AtlasTileHeader::ath_no_output_of_prior_atlas_frames_flag() const noexcept {
  return m_ath_no_output_of_prior_atlas_frames_flag.value_or(false);
}

constexpr auto AtlasTileHeader::ath_atlas_frame_parameter_set_id() const noexcept {
  return m_ath_atlas_frame_parameter_set_id;
}

constexpr auto AtlasTileHeader::ath_atlas_adaptation_parameter_set_id() const noexcept {
  return m_ath_adaptation_parameter_set_id;
}

constexpr auto AtlasTileHeader::ath_id() const noexcept { return m_ath_id; }

constexpr auto AtlasTileHeader::ath_type() const noexcept { return m_ath_type; }

constexpr auto AtlasTileHeader::ath_atlas_frm_order_cnt_lsb() const noexcept {
  return m_ath_atlas_frm_order_cnt_lsb;
}

constexpr auto AtlasTileHeader::ath_ref_atlas_frame_list_asps_flag() const noexcept {
  return m_ath_ref_atlas_frame_list_asps_flag.value_or(false);
}

constexpr auto AtlasTileHeader::ath_pos_min_d_quantizer() const noexcept {
  return m_ath_pos_min_d_quantizer.value_or(0);
}

constexpr auto AtlasTileHeader::ath_pos_delta_max_d_quantizer() const noexcept {
  return m_ath_pos_delta_max_d_quantizer.value_or(0);
}

constexpr auto AtlasTileHeader::ath_no_output_of_prior_atlas_frames_flag(bool value) noexcept
    -> auto & {
  m_ath_no_output_of_prior_atlas_frames_flag = value;
  return *this;
}

constexpr auto AtlasTileHeader::ath_atlas_frame_parameter_set_id(const std::uint8_t value) noexcept
    -> auto & {
  VERIFY_MIVBITSTREAM(value <= 63U);
  m_ath_atlas_frame_parameter_set_id = value;
  return *this;
}

constexpr auto
AtlasTileHeader::ath_atlas_adaptation_parameter_set_id(const std::uint8_t value) noexcept
    -> auto & {
  m_ath_adaptation_parameter_set_id = value;
  return *this;
}

constexpr auto AtlasTileHeader::ath_id(const std::uint8_t value) noexcept -> auto & {
  m_ath_id = value;
  return *this;
}

constexpr auto AtlasTileHeader::ath_type(const AthType value) noexcept -> auto & {
  m_ath_type = value;
  return *this;
}

constexpr auto AtlasTileHeader::ath_atlas_output_flag(const bool value) noexcept -> auto & {
  m_ath_atlas_output_flag = value;
  return *this;
}

constexpr auto AtlasTileHeader::ath_atlas_frm_order_cnt_lsb(const std::uint16_t value) noexcept
    -> auto & {
  m_ath_atlas_frm_order_cnt_lsb = value;
  return *this;
}

constexpr auto AtlasTileHeader::ath_ref_atlas_frame_list_asps_flag(const bool value) noexcept
    -> auto & {
  m_ath_ref_atlas_frame_list_asps_flag = value;
  return *this;
}

constexpr auto AtlasTileHeader::ath_pos_min_d_quantizer(const std::uint8_t value) noexcept
    -> auto & {
  m_ath_pos_min_d_quantizer = value;
  return *this;
}

constexpr auto AtlasTileHeader::ath_pos_delta_max_d_quantizer(const std::uint8_t value) noexcept
    -> auto & {
  m_ath_pos_delta_max_d_quantizer = value;
  return *this;
}

constexpr auto AtlasTileHeader::operator==(const AtlasTileHeader &other) const noexcept {
  if (ath_no_output_of_prior_atlas_frames_flag() !=
          other.ath_no_output_of_prior_atlas_frames_flag() ||
      ath_atlas_frame_parameter_set_id() != other.ath_atlas_frame_parameter_set_id() ||
      ath_atlas_adaptation_parameter_set_id() != other.ath_atlas_adaptation_parameter_set_id() ||
      ath_id() != other.ath_id() || ath_type() != other.ath_type() ||
      m_ath_atlas_output_flag != other.m_ath_atlas_output_flag ||
      ath_atlas_frm_order_cnt_lsb() != other.ath_atlas_frm_order_cnt_lsb() ||
      ath_ref_atlas_frame_list_asps_flag() != other.ath_ref_atlas_frame_list_asps_flag()) {
    return false;
  }
  if (ath_type() == AthType::SKIP_TILE) {
    return true;
  }
  return ath_pos_min_d_quantizer() == other.ath_pos_min_d_quantizer() &&
         ath_pos_delta_max_d_quantizer() == other.ath_pos_delta_max_d_quantizer() &&
         m_ath_patch_size_x_info_quantizer == other.m_ath_patch_size_x_info_quantizer &&
         m_ath_patch_size_y_info_quantizer == other.m_ath_patch_size_y_info_quantizer;
}

constexpr auto AtlasTileHeader::operator!=(const AtlasTileHeader &other) const noexcept {
  return !operator==(other);
}

constexpr auto SkipPatchDataUnit::operator==(const SkipPatchDataUnit & /* other */) const noexcept {
  return true;
}
constexpr auto SkipPatchDataUnit::operator!=(const SkipPatchDataUnit & /* other */) const noexcept {
  return false;
}

inline auto SkipPatchDataUnit::decodeFrom(Common::InputBitstream & /* bitstream */)
    -> SkipPatchDataUnit {
  return {};
}

inline void SkipPatchDataUnit::encodeTo(Common::OutputBitstream & /* bitstream */) const {}

constexpr auto PduMivExtension::pdu_entity_id() const noexcept {
  return m_pdu_entity_id.value_or(0);
}

constexpr auto PduMivExtension::pdu_inpaint_flag() const noexcept {
  return m_pdu_inpaint_flag.value_or(false);
}

constexpr auto PduMivExtension::pdu_entity_id(std::uint32_t value) noexcept -> auto & {
  m_pdu_entity_id = value;
  return *this;
}

constexpr auto PduMivExtension::pdu_depth_occ_threshold(std::uint32_t value) noexcept -> auto & {
  m_pdu_depth_occ_threshold = value;
  return *this;
}

inline auto PduMivExtension::pdu_attribute_offset(Common::Vec3w value) noexcept -> auto & {
  m_pdu_attribute_offset = value;
  return *this;
}

constexpr auto PduMivExtension::pdu_inpaint_flag(bool value) noexcept -> auto & {
  m_pdu_inpaint_flag = value;
  return *this;
}

constexpr auto PatchDataUnit::pdu_2d_pos_x() const noexcept { return m_pdu_2d_pos_x; }

constexpr auto PatchDataUnit::pdu_2d_pos_y() const noexcept { return m_pdu_2d_pos_y; }

constexpr auto PatchDataUnit::pdu_2d_size_x_minus1() const noexcept {
  return m_pdu_2d_size_x_minus1;
}

constexpr auto PatchDataUnit::pdu_2d_size_y_minus1() const noexcept {
  return m_pdu_2d_size_y_minus1;
}

constexpr auto PatchDataUnit::pdu_3d_offset_u() const noexcept { return m_pdu_3d_offset_u; }

constexpr auto PatchDataUnit::pdu_3d_offset_v() const noexcept { return m_pdu_3d_offset_v; }

constexpr auto PatchDataUnit::pdu_3d_offset_d() const noexcept { return m_pdu_3d_offset_d; }

constexpr auto PatchDataUnit::pdu_projection_id() const noexcept { return m_pdu_view_id; }

constexpr auto PatchDataUnit::pdu_orientation_index() const noexcept {
  return m_pdu_orientation_index;
}

constexpr auto PatchDataUnit::pdu_lod_enabled_flag() const noexcept {
  return m_pdu_lod_enabled_flag.value_or(false);
}

constexpr auto PatchDataUnit::pdu_lod_scale_x_minus1() const noexcept {
  return m_pdu_lod_scale_x_minus1.value_or(0U);
}

constexpr auto PatchDataUnit::pdu_lod_scale_y_idc() const noexcept {
  return m_pdu_lod_scale_y_idc.value_or(0U);
}

inline auto PatchDataUnit::pdu_miv_extension() const noexcept -> PduMivExtension {
  return m_pdu_miv_extension.value_or(PduMivExtension{});
}

constexpr auto PatchDataUnit::pdu_2d_pos_x(std::uint32_t value) noexcept -> auto & {
  m_pdu_2d_pos_x = value;
  return *this;
}

constexpr auto PatchDataUnit::pdu_2d_pos_y(std::uint32_t value) noexcept -> auto & {
  m_pdu_2d_pos_y = value;
  return *this;
}

constexpr auto PatchDataUnit::pdu_2d_size_x_minus1(std::uint32_t value) noexcept -> auto & {
  m_pdu_2d_size_x_minus1 = value;
  return *this;
}

constexpr auto PatchDataUnit::pdu_2d_size_y_minus1(std::uint32_t value) noexcept -> auto & {
  m_pdu_2d_size_y_minus1 = value;
  return *this;
}

constexpr auto PatchDataUnit::pdu_3d_offset_u(std::uint32_t value) noexcept -> auto & {
  m_pdu_3d_offset_u = value;
  return *this;
}

constexpr auto PatchDataUnit::pdu_3d_offset_v(std::uint32_t value) noexcept -> auto & {
  m_pdu_3d_offset_v = value;
  return *this;
}

constexpr auto PatchDataUnit::pdu_3d_offset_d(std::uint32_t value) noexcept -> auto & {
  m_pdu_3d_offset_d = value;
  return *this;
}

constexpr auto PatchDataUnit::pdu_3d_range_d(std::uint32_t value) noexcept -> auto & {
  m_pdu_3d_range_d = value;
  return *this;
}

constexpr auto PatchDataUnit::pdu_projection_id(std::uint16_t value) noexcept -> auto & {
  m_pdu_view_id = value;
  return *this;
}

constexpr auto PatchDataUnit::pdu_orientation_index(const FlexiblePatchOrientation value) noexcept
    -> auto & {
  m_pdu_orientation_index = value;
  return *this;
}

constexpr auto PatchDataUnit::pdu_lod_enabled_flag(bool value) noexcept -> auto & {
  m_pdu_lod_enabled_flag = value;
  return *this;
}

constexpr auto PatchDataUnit::pdu_lod_scale_x_minus1(unsigned value) noexcept -> auto & {
  m_pdu_lod_scale_x_minus1 = value;
  return *this;
}

constexpr auto PatchDataUnit::pdu_lod_scale_y_idc(unsigned value) noexcept -> auto & {
  m_pdu_lod_scale_y_idc = value;
  return *this;
}

constexpr auto PatchDataUnit::pdu_miv_extension() noexcept -> auto & {
  if (!m_pdu_miv_extension) {
    m_pdu_miv_extension = PduMivExtension{};
  }
  return *m_pdu_miv_extension;
}

constexpr auto PatchDataUnit::operator==(const PatchDataUnit &other) const noexcept {
  if (pdu_2d_pos_x() != other.pdu_2d_pos_x() || pdu_2d_pos_y() != other.pdu_2d_pos_y() ||
      pdu_2d_size_x_minus1() != other.pdu_2d_size_x_minus1() ||
      pdu_2d_size_y_minus1() != other.pdu_2d_size_y_minus1() ||
      pdu_3d_offset_u() != other.pdu_3d_offset_u() ||
      pdu_3d_offset_v() != other.pdu_3d_offset_v() ||
      pdu_3d_offset_d() != other.pdu_3d_offset_d() || m_pdu_3d_range_d != other.m_pdu_3d_range_d ||
      pdu_projection_id() != other.pdu_projection_id() ||
      pdu_orientation_index() != other.pdu_orientation_index() ||
      pdu_lod_enabled_flag() != other.pdu_lod_enabled_flag() ||
      m_pdu_miv_extension != other.m_pdu_miv_extension) {
    return false;
  }
  if (pdu_lod_enabled_flag()) {
    if ((pdu_lod_scale_x_minus1() != other.pdu_lod_scale_x_minus1() ||
         pdu_lod_scale_y_idc() != other.pdu_lod_scale_y_idc())) {
      return false;
    }
  }
  return true;
}

constexpr auto PatchDataUnit::operator!=(const PatchDataUnit &other) const noexcept {
  return !operator==(other);
}

constexpr auto PatchInformationData::data() const noexcept -> auto & { return m_data; }

template <typename Visitor> void AtlasTileDataUnit::visit(Visitor &&visitor) const {
  for (std::size_t p = 0; p < m_vector.size(); ++p) {
    visitor(p, m_vector[p].first, m_vector[p].second);
  }
}

constexpr auto AtlasTileLayerRBSP::atlas_tile_header() const noexcept -> auto & {
  return m_atlas_tile_header;
}

constexpr auto AtlasTileLayerRBSP::atlas_tile_data_unit() const noexcept -> auto & {
  return m_atlas_tile_data_unit;
}

constexpr auto AtlasTileLayerRBSP::atlas_tile_header() noexcept -> auto & {
  return m_atlas_tile_header;
}

constexpr auto AtlasTileLayerRBSP::atlas_tile_data_unit() noexcept -> auto & {
  return m_atlas_tile_data_unit;
}
} // namespace TMIV::MivBitstream
