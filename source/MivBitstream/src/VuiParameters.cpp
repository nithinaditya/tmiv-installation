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

#include <TMIV/MivBitstream/VuiParameters.h>

#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, const CoordinateSystemParameters &x) -> std::ostream & {
  stream << "cas_forward_axis=" << int{x.cas_forward_axis()} << '\n';
  stream << "cas_delta_left_axis_minus1=" << int{x.cas_delta_left_axis_minus1()} << '\n';
  stream << "cas_forward_sign=" << std::boolalpha << x.cas_forward_sign() << '\n';
  stream << "cas_left_sign=" << std::boolalpha << x.cas_left_sign() << '\n';
  stream << "cas_up_sign=" << std::boolalpha << x.cas_up_sign() << '\n';
  return stream;
}

auto CoordinateSystemParameters::decodeFrom(Common::InputBitstream &bitstream)
    -> CoordinateSystemParameters {
  auto x = CoordinateSystemParameters{};

  x.cas_forward_axis(bitstream.readBits<uint8_t>(2))
      .cas_delta_left_axis_minus1(bitstream.readBits<uint8_t>(1))
      .cas_forward_sign(bitstream.getFlag())
      .cas_left_sign(bitstream.getFlag())
      .cas_up_sign(bitstream.getFlag());

  return x;
}

void CoordinateSystemParameters::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.writeBits(cas_forward_axis(), 2);
  bitstream.writeBits(cas_delta_left_axis_minus1(), 1);
  bitstream.putFlag(cas_forward_sign());
  bitstream.putFlag(cas_left_sign());
  bitstream.putFlag(cas_up_sign());
}

auto VuiParameters::vui_num_units_in_tick() const noexcept -> std::uint32_t {
  VERIFY_V3CBITSTREAM(vui_timing_info_present_flag());
  VERIFY_V3CBITSTREAM(m_vui_num_units_in_tick.has_value());
  return *m_vui_num_units_in_tick;
}

auto VuiParameters::vui_time_scale() const noexcept -> std::uint32_t {
  VERIFY_V3CBITSTREAM(vui_timing_info_present_flag());
  VERIFY_V3CBITSTREAM(m_vui_time_scale.has_value());
  return *m_vui_time_scale;
}

auto VuiParameters::vui_poc_proportional_to_timing_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(vui_timing_info_present_flag());
  VERIFY_V3CBITSTREAM(m_vui_poc_proportional_to_timing_flag.has_value());
  return *m_vui_poc_proportional_to_timing_flag;
}

auto VuiParameters::vui_num_ticks_poc_diff_one_minus1() const noexcept -> std::uint32_t {
  VERIFY_V3CBITSTREAM(vui_poc_proportional_to_timing_flag());
  VERIFY_V3CBITSTREAM(m_vui_num_ticks_poc_diff_one_minus1.has_value());
  return *m_vui_num_ticks_poc_diff_one_minus1;
}

auto VuiParameters::vui_hrd_parameters_present_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(vui_timing_info_present_flag());
  VERIFY_V3CBITSTREAM(m_vui_hrd_parameters_present_flag.has_value());
  return *m_vui_hrd_parameters_present_flag;
}

auto VuiParameters::vui_tiles_fixed_structure_for_atlas_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(vui_bitstream_restriction_present_flag());
  VERIFY_V3CBITSTREAM(m_vui_tiles_fixed_structure_for_atlas_flag.has_value());
  return *m_vui_tiles_fixed_structure_for_atlas_flag;
}

auto VuiParameters::vui_tiles_fixed_structure_for_video_substreams_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(vui_bitstream_restriction_present_flag());
  VERIFY_V3CBITSTREAM(m_vui_tiles_fixed_structure_for_video_substreams_flag.has_value());
  return *m_vui_tiles_fixed_structure_for_video_substreams_flag;
}

auto VuiParameters::vui_constrained_tiles_across_v3c_components_idc() const noexcept
    -> std::uint8_t {
  VERIFY_V3CBITSTREAM(vui_bitstream_restriction_present_flag());
  VERIFY_V3CBITSTREAM(m_vui_constrained_tiles_across_v3c_components_idc.has_value());
  return *m_vui_constrained_tiles_across_v3c_components_idc;
}

auto VuiParameters::vui_max_num_tiles_per_atlas_minus1() const noexcept -> unsigned {
  VERIFY_V3CBITSTREAM(vui_bitstream_restriction_present_flag());
  VERIFY_V3CBITSTREAM(m_vui_max_num_tiles_per_atlas_minus1.has_value());
  return *m_vui_max_num_tiles_per_atlas_minus1;
}

auto VuiParameters::coordinate_system_parameters() const noexcept
    -> const CoordinateSystemParameters & {
  VERIFY_V3CBITSTREAM(vui_coordinate_system_parameters_present_flag());
  VERIFY_V3CBITSTREAM(m_coordinate_system_parameters.has_value());
  return *m_coordinate_system_parameters;
}

auto VuiParameters::vui_display_box_origin(int d) const noexcept -> std::uint32_t {
  VERIFY_V3CBITSTREAM(vui_display_box_info_present_flag());
  VERIFY_V3CBITSTREAM(m_vui_display_box_origin.has_value());
  return (*m_vui_display_box_origin)[d];
}

auto VuiParameters::vui_display_box_size(int d) const noexcept -> std::uint32_t {
  VERIFY_V3CBITSTREAM(vui_display_box_info_present_flag());
  VERIFY_V3CBITSTREAM(m_vui_display_box_size.has_value());
  return (*m_vui_display_box_size)[d];
}

auto VuiParameters::vui_anchor_point(int d) const noexcept -> std::uint32_t {
  VERIFY_V3CBITSTREAM(vui_anchor_point_present_flag());
  VERIFY_V3CBITSTREAM(m_vui_anchor_point.has_value());
  return (*m_vui_anchor_point)[d];
}

auto VuiParameters::vui_num_units_in_tick(std::uint32_t value) noexcept -> VuiParameters & {
  VERIFY_V3CBITSTREAM(vui_timing_info_present_flag());
  m_vui_num_units_in_tick = value;
  return *this;
}

auto VuiParameters::vui_time_scale(std::uint32_t value) noexcept -> VuiParameters & {
  VERIFY_V3CBITSTREAM(vui_timing_info_present_flag());
  m_vui_time_scale = value;
  return *this;
}

auto VuiParameters::vui_poc_proportional_to_timing_flag(bool value) noexcept -> VuiParameters & {
  VERIFY_V3CBITSTREAM(vui_timing_info_present_flag());
  m_vui_poc_proportional_to_timing_flag = value;
  return *this;
}

auto VuiParameters::vui_num_ticks_poc_diff_one_minus1(std::uint32_t value) noexcept
    -> VuiParameters & {
  VERIFY_V3CBITSTREAM(vui_poc_proportional_to_timing_flag());
  m_vui_num_ticks_poc_diff_one_minus1 = value;
  return *this;
}

auto VuiParameters::vui_hrd_parameters_present_flag(bool value) noexcept -> VuiParameters & {
  VERIFY_V3CBITSTREAM(vui_timing_info_present_flag());
  m_vui_hrd_parameters_present_flag = value;
  return *this;
}

auto VuiParameters::vui_tiles_fixed_structure_for_atlas_flag(bool value) noexcept
    -> VuiParameters & {
  VERIFY_V3CBITSTREAM(vui_bitstream_restriction_present_flag());
  m_vui_tiles_fixed_structure_for_atlas_flag = value;
  return *this;
}

auto VuiParameters::vui_tiles_fixed_structure_for_video_substreams_flag(bool value) noexcept
    -> VuiParameters & {
  VERIFY_V3CBITSTREAM(vui_bitstream_restriction_present_flag());
  m_vui_tiles_fixed_structure_for_video_substreams_flag = value;
  return *this;
}

auto VuiParameters::vui_constrained_tiles_across_v3c_components_idc(std::uint8_t value) noexcept
    -> VuiParameters & {
  VERIFY_V3CBITSTREAM(vui_bitstream_restriction_present_flag());
  m_vui_constrained_tiles_across_v3c_components_idc = value;
  return *this;
}

auto VuiParameters::vui_max_num_tiles_per_atlas_minus1(unsigned value) noexcept -> VuiParameters & {
  VERIFY_V3CBITSTREAM(vui_bitstream_restriction_present_flag());
  m_vui_max_num_tiles_per_atlas_minus1 = value;
  return *this;
}

auto VuiParameters::coordinate_system_parameters() noexcept -> CoordinateSystemParameters & {
  VERIFY_V3CBITSTREAM(vui_coordinate_system_parameters_present_flag());
  if (!m_coordinate_system_parameters) {
    m_coordinate_system_parameters = CoordinateSystemParameters{};
  }
  return *m_coordinate_system_parameters;
}

auto VuiParameters::vui_display_box_origin(int d, std::uint32_t value) noexcept -> VuiParameters & {
  VERIFY_V3CBITSTREAM(vui_display_box_info_present_flag());
  if (!m_vui_display_box_origin) {
    m_vui_display_box_origin = std::array<uint32_t, 3>{};
  }
  (*m_vui_display_box_origin)[d] = value;
  return *this;
}

auto VuiParameters::vui_display_box_size(int d, std::uint32_t value) noexcept -> VuiParameters & {
  VERIFY_V3CBITSTREAM(vui_display_box_info_present_flag());
  if (!m_vui_display_box_size) {
    m_vui_display_box_size = std::array<uint32_t, 3>{};
  }
  (*m_vui_display_box_size)[d] = value;
  return *this;
}

auto VuiParameters::vui_anchor_point(int d, std::uint32_t value) noexcept -> VuiParameters & {
  VERIFY_V3CBITSTREAM(vui_anchor_point_present_flag());
  if (!m_vui_anchor_point) {
    m_vui_anchor_point = std::array<uint32_t, 3>{};
  }
  (*m_vui_anchor_point)[d] = value;
  return *this;
}

auto operator<<(std::ostream &stream, const VuiParameters &x) -> std::ostream & {
  stream << "vui_timing_info_present_flag=" << std::boolalpha << x.vui_timing_info_present_flag()
         << '\n';
  if (x.vui_timing_info_present_flag()) {
    stream << "vui_num_units_in_tick=" << x.vui_num_units_in_tick() << '\n';
    stream << "vui_time_scale=" << x.vui_time_scale() << '\n';
    stream << "vui_poc_proportional_to_timing_flag=" << std::boolalpha
           << x.vui_poc_proportional_to_timing_flag() << '\n';
    if (x.vui_poc_proportional_to_timing_flag()) {
      stream << "vui_num_ticks_poc_diff_one_minus1=" << x.vui_num_ticks_poc_diff_one_minus1()
             << '\n';
    }
    stream << "vui_hrd_parameters_present_flag=" << std::boolalpha
           << x.vui_hrd_parameters_present_flag() << '\n';
    LIMITATION(!x.vui_hrd_parameters_present_flag());
  }

  stream << "vui_bitstream_restriction_present_flag=" << std::boolalpha
         << x.vui_bitstream_restriction_present_flag() << '\n';
  if (x.vui_bitstream_restriction_present_flag()) {
    stream << "vui_tiles_fixed_structure_for_atlas_flag=" << std::boolalpha
           << x.vui_tiles_fixed_structure_for_atlas_flag() << '\n';
    stream << "vui_tiles_fixed_structure_for_video_substreams_flag=" << std::boolalpha
           << x.vui_tiles_fixed_structure_for_video_substreams_flag() << '\n';
    stream << "vui_constrained_tiles_across_v3c_components_idc="
           << int{x.vui_constrained_tiles_across_v3c_components_idc()} << '\n';
    stream << "vui_max_num_tiles_per_atlas_minus1=" << x.vui_max_num_tiles_per_atlas_minus1()
           << '\n';
  }

  stream << "vui_coordinate_system_parameters_present_flag=" << std::boolalpha
         << x.vui_coordinate_system_parameters_present_flag() << '\n';
  if (x.vui_coordinate_system_parameters_present_flag()) {
    stream << x.coordinate_system_parameters();
  }

  stream << "vui_unit_in_metres_flag=" << std::boolalpha << x.vui_unit_in_metres_flag() << '\n';

  stream << "vui_display_box_info_present_flag=" << std::boolalpha
         << x.vui_display_box_info_present_flag() << '\n';
  if (x.vui_display_box_info_present_flag()) {
    for (int d = 0; d < 3; ++d) {
      stream << "vui_display_box_origin[ " << d << " ]=" << x.vui_display_box_origin(d) << '\n';
      stream << "vui_display_box_size[ " << d << " ]=" << x.vui_display_box_size(d) << '\n';
    }
  }

  stream << "vui_anchor_point_present_flag=" << std::boolalpha << x.vui_anchor_point_present_flag()
         << '\n';
  if (x.vui_anchor_point_present_flag()) {
    for (int d = 0; d < 3; ++d) {
      stream << "vui_anchor_point[ " << d << " ]=" << x.vui_anchor_point(d) << '\n';
    }
  }
  return stream;
}

auto VuiParameters::operator==(const VuiParameters &other) const noexcept -> bool {
  if (vui_timing_info_present_flag() != other.vui_timing_info_present_flag() ||
      vui_bitstream_restriction_present_flag() != other.vui_bitstream_restriction_present_flag() ||
      vui_coordinate_system_parameters_present_flag() !=
          other.vui_coordinate_system_parameters_present_flag() ||
      vui_unit_in_metres_flag() != other.vui_unit_in_metres_flag() ||
      vui_display_box_info_present_flag() != other.vui_display_box_info_present_flag() ||
      vui_anchor_point_present_flag() != other.vui_anchor_point_present_flag()) {
    return false;
  }

  LIMITATION(!vui_timing_info_present_flag() || !vui_hrd_parameters_present_flag());

  if (vui_timing_info_present_flag() &&
      (vui_num_units_in_tick() != other.vui_num_units_in_tick() ||
       vui_time_scale() != other.vui_time_scale() ||
       vui_poc_proportional_to_timing_flag() != other.vui_poc_proportional_to_timing_flag() ||
       vui_hrd_parameters_present_flag() != other.vui_hrd_parameters_present_flag())) {
    return false;
  }

  if (vui_timing_info_present_flag() && vui_poc_proportional_to_timing_flag() &&
      vui_num_ticks_poc_diff_one_minus1() != other.vui_num_ticks_poc_diff_one_minus1()) {
    return false;
  }

  if (vui_bitstream_restriction_present_flag() &&
      (vui_tiles_fixed_structure_for_atlas_flag() !=
           other.vui_tiles_fixed_structure_for_atlas_flag() ||
       vui_tiles_fixed_structure_for_video_substreams_flag() !=
           other.vui_tiles_fixed_structure_for_video_substreams_flag() ||
       vui_constrained_tiles_across_v3c_components_idc() !=
           other.vui_constrained_tiles_across_v3c_components_idc() ||
       vui_max_num_tiles_per_atlas_minus1() != other.vui_max_num_tiles_per_atlas_minus1())) {
    return false;
  }

  if (vui_coordinate_system_parameters_present_flag() &&
      coordinate_system_parameters() != other.coordinate_system_parameters()) {
    return false;
  }

  if (vui_display_box_info_present_flag()) {
    for (int d = 0; d < 3; ++d) {
      if ((vui_display_box_origin(d) != other.vui_display_box_origin(d) ||
           vui_display_box_size(d) != other.vui_display_box_size(d))) {
        return false;
      }
    }
  }

  if (vui_anchor_point_present_flag()) {
    for (int d = 0; d < 3; ++d) {
      if (vui_anchor_point(d) != other.vui_anchor_point(d)) {
        return false;
      }
    }
  }
  return true;
}

auto VuiParameters::operator!=(const VuiParameters &other) const noexcept -> bool {
  return !operator==(other);
}

auto VuiParameters::decodeFrom(Common::InputBitstream &bitstream,
                               const AtlasSequenceParameterSetRBSP *asps) -> VuiParameters {
  auto x = VuiParameters{};

  x.vui_timing_info_present_flag(bitstream.getFlag());
  if (x.vui_timing_info_present_flag()) {
    x.vui_num_units_in_tick(bitstream.getUint32());
    x.vui_time_scale(bitstream.getUint32());
    x.vui_poc_proportional_to_timing_flag(bitstream.getFlag());
    if (x.vui_poc_proportional_to_timing_flag()) {
      x.vui_num_ticks_poc_diff_one_minus1(bitstream.getUExpGolomb<uint32_t>());
    }
    x.vui_hrd_parameters_present_flag(bitstream.getFlag());
    LIMITATION(!x.vui_hrd_parameters_present_flag());
  }

  x.vui_bitstream_restriction_present_flag(bitstream.getFlag());
  if (x.vui_bitstream_restriction_present_flag()) {
    x.vui_tiles_fixed_structure_for_atlas_flag(bitstream.getFlag());
    x.vui_tiles_fixed_structure_for_video_substreams_flag(bitstream.getFlag());
    x.vui_constrained_tiles_across_v3c_components_idc(bitstream.getUExpGolomb<uint8_t>());
    x.vui_max_num_tiles_per_atlas_minus1(bitstream.getUExpGolomb<unsigned>());
  }

  x.vui_coordinate_system_parameters_present_flag(bitstream.getFlag());
  if (x.vui_coordinate_system_parameters_present_flag()) {
    x.coordinate_system_parameters() = CoordinateSystemParameters::decodeFrom(bitstream);
  }

  x.vui_unit_in_metres_flag(bitstream.getFlag());

  x.vui_display_box_info_present_flag(bitstream.getFlag());
  if (x.vui_display_box_info_present_flag()) {
    VERIFY_MIVBITSTREAM(asps != nullptr);
    for (int d = 0; d < 3; ++d) {
      x.vui_display_box_origin(
          d, bitstream.readBits<uint32_t>(asps->asps_geometry_3d_bit_depth_minus1() + 1));
      x.vui_display_box_size(
          d, bitstream.readBits<uint32_t>(asps->asps_geometry_3d_bit_depth_minus1() + 1));
    }
  }

  x.vui_anchor_point_present_flag(bitstream.getFlag());
  if (x.vui_anchor_point_present_flag()) {
    for (int d = 0; d < 3; ++d) {
      x.vui_anchor_point(
          d, bitstream.readBits<uint32_t>(asps->asps_geometry_3d_bit_depth_minus1() + 1));
    }
  }

  return x;
}

void VuiParameters::encodeTo(Common::OutputBitstream &bitstream,
                             const AtlasSequenceParameterSetRBSP *asps) const {
  bitstream.putFlag(vui_timing_info_present_flag());
  if (vui_timing_info_present_flag()) {
    bitstream.putUint32(vui_num_units_in_tick());
    bitstream.putUint32(vui_time_scale());
    bitstream.putFlag(vui_poc_proportional_to_timing_flag());
    if (vui_poc_proportional_to_timing_flag()) {
      bitstream.putUExpGolomb(vui_num_ticks_poc_diff_one_minus1());
    }
    bitstream.putFlag(vui_hrd_parameters_present_flag());
    LIMITATION(!vui_hrd_parameters_present_flag());
  }

  bitstream.putFlag(vui_bitstream_restriction_present_flag());
  if (vui_bitstream_restriction_present_flag()) {
    bitstream.putFlag(vui_tiles_fixed_structure_for_atlas_flag());
    bitstream.putFlag(vui_tiles_fixed_structure_for_video_substreams_flag());
    bitstream.putUExpGolomb(vui_constrained_tiles_across_v3c_components_idc());
    bitstream.putUExpGolomb(vui_max_num_tiles_per_atlas_minus1());
  }

  bitstream.putFlag(vui_coordinate_system_parameters_present_flag());
  if (vui_coordinate_system_parameters_present_flag()) {
    coordinate_system_parameters().encodeTo(bitstream);
  }

  bitstream.putFlag(vui_unit_in_metres_flag());

  bitstream.putFlag(vui_display_box_info_present_flag());
  if (vui_display_box_info_present_flag()) {
    VERIFY_MIVBITSTREAM(asps != nullptr); // ASPS parsing dependency
    for (int d = 0; d < 3; ++d) {
      bitstream.writeBits(vui_display_box_origin(d), asps->asps_geometry_3d_bit_depth_minus1() + 1);
      bitstream.writeBits(vui_display_box_size(d), asps->asps_geometry_3d_bit_depth_minus1() + 1);
    }
  }

  bitstream.putFlag(vui_anchor_point_present_flag());
  if (vui_anchor_point_present_flag()) {
    VERIFY_MIVBITSTREAM(asps != nullptr); // ASPS parsing dependency
    for (int d = 0; d < 3; ++d) {
      bitstream.writeBits(vui_anchor_point(d), asps->asps_geometry_3d_bit_depth_minus1() + 1);
    }
  }
}
} // namespace TMIV::MivBitstream
