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

#ifndef _TMIV_MIVBITSTREAM_V3CPARAMETERSET_H_
#define _TMIV_MIVBITSTREAM_V3CPARAMETERSET_H_

#include <TMIV/Common/Bitstream.h>
#include <TMIV/MivBitstream/Types.h>

#include <fmt/format.h>

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

namespace TMIV::MivBitstream {
enum class PtlProfileCodecGroupIdc : std::uint8_t {
  AVC_Progressive_High,
  HEVC_Main10,
  HEVC444,
  VVC_Main10,
  MP4RA = 127
};

enum class PtlProfilePccToolsetIdc : std::uint8_t {
  VPCC_Basic,
  VPCC_Extended,
  MIV_Main = 64,
  MIV_Extended,
  MIV_Geometry_Absent
};
enum class PtlProfileReconstructionIdc : std::uint8_t {
  Rec0, // (V-PCC)
  Rec1, // (V-PCC)
  Rec2, // (V-PCC)
  MIV_Main = 64,
  Rec_Unconstrained = 255
};

enum class PtlMaxDecodesIdc : std::uint8_t {
  max_1,
  max_2,
  max_3,
  max_4,
  max_6,
  max_12,
  max_16,
  max_24,
  max_32,
  unconstrained = 15
};
enum class PtlLevelIdc : std::uint8_t {
  Level_1_0 = 30,
  Level_1_5 = 45,
  Level_2_0 = 60,
  Level_2_5 = 75,
  Level_3_0 = 90,
  Level_3_5 = 105,
  Level_4_0 = 120,
  Level_4_5 = 135
};

enum class AiAttributeTypeId : std::uint8_t {
  ATTR_TEXTURE,
  ATTR_MATERIAL_ID,
  ATTR_TRANSPARENCY,
  ATTR_REFLECTANCE,
  ATTR_NORMAL,
  ATTR_UNSPECIFIED = 15
};

// Use a type to avoid confusing atlas index and ID
//
// NOTE(BK): The class interface deliberately disallows integer computations
class AtlasId {
public:
  constexpr AtlasId() noexcept = default;
  constexpr explicit AtlasId(std::uint8_t j) noexcept : m_atlasId{j} {}

  friend auto operator<<(std::ostream &stream, AtlasId atlasId) -> std::ostream &;

  constexpr auto operator==(AtlasId other) const noexcept { return m_atlasId == other.m_atlasId; }
  constexpr auto operator!=(AtlasId other) const noexcept { return m_atlasId != other.m_atlasId; }
  constexpr auto operator<(AtlasId other) const noexcept { return m_atlasId < other.m_atlasId; }

  static auto decodeFrom(Common::InputBitstream &bitstream) -> AtlasId;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  friend struct fmt::formatter<TMIV::MivBitstream::AtlasId>;

  std::uint8_t m_atlasId{};
};
} // namespace TMIV::MivBitstream

template <> struct fmt::formatter<TMIV::MivBitstream::AtlasId> {
  fmt::formatter<int> base;

  constexpr auto parse(format_parse_context &ctx) { return base.parse(ctx); }

  template <typename FormatContext>
  auto format(const TMIV::MivBitstream::AtlasId &id, FormatContext &ctx) {
    return base.format(id.m_atlasId, ctx);
  }
};

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, const PtlProfileCodecGroupIdc &x) -> std::ostream &;
auto operator<<(std::ostream &stream, const PtlProfilePccToolsetIdc &x) -> std::ostream &;
auto operator<<(std::ostream &stream, const PtlProfileReconstructionIdc &x) -> std::ostream &;
auto operator<<(std::ostream &stream, const PtlMaxDecodesIdc &x) -> std::ostream &;
auto operator<<(std::ostream &stream, const PtlLevelIdc &x) -> std::ostream &;
auto operator<<(std::ostream &stream, const AiAttributeTypeId &x) -> std::ostream &;

// TMIV-internal filename convention
auto codeOf(AiAttributeTypeId typeId) -> char;

// 23090-5: profile_toolset_constraints_information
//
// 23090-12 restrictions:
//   * ptc_num_reserved_constraint_bytes[ ] == 0
class ProfileToolsetConstraintsInformation {
public:
  [[nodiscard]] constexpr auto ptc_one_v3c_frame_only_flag() const noexcept;
  [[nodiscard]] constexpr auto ptc_eom_constraint_flag() const noexcept;
  [[nodiscard]] constexpr auto ptc_max_map_count_minus1() const noexcept;
  [[nodiscard]] constexpr auto ptc_max_atlas_count_minus1() const noexcept;
  [[nodiscard]] constexpr auto ptc_multiple_map_streams_constraint_flag() const noexcept;
  [[nodiscard]] constexpr auto ptc_plr_constraint_flag() const noexcept;
  [[nodiscard]] constexpr auto ptc_attribute_max_dimension_minus1() const noexcept;
  [[nodiscard]] constexpr auto ptc_attribute_max_dimension_partitions_minus1() const noexcept;
  [[nodiscard]] constexpr auto ptc_no_eight_orientations_constraint_flag() const noexcept;
  [[nodiscard]] constexpr auto ptc_no_45degree_projection_patch_constraint_flag() const noexcept;
  [[nodiscard]] constexpr auto ptc_restricted_geometry_flag() const noexcept;
  [[nodiscard]] constexpr auto ptc_num_reserved_constraint_bytes() const noexcept;

  constexpr auto ptc_one_v3c_frame_only_flag(bool value) noexcept -> auto &;
  constexpr auto ptc_eom_constraint_flag(bool value) noexcept -> auto &;
  constexpr auto ptc_max_map_count_minus1(std::uint8_t value) noexcept -> auto &;
  constexpr auto ptc_max_atlas_count_minus1(std::uint8_t value) noexcept -> auto &;
  constexpr auto ptc_multiple_map_streams_constraint_flag(bool value) noexcept -> auto &;
  constexpr auto ptc_plr_constraint_flag(bool value) noexcept -> auto &;
  constexpr auto ptc_attribute_max_dimension_minus1(std::uint8_t value) noexcept -> auto &;
  constexpr auto ptc_attribute_max_dimension_partitions_minus1(std::uint8_t value) noexcept
      -> auto &;
  constexpr auto ptc_no_eight_orientations_constraint_flag(bool value) noexcept -> auto &;
  constexpr auto ptc_no_45degree_projection_patch_constraint_flag(bool value) noexcept -> auto &;
  constexpr auto ptc_restricted_geometry_flag(bool value) noexcept -> auto &;
  constexpr auto ptc_num_reserved_constraint_bytes(std::uint8_t value) noexcept -> auto &;

  friend auto operator<<(std::ostream &stream, const ProfileToolsetConstraintsInformation &x)
      -> std::ostream &;

  auto operator==(const ProfileToolsetConstraintsInformation &other) const noexcept -> bool;
  auto operator!=(const ProfileToolsetConstraintsInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> ProfileToolsetConstraintsInformation;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  bool m_ptc_one_v3c_frame_only_flag{};
  bool m_ptc_eom_constraint_flag{};
  std::uint8_t m_ptc_max_map_count_minus1{};
  std::uint8_t m_ptc_max_atlas_count_minus1{};
  bool m_ptc_multiple_map_streams_constraint_flag{};
  bool m_ptc_plr_constraint_flag{};
  std::uint8_t m_ptc_attribute_max_dimension_minus1{};
  std::uint8_t m_ptc_attribute_max_dimension_partitions_minus1{};
  bool m_ptc_no_eight_orientations_constraint_flag{};
  bool m_ptc_no_45degree_projection_patch_constraint_flag{};
  bool m_ptc_restricted_geometry_flag{};
  std::uint8_t m_ptc_num_reserved_constraint_bytes{};
};

// 23090-5: profile_tier_level()
//
class ProfileTierLevel {
public:
  [[nodiscard]] constexpr auto ptl_tier_flag() const noexcept;
  [[nodiscard]] constexpr auto ptl_profile_codec_group_idc() const noexcept;
  [[nodiscard]] constexpr auto ptl_profile_toolset_idc() const noexcept;
  [[nodiscard]] constexpr auto ptl_profile_reconstruction_idc() const noexcept;
  [[nodiscard]] constexpr auto ptl_max_decodes_idc() const noexcept;
  [[nodiscard]] constexpr auto ptl_level_idc() const noexcept;
  [[nodiscard]] auto ptl_num_sub_profiles() const noexcept -> std::uint8_t;
  [[nodiscard]] constexpr auto ptl_extended_sub_profile_flag() const noexcept;
  [[nodiscard]] auto ptl_sub_profile_idc(std::uint8_t i) const noexcept -> std::uint64_t;
  [[nodiscard]] constexpr auto ptl_toolset_constraints_present_flag() const noexcept;
  [[nodiscard]] auto ptl_profile_toolset_constraints_information() const
      -> const ProfileToolsetConstraintsInformation &;

  constexpr auto ptl_tier_flag(bool value) noexcept -> auto &;
  constexpr auto ptl_profile_codec_group_idc(PtlProfileCodecGroupIdc value) noexcept -> auto &;
  constexpr auto ptl_profile_toolset_idc(PtlProfilePccToolsetIdc value) noexcept -> auto &;
  constexpr auto ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc value) noexcept
      -> auto &;
  constexpr auto ptl_max_decodes_idc(PtlMaxDecodesIdc value) noexcept -> auto &;
  constexpr auto ptl_level_idc(PtlLevelIdc value) noexcept -> auto &;
  auto ptl_num_sub_profiles(std::uint8_t value) noexcept -> ProfileTierLevel &;
  auto ptl_extended_sub_profile_flag(bool value) noexcept -> ProfileTierLevel &;
  auto ptl_sub_profile_idc(std::uint8_t i, std::uint64_t value) noexcept -> ProfileTierLevel &;
  constexpr auto ptl_toolset_constraints_present_flag(bool value) noexcept -> auto &;
  auto ptl_profile_toolset_constraints_information(ProfileToolsetConstraintsInformation value)
      -> ProfileTierLevel &;

  friend auto operator<<(std::ostream &stream, const ProfileTierLevel &x) -> std::ostream &;

  auto operator==(const ProfileTierLevel &other) const noexcept -> bool;
  auto operator!=(const ProfileTierLevel &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> ProfileTierLevel;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  bool m_ptl_tier_flag{};
  PtlProfileCodecGroupIdc m_ptl_profile_codec_group_idc{};
  PtlProfilePccToolsetIdc m_ptl_profile_toolset_idc{};
  PtlProfileReconstructionIdc m_ptl_profile_reconstruction_idc{};
  PtlMaxDecodesIdc m_ptl_max_decodes_idc{PtlMaxDecodesIdc::unconstrained};
  PtlLevelIdc m_ptl_level_idc{};
  std::vector<std::uint64_t> m_subProfileIdcs;
  bool m_ptl_extended_sub_profile_flag{};
  bool m_ptl_toolset_constraints_present_flag{};
  std::optional<ProfileToolsetConstraintsInformation>
      m_ptl_profile_toolset_constraints_information{};
};

// 23090-5: occupancy_information( atlasID )
class OccupancyInformation {
public:
  [[nodiscard]] constexpr auto oi_occupancy_codec_id() const noexcept;
  [[nodiscard]] constexpr auto oi_lossy_occupancy_compression_threshold() const noexcept;
  [[nodiscard]] constexpr auto oi_occupancy_2d_bit_depth_minus1() const noexcept;
  [[nodiscard]] constexpr auto oi_occupancy_MSB_align_flag() const noexcept;

  constexpr auto oi_occupancy_codec_id(std::uint8_t value) noexcept -> auto &;
  constexpr auto oi_lossy_occupancy_compression_threshold(std::uint8_t value) noexcept -> auto &;
  constexpr auto oi_occupancy_2d_bit_depth_minus1(std::uint8_t value) noexcept -> auto &;
  constexpr auto oi_occupancy_MSB_align_flag(bool value) noexcept -> auto &;

  auto printTo(std::ostream &stream, AtlasId atlasId) const -> std::ostream &;

  auto operator==(const OccupancyInformation &other) const noexcept -> bool;
  auto operator!=(const OccupancyInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> OccupancyInformation;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  std::uint8_t m_oi_occupancy_codec_id{};
  std::uint8_t m_oi_lossy_occupancy_compression_threshold{};
  std::uint8_t m_oi_occupancy_2d_bit_depth_minus1{};
  bool m_oi_occupancy_MSB_align_flag{};
};

class V3cParameterSet;

// 23090-5: geometry_information( atlasID )
//
// 23090-12 restrictions:
//   * vps_auxiliary_video_present_flag[ ] == 0
class GeometryInformation {
public:
  [[nodiscard]] constexpr auto gi_geometry_codec_id() const noexcept;
  [[nodiscard]] constexpr auto gi_geometry_2d_bit_depth_minus1() const noexcept;
  [[nodiscard]] constexpr auto gi_geometry_MSB_align_flag() const noexcept;
  [[nodiscard]] constexpr auto gi_geometry_3d_coordinates_bit_depth_minus1() const noexcept;

  constexpr auto gi_geometry_codec_id(std::uint8_t value) noexcept -> auto &;
  constexpr auto gi_geometry_2d_bit_depth_minus1(std::uint8_t value) noexcept -> auto &;
  constexpr auto gi_geometry_MSB_align_flag(bool value) noexcept -> auto &;
  constexpr auto gi_geometry_3d_coordinates_bit_depth_minus1(std::uint8_t value) noexcept -> auto &;

  auto printTo(std::ostream &stream, AtlasId atlasId) const -> std::ostream &;

  auto operator==(const GeometryInformation &other) const noexcept -> bool;
  auto operator!=(const GeometryInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps,
                         AtlasId atlasId) -> GeometryInformation;

  void encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps,
                AtlasId atlasId) const;

private:
  std::uint8_t m_gi_geometry_codec_id{};
  std::uint8_t m_gi_geometry_2d_bit_depth_minus1{};
  bool m_gi_geometry_MSB_align_flag{};
  std::uint8_t m_gi_geometry_3d_coordinates_bit_depth_minus1{};
};

// 23090-5: attribute_information( atlasId )
//
// 23090-12 restrictions:
//   * vps_auxiliary_video_present_flag[ ] == 0
//   * ai_attribute_dimension_partitions_minus1[ ][ ] == 0
class AttributeInformation {
public:
  [[nodiscard]] auto ai_attribute_count() const noexcept -> std::uint8_t;
  [[nodiscard]] auto ai_attribute_type_id(std::uint8_t attributeId) const -> AiAttributeTypeId;
  [[nodiscard]] auto ai_attribute_codec_id(std::uint8_t attributeId) const -> std::uint8_t;
  [[nodiscard]] auto
  ai_attribute_map_absolute_coding_persistence_flag(std::uint8_t attributeId) const -> bool;
  [[nodiscard]] auto ai_attribute_dimension_minus1(std::uint8_t attributeId) const -> std::uint8_t;
  [[nodiscard]] auto ai_attribute_2d_bit_depth_minus1(std::uint8_t attributeId) const
      -> std::uint8_t;
  [[nodiscard]] auto ai_attribute_MSB_align_flag(std::uint8_t attributeId) const -> bool;

  auto ai_attribute_count(std::uint8_t value) -> AttributeInformation &;
  auto ai_attribute_type_id(std::uint8_t attributeId, AiAttributeTypeId value)
      -> AttributeInformation &;
  auto ai_attribute_codec_id(std::uint8_t attributeId, std::uint8_t value)
      -> AttributeInformation &;
  auto ai_attribute_map_absolute_coding_persistence_flag(std::uint8_t attributeId, bool value)
      -> AttributeInformation &;
  auto ai_attribute_dimension_minus1(std::uint8_t attributeId, std::uint8_t value)
      -> AttributeInformation &;
  auto ai_attribute_2d_bit_depth_minus1(std::uint8_t attributeId, std::uint8_t value)
      -> AttributeInformation &;
  auto ai_attribute_MSB_align_flag(std::uint8_t attributeId, bool value) -> AttributeInformation &;

  auto printTo(std::ostream &stream, AtlasId atlasId) const -> std::ostream &;

  auto operator==(const AttributeInformation &other) const noexcept -> bool;
  auto operator!=(const AttributeInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps,
                         AtlasId atlasId) -> AttributeInformation;

  void encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps,
                AtlasId atlasId) const;

private:
  struct AiAttribute {
    AiAttributeTypeId ai_attribute_type_id{};
    std::uint8_t ai_attribute_codec_id{};
    std::optional<bool> ai_attribute_map_absolute_coding_persistence_flag{};
    std::uint8_t ai_attribute_dimension_minus1{};
    std::uint8_t ai_attribute_2d_bit_depth_minus1{};
    bool ai_attribute_MSB_align_flag{};
  };

  std::vector<AiAttribute> m_aiAttributes; // 23090-5: ai_attribute_count
};

struct PinRegion {
  auto operator==(const PinRegion &other) const noexcept -> bool {
    return (pin_region_tile_id == other.pin_region_tile_id) &&
           (pin_region_type_id_minus2 == other.pin_region_type_id_minus2) &&
           (pin_region_top_left_x == other.pin_region_top_left_x) &&
           (pin_region_top_left_y == other.pin_region_top_left_y) &&
           (pin_region_width_minus1 == other.pin_region_width_minus1) &&
           (pin_region_height_minus1 == other.pin_region_height_minus1) &&
           (pin_region_top_left_x == other.pin_region_unpack_top_left_y) &&
           (pin_region_unpack_top_left_y == other.pin_region_unpack_top_left_y) &&
           (pin_region_map_index == other.pin_region_map_index) &&
           (pin_region_rotation_flag == other.pin_region_rotation_flag) &&
           (pin_region_auxiliary_data_flag == other.pin_region_auxiliary_data_flag) &&
           (pin_region_attr_type_id == other.pin_region_attr_type_id) &&
           (pin_region_attr_partitions_flag == other.pin_region_attr_partitions_flag) &&
           (pin_region_attr_partition_index == other.pin_region_attr_partition_index) &&
           (pin_region_attr_partitions_minus1 == other.pin_region_attr_partitions_minus1);
  }

  std::uint8_t pin_region_tile_id{};
  VuhUnitType pin_region_type_id_minus2{};
  std::uint16_t pin_region_top_left_x{};
  std::uint16_t pin_region_top_left_y{};
  std::uint16_t pin_region_width_minus1{};
  std::uint16_t pin_region_height_minus1{};
  std::uint16_t pin_region_unpack_top_left_x{};
  std::uint16_t pin_region_unpack_top_left_y{};
  std::uint8_t pin_region_map_index{};
  bool pin_region_rotation_flag{};
  std::optional<bool> pin_region_auxiliary_data_flag{};
  std::optional<std::uint8_t> pin_region_attr_type_id{};
  std::optional<bool> pin_region_attr_partitions_flag{};
  std::optional<std::uint8_t> pin_region_attr_partition_index{};
  std::optional<std::uint8_t> pin_region_attr_partitions_minus1{};
};

// 23090-5: packing_information( j )
//
// 23090-12 restrictions:
//   * TODO I didn't see any restrictions, as this was entirely moved from 23090-12 to 23090-5
class PackingInformation {
public:
  [[nodiscard]] constexpr auto pin_codec_id() const noexcept -> std::uint8_t;
  [[nodiscard]] auto pin_regions_count_minus1() const -> std::size_t;
  [[nodiscard]] auto pin_region_tile_id(std::size_t i) const noexcept -> std::uint8_t;
  [[nodiscard]] auto pin_region_type_id_minus2(std::size_t i) const noexcept -> VuhUnitType;
  [[nodiscard]] auto pin_region_top_left_x(std::size_t i) const noexcept -> std::uint16_t;
  [[nodiscard]] auto pin_region_top_left_y(std::size_t i) const noexcept -> std::uint16_t;
  [[nodiscard]] auto pin_region_width_minus1(std::size_t i) const noexcept -> std::uint16_t;
  [[nodiscard]] auto pin_region_height_minus1(std::size_t i) const noexcept -> std::uint16_t;
  [[nodiscard]] auto pin_region_unpack_top_left_x(std::size_t i) const noexcept -> std::uint16_t;
  [[nodiscard]] auto pin_region_unpack_top_left_y(std::size_t i) const noexcept -> std::uint16_t;
  [[nodiscard]] auto pin_region_map_index(std::size_t i) const noexcept -> std::uint8_t;
  [[nodiscard]] auto pin_region_rotation_flag(std::size_t i) const noexcept -> bool;
  [[nodiscard]] auto pin_region_auxiliary_data_flag(std::size_t i) const -> bool;
  [[nodiscard]] auto pin_region_attr_type_id(std::size_t i) const -> std::uint8_t;
  [[nodiscard]] auto pin_region_attr_partitions_flag(std::size_t i) const -> bool;
  [[nodiscard]] auto pin_region_attr_partition_index(std::size_t i) const -> std::uint8_t;
  [[nodiscard]] auto pin_region_attr_partitions_minus1(std::size_t i) const -> std::uint8_t;

  constexpr auto pin_codec_id(std::uint8_t value) noexcept -> auto &;
  auto pin_regions_count_minus1(std::size_t value) -> auto &;
  auto pin_region_tile_id(std::size_t i, std::uint8_t value) -> auto &;
  auto pin_region_type_id_minus2(std::size_t i, VuhUnitType value) -> auto &;
  auto pin_region_top_left_x(std::size_t i, std::uint16_t value) -> auto &;
  auto pin_region_top_left_y(std::size_t i, std::uint16_t value) -> auto &;
  auto pin_region_width_minus1(std::size_t i, std::uint16_t value) -> auto &;
  auto pin_region_height_minus1(std::size_t i, std::uint16_t value) -> auto &;
  auto pin_region_unpack_top_left_x(std::size_t i, std::uint16_t value) -> auto &;
  auto pin_region_unpack_top_left_y(std::size_t i, std::uint16_t value) -> auto &;
  auto pin_region_map_index(std::size_t i, std::uint8_t value) -> auto &;
  auto pin_region_rotation_flag(std::size_t i, bool value) -> auto &;
  auto pin_region_auxiliary_data_flag(std::size_t i, bool value) -> auto &;
  auto pin_region_attr_type_id(std::size_t i, std::uint8_t value) -> auto &;
  auto pin_region_attr_partitions_flag(std::size_t i, bool value) -> auto &;
  auto pin_region_attr_partition_index(std::size_t i, std::uint8_t value) -> auto &;
  auto pin_region_attr_partitions_minus1(std::size_t i, std::uint8_t value) -> auto &;

  auto printTo(std::ostream &stream, const AtlasId &j) const -> std::ostream &;

  auto operator==(const PackingInformation &other) const noexcept -> bool;
  auto operator!=(const PackingInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> PackingInformation;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  std::uint8_t m_pin_codec_id{};
  std::vector<PinRegion> m_pinRegions{std::vector<PinRegion>(1U)};
};

// 23090-12: group_mapping()
class GroupMapping {
public:
  [[nodiscard]] constexpr auto gm_group_count() const noexcept;
  [[nodiscard]] auto gm_group_id(std::size_t i) const noexcept -> std::uint8_t;

  constexpr auto gm_group_count(std::uint8_t value) noexcept -> auto &;
  auto gm_group_id(std::size_t i, std::uint8_t value) noexcept -> GroupMapping &;

  friend auto operator<<(std::ostream &stream, const GroupMapping &x) -> std::ostream &;

  auto operator==(const GroupMapping &other) const noexcept -> bool;
  auto operator!=(const GroupMapping &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps)
      -> GroupMapping;
  void encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps) const;

private:
  std::uint8_t m_gm_group_count{};
  std::vector<std::uint8_t> m_gm_group_id;
};

// 23090-12: vps_miv_extension()
class VpsMivExtension {
public:
  [[nodiscard]] constexpr auto vme_geometry_scale_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto vme_embedded_occupancy_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto vme_occupancy_scale_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto group_mapping() const noexcept -> const GroupMapping &;

  constexpr auto vme_geometry_scale_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto vme_embedded_occupancy_enabled_flag(bool value) noexcept -> auto &;
  auto vme_occupancy_scale_enabled_flag(bool value) noexcept -> VpsMivExtension &;

  [[nodiscard]] constexpr auto group_mapping() noexcept -> GroupMapping &;

  friend auto operator<<(std::ostream &stream, const VpsMivExtension &x) -> std::ostream &;

  constexpr auto operator==(const VpsMivExtension &other) const noexcept;
  constexpr auto operator!=(const VpsMivExtension &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps)
      -> VpsMivExtension;
  void encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps) const;

private:
  bool m_vme_geometry_scale_enabled_flag{};
  bool m_vme_embedded_occupancy_enabled_flag{true};
  bool m_vme_occupancy_scale_enabled_flag{};
  GroupMapping m_group_mapping;
};

// 23090-5: v3c_parameter_set()
//
// 23090-12 restrictions:
//   * vps_multiple_map_streams_present_flag[ ] == 0
class V3cParameterSet {
public:
  [[nodiscard]] auto profile_tier_level() const noexcept -> const ProfileTierLevel &;
  [[nodiscard]] constexpr auto vps_v3c_parameter_set_id() const noexcept;
  [[nodiscard]] auto vps_atlas_count_minus1() const noexcept -> std::uint8_t;
  [[nodiscard]] auto vps_atlas_id(size_t k) const -> AtlasId;
  [[nodiscard]] auto vps_frame_width(AtlasId j) const -> std::uint16_t;
  [[nodiscard]] auto vps_frame_height(AtlasId j) const -> std::uint16_t;
  [[nodiscard]] auto vps_map_count_minus1(AtlasId j) const -> std::uint8_t;
  [[nodiscard]] auto vps_auxiliary_video_present_flag(AtlasId j) const -> bool;
  [[nodiscard]] auto vps_occupancy_video_present_flag(AtlasId j) const -> bool;
  [[nodiscard]] auto vps_geometry_video_present_flag(AtlasId j) const -> bool;
  [[nodiscard]] auto vps_attribute_video_present_flag(AtlasId j) const -> bool;
  [[nodiscard]] auto occupancy_information(AtlasId j) const -> const OccupancyInformation &;
  [[nodiscard]] auto geometry_information(AtlasId j) const -> const GeometryInformation &;
  [[nodiscard]] auto attribute_information(AtlasId j) const -> const AttributeInformation &;
  [[nodiscard]] constexpr auto vps_extension_present_flag() const noexcept;
  [[nodiscard]] constexpr auto vps_packing_information_present_flag() const noexcept;
  [[nodiscard]] constexpr auto vps_miv_extension_present_flag() const noexcept;
  [[nodiscard]] constexpr auto vps_extension_6bits() const noexcept;
  [[nodiscard]] auto vps_packed_video_present_flag(const AtlasId &j) const;
  [[nodiscard]] auto packing_information(const AtlasId &j) const;
  [[nodiscard]] auto vps_miv_extension() const noexcept -> const VpsMivExtension &;
  [[nodiscard]] auto vps_extension_length_minus1() const noexcept -> std::size_t;
  [[nodiscard]] auto vpsExtensionData() const noexcept -> const std::vector<std::uint8_t> &;

  auto profile_tier_level(ProfileTierLevel value) noexcept -> V3cParameterSet &;
  constexpr auto vps_v3c_parameter_set_id(std::uint8_t value) noexcept -> auto &;
  auto vps_atlas_count_minus1(std::uint8_t value) -> V3cParameterSet &;
  auto vps_atlas_id(size_t k, AtlasId value) -> V3cParameterSet &;
  auto vps_frame_width(AtlasId j, std::uint16_t value) -> V3cParameterSet &;
  auto vps_frame_height(AtlasId j, std::uint16_t value) -> V3cParameterSet &;
  auto vps_map_count_minus1(AtlasId j, std::uint8_t value) -> V3cParameterSet &;
  auto vps_auxiliary_video_present_flag(AtlasId j, bool value) -> V3cParameterSet &;
  auto vps_occupancy_video_present_flag(AtlasId j, bool value) -> V3cParameterSet &;
  auto vps_geometry_video_present_flag(AtlasId j, bool value) -> V3cParameterSet &;
  auto vps_attribute_video_present_flag(AtlasId j, bool value) -> V3cParameterSet &;
  auto occupancy_information(AtlasId j, OccupancyInformation value) -> V3cParameterSet &;
  auto geometry_information(AtlasId j, GeometryInformation value) -> V3cParameterSet &;
  auto attribute_information(AtlasId j, AttributeInformation value) -> V3cParameterSet &;
  constexpr auto vps_extension_present_flag(bool value) noexcept -> auto &;
  auto vps_packing_information_present_flag(bool value) noexcept -> V3cParameterSet &;
  auto vps_miv_extension_present_flag(bool value) noexcept -> V3cParameterSet &;
  auto vps_extension_6bits(std::uint8_t value) noexcept -> V3cParameterSet &;
  auto vps_packed_video_present_flag(const AtlasId &j, bool value) -> V3cParameterSet &;
  auto packing_information(const AtlasId &j, PackingInformation value) -> V3cParameterSet &;
  auto vps_miv_extension(const VpsMivExtension &value) noexcept -> V3cParameterSet &;
  auto vpsExtensionData(std::vector<std::uint8_t> value) noexcept -> V3cParameterSet &;

  constexpr auto profile_tier_level() noexcept -> auto &;
  [[nodiscard]] auto occupancy_information(AtlasId j) -> OccupancyInformation &;
  [[nodiscard]] auto geometry_information(AtlasId j) -> GeometryInformation &;
  [[nodiscard]] auto attribute_information(AtlasId j) -> AttributeInformation &;
  [[nodiscard]] auto vps_miv_extension() noexcept -> VpsMivExtension &;

  // Convenience function
  [[nodiscard]] auto indexOf(AtlasId atlasId) const noexcept -> std::size_t;

  friend auto operator<<(std::ostream &stream, const V3cParameterSet &x) -> std::ostream &;

  auto operator==(const V3cParameterSet &other) const noexcept -> bool;
  auto operator!=(const V3cParameterSet &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> V3cParameterSet;

  void encodeTo(std::ostream &stream) const;

private:
  struct VpsAtlas {
    AtlasId vps_atlas_id{};
    std::uint16_t vps_frame_width{};
    std::uint16_t vps_frame_height{};
    std::uint8_t vps_map_count_minus1{};
    bool vps_auxiliary_video_present_flag{};
    bool vps_occupancy_video_present_flag{};
    bool vps_geometry_video_present_flag{};
    bool vps_attribute_video_present_flag{};
    bool vps_packed_video_present_flag{};
    std::optional<OccupancyInformation> occupancy_information{};
    std::optional<GeometryInformation> geometry_information{};
    std::optional<AttributeInformation> attribute_information{};
    std::optional<PackingInformation> packing_information{};
  };

  [[nodiscard]] auto atlas(AtlasId atlasId) const noexcept -> const VpsAtlas &;
  [[nodiscard]] auto atlas(AtlasId atlasId) noexcept -> VpsAtlas &;

  ProfileTierLevel m_profile_tier_level;
  std::uint8_t m_vps_v3c_parameter_set_id{};
  std::vector<VpsAtlas> m_vpsAtlases{VpsAtlas{}};
  bool m_vps_extension_present_flag{};
  std::optional<bool> m_vps_packing_information_present_flag{};
  std::optional<bool> m_vps_miv_extension_present_flag{};
  std::optional<std::uint8_t> m_vps_extension_7bits{};
  std::optional<VpsMivExtension> m_vps_miv_extension;
  std::optional<std::vector<std::uint8_t>> m_vpsExtensionData;
};
} // namespace TMIV::MivBitstream

#include "V3cParameterSet.hpp"

#endif
