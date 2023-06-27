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
#define _TMIV_MIVBITSTREAM_ATLASTILELAYERRBSP_H_

#include <TMIV/Common/Bitstream.h>
#include <TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/V3cParameterSet.h>
#include <TMIV/MivBitstream/V3cUnit.h>

#include <TMIV/Common/Vector.h>

#include <cstdint>
#include <cstdlib>
#include <iosfwd>
#include <optional>
#include <variant>

namespace TMIV::MivBitstream {
enum class AthType : std::uint8_t { P_TILE, I_TILE, SKIP_TILE };

enum class FlexiblePatchOrientation : std::uint8_t {
  FPO_NULL,
  FPO_SWAP,
  FPO_MROT270 = FPO_SWAP,
  FPO_ROT90,
  FPO_ROT180,
  FPO_ROT270,
  FPO_MIRROR,
  FPO_MROT90,
  FPO_MROT180,
  FPO_INVALID = UINT8_MAX
};

enum class AtduPatchMode : std::uint8_t {
  I_INTRA,
  I_RAW,
  I_EOM,
  I_END = 14,
  P_SKIP = I_INTRA,
  P_MERGE,
  P_INTER,
  P_INTRA,
  P_RAW,
  P_EOM,
  P_END = I_END
};

auto operator<<(std::ostream &stream, AthType x) -> std::ostream &;
auto operator<<(std::ostream &stream, FlexiblePatchOrientation x) -> std::ostream &;
auto printTo(std::ostream &stream, AtduPatchMode x, AthType ath_type) -> std::ostream &;

// 23090-5: atlas_tile_header( )
//
// 23090-12 restrictions:
//   * asps_long_term_ref_atlas_frames_flag == 0
//   * afps_raw_3d_offset_bit_count_explicit_mode_flag == 0
//   * ath_type in { I_TILE, SKIP_TILE }
//
// Limitations of the implementation:
//   * asps_num_ref_atlas_frame_lists_in_asps == 1
//   * ath_ref_atlas_frame_list_asps_flag == 1
class AtlasTileHeader {
public:
  [[nodiscard]] constexpr auto ath_no_output_of_prior_atlas_frames_flag() const noexcept;
  [[nodiscard]] constexpr auto ath_atlas_frame_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto ath_atlas_adaptation_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto ath_id() const noexcept;
  [[nodiscard]] constexpr auto ath_type() const noexcept;
  [[nodiscard]] auto ath_atlas_output_flag() const noexcept -> bool;
  [[nodiscard]] constexpr auto ath_atlas_frm_order_cnt_lsb() const noexcept;
  [[nodiscard]] constexpr auto ath_ref_atlas_frame_list_asps_flag() const noexcept;
  [[nodiscard]] constexpr auto ath_pos_min_d_quantizer() const noexcept;
  [[nodiscard]] constexpr auto ath_pos_delta_max_d_quantizer() const noexcept;
  [[nodiscard]] auto ath_patch_size_x_info_quantizer() const noexcept -> std::uint8_t;
  [[nodiscard]] auto ath_patch_size_y_info_quantizer() const noexcept -> std::uint8_t;

  constexpr auto ath_no_output_of_prior_atlas_frames_flag(bool value) noexcept -> auto &;
  constexpr auto ath_atlas_frame_parameter_set_id(const std::uint8_t value) noexcept -> auto &;
  constexpr auto ath_atlas_adaptation_parameter_set_id(const std::uint8_t value) noexcept -> auto &;
  constexpr auto ath_id(const std::uint8_t value) noexcept -> auto &;
  constexpr auto ath_type(const AthType value) noexcept -> auto &;
  constexpr auto ath_atlas_output_flag(const bool value) noexcept -> auto &;
  constexpr auto ath_pos_min_d_quantizer(const std::uint8_t value) noexcept -> auto &;
  constexpr auto ath_pos_delta_max_d_quantizer(const std::uint8_t value) noexcept -> auto &;
  constexpr auto ath_atlas_frm_order_cnt_lsb(const std::uint16_t value) noexcept -> auto &;
  constexpr auto ath_ref_atlas_frame_list_asps_flag(const bool value) noexcept -> auto &;
  auto ath_patch_size_x_info_quantizer(const std::uint8_t value) noexcept -> AtlasTileHeader &;
  auto ath_patch_size_y_info_quantizer(const std::uint8_t value) noexcept -> AtlasTileHeader &;

  friend auto operator<<(std::ostream &stream, const AtlasTileHeader &x) -> std::ostream &;

  constexpr auto operator==(const AtlasTileHeader &other) const noexcept;
  constexpr auto operator!=(const AtlasTileHeader &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream, const NalUnitHeader &nuh,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsV) -> AtlasTileHeader;

  void encodeTo(Common::OutputBitstream &bitstream, const NalUnitHeader &nuh,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsV) const;

private:
  std::optional<bool> m_ath_no_output_of_prior_atlas_frames_flag{};
  std::uint8_t m_ath_atlas_frame_parameter_set_id{};
  std::uint8_t m_ath_adaptation_parameter_set_id{};
  std::uint8_t m_ath_id{};
  AthType m_ath_type{};
  std::optional<bool> m_ath_atlas_output_flag{};
  std::uint16_t m_ath_atlas_frm_order_cnt_lsb{};
  std::optional<bool> m_ath_ref_atlas_frame_list_asps_flag{};
  std::optional<std::uint8_t> m_ath_pos_min_d_quantizer{};
  std::optional<std::uint8_t> m_ath_pos_delta_max_d_quantizer{};
  std::optional<std::uint8_t> m_ath_patch_size_x_info_quantizer{};
  std::optional<std::uint8_t> m_ath_patch_size_y_info_quantizer{};
};

// 23090-5: skip_patch_data_unit( patchIdx )
class SkipPatchDataUnit {
public:
  friend auto operator<<(std::ostream &stream, const SkipPatchDataUnit &x) -> std::ostream &;

  constexpr auto operator==(const SkipPatchDataUnit &other) const noexcept;
  constexpr auto operator!=(const SkipPatchDataUnit &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> SkipPatchDataUnit;

  void encodeTo(Common::OutputBitstream &bitstream) const;
};

// 23090-12: pdu_miv_extension( patchIdx )
class PduMivExtension {
public:
  [[nodiscard]] constexpr auto pdu_entity_id() const noexcept;
  [[nodiscard]] auto pdu_depth_occ_threshold() const -> std::uint32_t;
  [[nodiscard]] auto pdu_attribute_offset() const -> Common::Vec3w;
  [[nodiscard]] constexpr auto pdu_inpaint_flag() const noexcept;

  constexpr auto pdu_entity_id(std::uint32_t value) noexcept -> auto &;
  constexpr auto pdu_depth_occ_threshold(std::uint32_t value) noexcept -> auto &;
  auto pdu_attribute_offset(Common::Vec3w value) noexcept -> auto &;
  constexpr auto pdu_inpaint_flag(bool value) noexcept -> auto &;

  auto printTo(std::ostream &stream, unsigned tileId, std::size_t patchIdx) const -> std::ostream &;

  auto operator==(const PduMivExtension &other) const noexcept -> bool;
  auto operator!=(const PduMivExtension &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const AtlasSequenceParameterSetRBSP &asps) -> PduMivExtension;

  void encodeTo(Common::OutputBitstream &bitstream,
                const AtlasSequenceParameterSetRBSP &asps) const;

private:
  std::optional<std::uint32_t> m_pdu_entity_id;
  std::optional<std::uint32_t> m_pdu_depth_occ_threshold;
  std::optional<Common::Vec3w> m_pdu_attribute_offset;
  std::optional<bool> m_pdu_inpaint_flag;
};

// 23090-12: patch_data_unit( patchIdx )
//
// 23090-12 limitations:
//   * asps_plr_enabled_flag == 0
class PatchDataUnit {
public:
  [[nodiscard]] constexpr auto pdu_2d_pos_x() const noexcept;
  [[nodiscard]] constexpr auto pdu_2d_pos_y() const noexcept;
  [[nodiscard]] constexpr auto pdu_2d_size_x_minus1() const noexcept;
  [[nodiscard]] constexpr auto pdu_2d_size_y_minus1() const noexcept;
  [[nodiscard]] constexpr auto pdu_3d_offset_u() const noexcept;
  [[nodiscard]] constexpr auto pdu_3d_offset_v() const noexcept;
  [[nodiscard]] constexpr auto pdu_3d_offset_d() const noexcept;
  [[nodiscard]] auto pdu_3d_range_d() const -> std::uint32_t;
  [[nodiscard]] constexpr auto pdu_projection_id() const noexcept;
  [[nodiscard]] constexpr auto pdu_orientation_index() const noexcept;
  [[nodiscard]] constexpr auto pdu_lod_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto pdu_lod_scale_x_minus1() const noexcept;
  [[nodiscard]] constexpr auto pdu_lod_scale_y_idc() const noexcept;
  [[nodiscard]] auto pdu_miv_extension() const noexcept -> PduMivExtension;

  constexpr auto pdu_2d_pos_x(std::uint32_t value) noexcept -> auto &;
  constexpr auto pdu_2d_pos_y(std::uint32_t value) noexcept -> auto &;
  constexpr auto pdu_2d_size_x_minus1(std::uint32_t value) noexcept -> auto &;
  constexpr auto pdu_2d_size_y_minus1(std::uint32_t value) noexcept -> auto &;
  constexpr auto pdu_3d_offset_u(std::uint32_t value) noexcept -> auto &;
  constexpr auto pdu_3d_offset_v(std::uint32_t value) noexcept -> auto &;
  constexpr auto pdu_3d_offset_d(std::uint32_t value) noexcept -> auto &;
  constexpr auto pdu_3d_range_d(std::uint32_t value) noexcept -> auto &;
  constexpr auto pdu_projection_id(std::uint16_t value) noexcept -> auto &;
  constexpr auto pdu_orientation_index(FlexiblePatchOrientation value) noexcept -> auto &;
  constexpr auto pdu_lod_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto pdu_lod_scale_x_minus1(unsigned value) noexcept -> auto &;
  constexpr auto pdu_lod_scale_y_idc(unsigned value) noexcept -> auto &;
  auto pdu_miv_extension(const PduMivExtension &value) noexcept -> PatchDataUnit &;

  [[nodiscard]] constexpr auto pdu_miv_extension() noexcept -> auto &;

  auto printTo(std::ostream &stream, unsigned tileId, std::size_t patchIdx) const -> std::ostream &;

  constexpr auto operator==(const PatchDataUnit &other) const noexcept;
  constexpr auto operator!=(const PatchDataUnit &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsVector,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsVector,
                         const AtlasTileHeader &ath) -> PatchDataUnit;

  void encodeTo(Common::OutputBitstream &bitstream,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsVector,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsVector,
                const AtlasTileHeader &ath) const;

private:
  std::uint32_t m_pdu_2d_pos_x{};
  std::uint32_t m_pdu_2d_pos_y{};
  std::uint32_t m_pdu_2d_size_x_minus1{};
  std::uint32_t m_pdu_2d_size_y_minus1{};
  std::uint32_t m_pdu_3d_offset_u{};
  std::uint32_t m_pdu_3d_offset_v{};
  std::uint32_t m_pdu_3d_offset_d{};
  std::optional<std::uint32_t> m_pdu_3d_range_d{};
  std::uint16_t m_pdu_view_id{};
  FlexiblePatchOrientation m_pdu_orientation_index{};
  std::optional<bool> m_pdu_lod_enabled_flag{};
  std::optional<unsigned> m_pdu_lod_scale_x_minus1{};
  std::optional<unsigned> m_pdu_lod_scale_y_idc{};
  std::optional<PduMivExtension> m_pdu_miv_extension;
};

// 23090-5: patch_information_data( )
//
// 23090-12 restrictions:
//   * ath_type in { I_TILE, SKIP_TILE }
//   * patchMode in { I_INTRA, I_END }
class PatchInformationData {
public:
  using Data = std::variant<std::monostate, SkipPatchDataUnit, PatchDataUnit>;

  PatchInformationData() = default;

  template <typename Value>
  constexpr explicit PatchInformationData(Value &&value) : m_data{std::forward<Value>(value)} {}

  [[nodiscard]] constexpr auto data() const noexcept -> auto &;

  [[nodiscard]] auto skip_patch_data_unit() const noexcept -> const SkipPatchDataUnit &;
  [[nodiscard]] auto patch_data_unit() const noexcept -> const PatchDataUnit &;

  auto printTo(std::ostream &stream, unsigned tileId, std::size_t patchIdx) const -> std::ostream &;

  auto operator==(const PatchInformationData &other) const noexcept -> bool;
  auto operator!=(const PatchInformationData &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                         const AtlasTileHeader &ath, AtduPatchMode patchMode)
      -> PatchInformationData;

  void encodeTo(Common::OutputBitstream &bitstream,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsV, const AtlasTileHeader &ath,
                AtduPatchMode patchMode) const;

private:
  Data m_data;
};

// 23090-5: atlas_tile_data_unit( )
class AtlasTileDataUnit {
public:
  using Vector = std::vector<std::pair<AtduPatchMode, PatchInformationData>>;

  AtlasTileDataUnit() = default;

  template <typename... Args>
  explicit AtlasTileDataUnit(Args &&...args) : m_vector{std::forward<Args>(args)...} {}

  [[nodiscard]] auto atduTotalNumberOfPatches() const noexcept -> std::size_t;
  [[nodiscard]] auto atdu_patch_mode(std::size_t p) const -> AtduPatchMode;
  [[nodiscard]] auto patch_information_data(std::size_t p) const -> const PatchInformationData &;

  // Visit all elements in the atlas tile data unit in ascending order. The expected signature of
  // the visitor is: void(std::size_t p, AtduPatchMode, const PatchInformationData &)
  template <typename Visitor> void visit(Visitor &&visitor) const;

  auto printTo(std::ostream &stream, const AtlasTileHeader &ath) const -> std::ostream &;

  auto operator==(const AtlasTileDataUnit &other) const -> bool;
  auto operator!=(const AtlasTileDataUnit &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                         const AtlasTileHeader &ath) -> AtlasTileDataUnit;

  void encodeTo(Common::OutputBitstream &bitstream,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                const AtlasTileHeader &ath) const;

private:
  Vector m_vector;
};

// 23090-5: atlas_tile_layer_rbsp( )
class AtlasTileLayerRBSP {
public:
  [[nodiscard]] constexpr auto atlas_tile_header() const noexcept -> auto &;
  [[nodiscard]] constexpr auto atlas_tile_data_unit() const noexcept -> auto &;

  [[nodiscard]] constexpr auto atlas_tile_header() noexcept -> auto &;
  [[nodiscard]] constexpr auto atlas_tile_data_unit() noexcept -> auto &;

  friend auto operator<<(std::ostream &stream, const AtlasTileLayerRBSP &x) -> std::ostream &;

  auto operator==(const AtlasTileLayerRBSP &other) const noexcept -> bool;
  auto operator!=(const AtlasTileLayerRBSP &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream, const NalUnitHeader &nuh,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsV)
      -> AtlasTileLayerRBSP;

  void encodeTo(std::ostream &stream, const NalUnitHeader &nuh,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsV) const;

private:
  AtlasTileHeader m_atlas_tile_header;
  AtlasTileDataUnit m_atlas_tile_data_unit;
};
} // namespace TMIV::MivBitstream

#include "AtlasTileLayerRBSP.hpp"

#endif
