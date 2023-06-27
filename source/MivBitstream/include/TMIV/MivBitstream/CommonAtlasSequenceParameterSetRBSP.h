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

#ifndef _TMIV_MIVBITSTREAM_COMMONATLASSEQUENCEPARAMETERSETRBSP_H_
#define _TMIV_MIVBITSTREAM_COMMONATLASSEQUENCEPARAMETERSETRBSP_H_

#include <optional>
#include <vector>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/MivBitstream/VuiParameters.h>

namespace TMIV::MivBitstream {
// 23090-12: casps_miv_extension( )
class CaspsMivExtension {
public:
  [[nodiscard]] constexpr auto casme_omaf_v1_compatible_flag() const noexcept;
  [[nodiscard]] constexpr auto casme_depth_low_quality_flag() const noexcept;
  [[nodiscard]] constexpr auto casme_depth_quantization_params_present_flag() const noexcept;
  [[nodiscard]] constexpr auto casme_vui_params_present_flag() const noexcept;
  [[nodiscard]] auto vui_parameters() const noexcept -> const VuiParameters &;

  constexpr auto casme_omaf_v1_compatible_flag(bool value) noexcept -> auto &;
  constexpr auto casme_depth_low_quality_flag(bool value) noexcept -> auto &;
  constexpr auto casme_depth_quantization_params_present_flag(bool value) noexcept -> auto &;
  constexpr auto casme_vui_params_present_flag(bool value) noexcept -> auto &;
  auto vui_parameters(const VuiParameters &value) noexcept -> CaspsMivExtension &;

  friend auto operator<<(std::ostream &stream, const CaspsMivExtension &x) -> std::ostream &;

  auto operator==(const CaspsMivExtension &) const noexcept -> bool;
  auto operator!=(const CaspsMivExtension &) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &stream) -> CaspsMivExtension;

  void encodeTo(Common::OutputBitstream &stream) const;

private:
  bool m_casme_omaf_v1_compatible_flag{};
  bool m_casme_depth_low_quality_flag{};
  bool m_casme_depth_quantization_params_present_flag{};
  bool m_casme_vui_params_present_flag{};
  std::optional<VuiParameters> m_vui_parameters;
};

// 23090-5 common_atlas_sequence_parameter_set_rbsp()
class CommonAtlasSequenceParameterSetRBSP {
public:
  [[nodiscard]] constexpr auto casps_common_atlas_sequence_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto
  casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4() const noexcept;
  [[nodiscard]] constexpr auto casps_extension_present_flag() const noexcept;
  [[nodiscard]] auto casps_miv_extension_present_flag() const noexcept -> bool;
  [[nodiscard]] auto casps_extension_7bits() const noexcept;
  [[nodiscard]] auto casps_miv_extension() const noexcept -> const CaspsMivExtension &;
  [[nodiscard]] auto caspsExtensionData() const noexcept
      -> const std::vector<bool> &; // to access multiple casps_extension_data_flags

  constexpr auto casps_common_atlas_sequence_parameter_set_id(std::uint8_t value) noexcept
      -> auto &;
  constexpr auto casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4(std::uint8_t value) noexcept
      -> auto &;
  constexpr auto casps_extension_present_flag(bool flag) noexcept -> auto &;
  auto casps_miv_extension_present_flag(bool flag) noexcept
      -> CommonAtlasSequenceParameterSetRBSP &;
  auto casps_extension_7bits(std::uint8_t value) noexcept -> CommonAtlasSequenceParameterSetRBSP &;
  auto casps_miv_extension() noexcept -> CaspsMivExtension &;
  auto caspsExtensionData(std::vector<bool> value) noexcept
      -> CommonAtlasSequenceParameterSetRBSP &;

  friend auto operator<<(std::ostream &stream, const CommonAtlasSequenceParameterSetRBSP &x)
      -> std::ostream &;

  auto operator==(const CommonAtlasSequenceParameterSetRBSP &other) const noexcept -> bool;
  auto operator!=(const CommonAtlasSequenceParameterSetRBSP &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> CommonAtlasSequenceParameterSetRBSP;

  void encodeTo(std::ostream &stream) const;

private:
  std::uint8_t m_casps_common_atlas_sequence_parameter_set_id{};
  std::uint8_t m_casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4{};
  bool m_casps_extension_present_flag{};
  std::optional<bool> m_casps_miv_extension_present_flag{};
  std::optional<std::uint8_t> m_casps_extension_7bits{};
  std::optional<CaspsMivExtension> m_casps_miv_extension{};
  std::optional<std::vector<bool>> m_caspsExtensionData{};
};

auto caspsById(const std::vector<CommonAtlasSequenceParameterSetRBSP> &caspsV, int id) noexcept
    -> const CommonAtlasSequenceParameterSetRBSP &;
} // namespace TMIV::MivBitstream

#include "CommonAtlasSequenceParameterSetRBSP.hpp"

#endif
