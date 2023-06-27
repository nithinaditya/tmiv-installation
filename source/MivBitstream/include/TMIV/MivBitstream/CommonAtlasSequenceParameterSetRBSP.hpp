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
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto CaspsMivExtension::casme_omaf_v1_compatible_flag() const noexcept {
  return m_casme_omaf_v1_compatible_flag;
}

constexpr auto CaspsMivExtension::casme_depth_low_quality_flag() const noexcept {
  return m_casme_depth_low_quality_flag;
}

constexpr auto CaspsMivExtension::casme_depth_quantization_params_present_flag() const noexcept {
  return m_casme_depth_quantization_params_present_flag;
}

constexpr auto CaspsMivExtension::casme_vui_params_present_flag() const noexcept {
  return m_casme_vui_params_present_flag;
}

constexpr auto CaspsMivExtension::casme_omaf_v1_compatible_flag(bool value) noexcept -> auto & {
  m_casme_omaf_v1_compatible_flag = value;
  return *this;
}

constexpr auto CaspsMivExtension::casme_depth_low_quality_flag(bool value) noexcept -> auto & {
  m_casme_depth_low_quality_flag = value;
  return *this;
}

constexpr auto CaspsMivExtension::casme_depth_quantization_params_present_flag(bool value) noexcept
    -> auto & {
  m_casme_depth_quantization_params_present_flag = value;
  return *this;
}

constexpr auto CaspsMivExtension::casme_vui_params_present_flag(bool value) noexcept -> auto & {
  m_casme_vui_params_present_flag = value;
  return *this;
}

constexpr auto
CommonAtlasSequenceParameterSetRBSP::casps_common_atlas_sequence_parameter_set_id() const noexcept {
  return m_casps_common_atlas_sequence_parameter_set_id;
}

constexpr auto
CommonAtlasSequenceParameterSetRBSP::casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4()
    const noexcept {
  return m_casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4;
}

constexpr auto CommonAtlasSequenceParameterSetRBSP::casps_extension_present_flag() const noexcept {
  return m_casps_extension_present_flag;
}

constexpr auto CommonAtlasSequenceParameterSetRBSP::casps_common_atlas_sequence_parameter_set_id(
    std::uint8_t value) noexcept -> auto & {
  m_casps_common_atlas_sequence_parameter_set_id = value;
  return *this;
}
constexpr auto
CommonAtlasSequenceParameterSetRBSP::casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4(
    std::uint8_t value) noexcept -> auto & {
  m_casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4 = value;
  return *this;
}
constexpr auto CommonAtlasSequenceParameterSetRBSP::casps_extension_present_flag(bool flag) noexcept
    -> auto & {
  m_casps_extension_present_flag = flag;
  return *this;
}
} // namespace TMIV::MivBitstream