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

#ifndef _TMIV_MIVBITSTREAM_COMMONATLASFRAMERBSP_H_
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/verify.h>

namespace TMIV::MivBitstream {
constexpr auto CommonAtlasFrameRBSP::caf_common_atlas_sequence_parameter_set_id() const noexcept {
  return m_caf_common_atlas_sequence_parameter_set_id;
}

constexpr auto CommonAtlasFrameRBSP::caf_common_atlas_frm_order_cnt_lsb() const noexcept {
  return m_caf_common_atlas_frm_order_cnt_lsb;
}

constexpr auto CommonAtlasFrameRBSP::caf_extension_present_flag() const noexcept {
  return m_caf_extension_present_flag;
}

constexpr auto CommonAtlasFrameRBSP::caf_miv_extension_present_flag() const noexcept {
  return m_caf_miv_extension_present_flag.value_or(false);
}

constexpr auto CommonAtlasFrameRBSP::caf_extension_7bits() const noexcept {
  return m_caf_extension_7bits.value_or(0);
}

constexpr auto
CommonAtlasFrameRBSP::caf_common_atlas_sequence_parameter_set_id(uint8_t value) noexcept -> auto & {
  m_caf_common_atlas_sequence_parameter_set_id = value;
  return *this;
}

constexpr auto
CommonAtlasFrameRBSP::caf_common_atlas_frm_order_cnt_lsb(std::uint16_t value) noexcept -> auto & {
  m_caf_common_atlas_frm_order_cnt_lsb = value;
  return *this;
}

constexpr auto CommonAtlasFrameRBSP::caf_extension_present_flag(bool value) noexcept -> auto & {
  m_caf_extension_present_flag = value;
  return *this;
}

constexpr auto CommonAtlasFrameRBSP::caf_miv_extension_present_flag(bool value) noexcept -> auto & {
  m_caf_miv_extension_present_flag = value;
  return *this;
}

constexpr auto CommonAtlasFrameRBSP::caf_extension_7bits(std::uint8_t value) noexcept
    -> CommonAtlasFrameRBSP & {
  VERIFY_V3CBITSTREAM(caf_extension_present_flag());
  m_caf_extension_7bits = value;
  return *this;
}

} // namespace TMIV::MivBitstream
