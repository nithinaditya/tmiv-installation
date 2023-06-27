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
#define _TMIV_MIVBITSTREAM_COMMONATLASFRAMERBSP_H_

#include <TMIV/MivBitstream/CommonAtlasFrameMivExtension.h>
#include <TMIV/MivBitstream/V3cParameterSet.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Matrix.h>
#include <TMIV/Common/Quaternion.h>
#include <TMIV/Common/Vector.h>

#include <iosfwd>
#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
// 23090-12: common_atlas_frame_rbsp( )
class CommonAtlasFrameRBSP {
public:
  [[nodiscard]] constexpr auto caf_common_atlas_sequence_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto caf_common_atlas_frm_order_cnt_lsb() const noexcept;
  [[nodiscard]] constexpr auto caf_extension_present_flag() const noexcept;
  [[nodiscard]] constexpr auto caf_miv_extension_present_flag() const noexcept;
  [[nodiscard]] auto caf_miv_extension() const noexcept -> const CommonAtlasFrameMivExtension &;
  [[nodiscard]] constexpr auto caf_extension_7bits() const noexcept;
  [[nodiscard]] auto cafExtensionData() const noexcept -> const std::vector<bool> &;

  constexpr auto caf_common_atlas_sequence_parameter_set_id(std::uint8_t value) noexcept -> auto &;
  constexpr auto caf_common_atlas_frm_order_cnt_lsb(std::uint16_t value) noexcept -> auto &;
  constexpr auto caf_extension_present_flag(bool value) noexcept -> auto &;
  constexpr auto caf_miv_extension_present_flag(bool value) noexcept -> auto &;
  auto caf_miv_extension() noexcept -> CommonAtlasFrameMivExtension &;
  constexpr auto caf_extension_7bits(std::uint8_t value) noexcept -> CommonAtlasFrameRBSP &;
  auto cafExtensionData(std::vector<bool> value) noexcept -> CommonAtlasFrameRBSP &;

  friend auto operator<<(std::ostream &stream, const CommonAtlasFrameRBSP &x) -> std::ostream &;

  auto operator==(const CommonAtlasFrameRBSP &) const noexcept -> bool;
  auto operator!=(const CommonAtlasFrameRBSP &) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream, const V3cParameterSet &vps, const NalUnitHeader &nuh,
                         const std::vector<CommonAtlasSequenceParameterSetRBSP> &caspsV,
                         unsigned maxCommonAtlasFrmOrderCntLsb) -> CommonAtlasFrameRBSP;

  void encodeTo(std::ostream &stream, const V3cParameterSet &vps, const NalUnitHeader &nuh,
                const std::vector<CommonAtlasSequenceParameterSetRBSP> &caspsV,
                unsigned maxCommonAtlasFrmOrderCntLsb) const;

private:
  std::uint8_t m_caf_common_atlas_sequence_parameter_set_id{};
  std::uint16_t m_caf_common_atlas_frm_order_cnt_lsb{};
  bool m_caf_extension_present_flag{};
  std::optional<bool> m_caf_miv_extension_present_flag{};
  std::optional<CommonAtlasFrameMivExtension> m_caf_miv_extension{};
  std::optional<std::uint8_t> m_caf_extension_7bits{};
  std::optional<std::vector<bool>> m_cafExtensionData{};
};
} // namespace TMIV::MivBitstream

#include "CommonAtlasFrameRBSP.hpp"

#endif
