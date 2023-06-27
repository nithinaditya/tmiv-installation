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

#include <TMIV/MivBitstream/CommonAtlasFrameRBSP.h>

#include <TMIV/Common/verify.h>

#include <cmath>

namespace TMIV::MivBitstream {

auto CommonAtlasFrameRBSP::caf_miv_extension() const noexcept
    -> const CommonAtlasFrameMivExtension & {
  VERIFY_V3CBITSTREAM(caf_miv_extension_present_flag());
  VERIFY_V3CBITSTREAM(m_caf_miv_extension.has_value());
  return *m_caf_miv_extension;
}

auto CommonAtlasFrameRBSP::cafExtensionData() const noexcept -> const std::vector<bool> & {
  VERIFY_V3CBITSTREAM(caf_extension_7bits() != 0);
  VERIFY_V3CBITSTREAM(m_cafExtensionData.has_value());
  return *m_cafExtensionData;
}

auto CommonAtlasFrameRBSP::caf_miv_extension() noexcept -> CommonAtlasFrameMivExtension & {
  VERIFY_V3CBITSTREAM(caf_miv_extension_present_flag());
  if (!m_caf_miv_extension.has_value()) {
    m_caf_miv_extension = CommonAtlasFrameMivExtension{};
  }
  return *m_caf_miv_extension;
}

auto CommonAtlasFrameRBSP::cafExtensionData(std::vector<bool> value) noexcept
    -> CommonAtlasFrameRBSP & {
  VERIFY_V3CBITSTREAM(caf_extension_7bits() != 0);
  m_cafExtensionData = std::move(value);
  return *this;
}

auto operator<<(std::ostream &stream, const CommonAtlasFrameRBSP &x) -> std::ostream & {
  stream << "caf_common_atlas_sequence_parameter_set_id="
         << int{x.caf_common_atlas_sequence_parameter_set_id()} << '\n';
  stream << "caf_common_atlas_frm_order_cnt_lsb=" << x.caf_common_atlas_frm_order_cnt_lsb() << '\n';
  stream << "caf_extension_present_flag=" << std::boolalpha << x.caf_extension_present_flag()
         << '\n';
  if (x.caf_extension_present_flag()) {
    stream << "caf_miv_extension_present_flag=" << std::boolalpha
           << x.caf_miv_extension_present_flag() << '\n';
    stream << "caf_extension_7bits=" << int{x.caf_extension_7bits()} << '\n';
  }
  if (x.caf_miv_extension_present_flag()) {
    stream << x.caf_miv_extension();
  }
  if (x.caf_extension_7bits() != 0U) {
    for (const auto bit : x.cafExtensionData()) {
      stream << "caf_extension_data_flag=" << std::boolalpha << bit << '\n';
    }
  }
  return stream;
}

auto CommonAtlasFrameRBSP::operator==(const CommonAtlasFrameRBSP &other) const noexcept -> bool {
  if (caf_common_atlas_sequence_parameter_set_id() !=
          other.caf_common_atlas_sequence_parameter_set_id() ||
      caf_common_atlas_frm_order_cnt_lsb() != other.caf_common_atlas_frm_order_cnt_lsb() ||
      caf_extension_present_flag() != other.caf_extension_present_flag() ||
      caf_extension_7bits() != other.caf_extension_7bits()) {
    return false;
  }
  return caf_extension_7bits() == 0 || cafExtensionData() == other.cafExtensionData();
}

auto CommonAtlasFrameRBSP::operator!=(const CommonAtlasFrameRBSP &other) const noexcept -> bool {
  return !operator==(other);
}

auto CommonAtlasFrameRBSP::decodeFrom(
    std::istream &stream, const V3cParameterSet &vps, const NalUnitHeader &nuh,
    const std::vector<CommonAtlasSequenceParameterSetRBSP> &caspsV,
    unsigned maxCommonAtlasFrmOrderCntLsb) -> CommonAtlasFrameRBSP {
  Common::InputBitstream bitstream{stream};

  auto x = CommonAtlasFrameRBSP{};

  x.caf_common_atlas_sequence_parameter_set_id(bitstream.readBits<std::uint8_t>(4));
  const auto &casps = caspsById(caspsV, x.caf_common_atlas_sequence_parameter_set_id());

  x.caf_common_atlas_frm_order_cnt_lsb(
      bitstream.getUVar<std::uint16_t>(maxCommonAtlasFrmOrderCntLsb));
  x.caf_extension_present_flag(bitstream.getFlag());

  if (x.caf_extension_present_flag()) {
    x.caf_miv_extension_present_flag(bitstream.getFlag());
    x.caf_extension_7bits(bitstream.readBits<std::uint8_t>(7));
  }
  if (x.caf_miv_extension_present_flag()) {
    x.caf_miv_extension() = CommonAtlasFrameMivExtension::decodeFrom(bitstream, vps, nuh, casps);
  }
  if (x.caf_extension_7bits() != 0) {
    auto cafExtensionData = std::vector<bool>{};
    while (bitstream.moreRbspData()) {
      cafExtensionData.push_back(bitstream.getFlag());
    }
    x.cafExtensionData(std::move(cafExtensionData));
  }
  bitstream.rbspTrailingBits();

  return x;
}

void CommonAtlasFrameRBSP::encodeTo(std::ostream &stream, const V3cParameterSet &vps,
                                    const NalUnitHeader &nuh,
                                    const std::vector<CommonAtlasSequenceParameterSetRBSP> &caspsV,
                                    unsigned maxCommonAtlasFrmOrderCntLsb) const {
  VERIFY_V3CBITSTREAM(0 < maxCommonAtlasFrmOrderCntLsb);
  Common::OutputBitstream bitstream{stream};

  bitstream.writeBits(caf_common_atlas_sequence_parameter_set_id(), 4);
  const auto &casps = caspsById(caspsV, caf_common_atlas_sequence_parameter_set_id());

  bitstream.putUVar(caf_common_atlas_frm_order_cnt_lsb(), maxCommonAtlasFrmOrderCntLsb);
  bitstream.putFlag(caf_extension_present_flag());

  if (caf_extension_present_flag()) {
    bitstream.putFlag(caf_miv_extension_present_flag());
    bitstream.writeBits(caf_extension_7bits(), 7);
  }
  if (caf_miv_extension_present_flag()) {
    caf_miv_extension().encodeTo(bitstream, vps, nuh, casps);
  }
  if (caf_extension_7bits() != 0) {
    for (auto bit : cafExtensionData()) {
      bitstream.putFlag(bit);
    }
  }
  bitstream.rbspTrailingBits();
}
} // namespace TMIV::MivBitstream
