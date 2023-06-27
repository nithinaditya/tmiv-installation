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

#include <TMIV/Decoder/CommonAtlasDecoder.h>

#include <TMIV/Common/verify.h>

#include "NalUnitSemantics.h"

#include <cassert>
#include <sstream>
#include <utility>

namespace TMIV::Decoder {
CommonAtlasDecoder::CommonAtlasDecoder(V3cUnitSource source, MivBitstream::V3cParameterSet vps,
                                       int32_t foc)
    : m_source{std::move(source)}, m_vps{std::move(vps)}, m_foc{foc} {}

auto CommonAtlasDecoder::operator()() -> std::optional<AccessUnit> {
  if (!m_buffer.empty() || decodeAsb()) {
    return decodeAu();
  }
  return {};
}

auto CommonAtlasDecoder::decodeAsb() -> bool {
  if (auto asb = m_source()) {
    for (const auto &nu : asb->v3c_unit_payload().atlas_sub_bitstream().nal_units()) {
      if (nu.nal_unit_header().nal_layer_id() == 0) {
        m_buffer.push_back(nu);
      } else {
        std::cout << "WARNING: Ignoring NAL unit:\n" << nu;
      }
    }
    VERIFY_V3CBITSTREAM(!m_buffer.empty());
    return true;
  }
  return false;
}

namespace {
constexpr auto isIrap(MivBitstream::NalUnitType nut) noexcept -> bool {
  return nut == MivBitstream::NalUnitType::NAL_IDR_CAF;
}
} // namespace

auto CommonAtlasDecoder::decodeAu() -> AccessUnit {
  auto au = AccessUnit{};

  const auto nut = [this]() { return m_buffer.front().nal_unit_header().nal_unit_type(); };

  if (!m_buffer.empty() && isAud(nut())) {
    m_buffer.pop_front();
  }

  while (!m_buffer.empty() && isPrefixNalUnit(nut())) {
    decodePrefixNalUnit(au, m_buffer.front());
    m_buffer.pop_front();
  }

  VERIFY_V3CBITSTREAM(!m_buffer.empty() && isCaf(nut()));
  au.irap = isIrap(nut());
  decodeCafNalUnit(au, m_buffer.front());
  m_buffer.pop_front();

  while (!m_buffer.empty() && isSuffixNalUnit(nut())) {
    decodeSuffixNalUnit(au, m_buffer.front());
    m_buffer.pop_front();
  }

  if (!m_buffer.empty() && isEos(nut())) {
    m_buffer.pop_front();
  }

  if (!m_buffer.empty() && isEob(nut())) {
    m_buffer.pop_front();
  }

  const auto focLsb = au.caf.caf_common_atlas_frm_order_cnt_lsb();
  VERIFY_V3CBITSTREAM(focLsb < m_maxCommonAtlasFrmOrderCntLsb);
  while (++m_foc % m_maxCommonAtlasFrmOrderCntLsb != focLsb) {
    // deliberately empty
  }
  std::cout << "Common atlas frame: foc=" << m_foc << '\n';
  au.foc = m_foc;

  return au;
}

void CommonAtlasDecoder::decodePrefixNalUnit(AccessUnit &au, const MivBitstream::NalUnit &nu) {
  std::istringstream stream{nu.rbsp()};

  switch (nu.nal_unit_header().nal_unit_type()) {
  case MivBitstream::NalUnitType::NAL_CASPS:
    return decodeCasps(stream);
  case MivBitstream::NalUnitType::NAL_PREFIX_ESEI:
  case MivBitstream::NalUnitType::NAL_PREFIX_NSEI:
    return decodeSei(au, stream);
  default:
    std::cout << "WARNING: Ignoring NAL unit:\n" << nu;
  }
}

void CommonAtlasDecoder::decodeCafNalUnit(AccessUnit &au, const MivBitstream::NalUnit &nu) {
  std::istringstream stream{nu.rbsp()};

  VERIFY_MIVBITSTREAM(0 < m_maxCommonAtlasFrmOrderCntLsb);
  au.caf = MivBitstream::CommonAtlasFrameRBSP::decodeFrom(stream, m_vps, nu.nal_unit_header(),
                                                          m_caspsV, m_maxCommonAtlasFrmOrderCntLsb);
  au.casps = caspsById(m_caspsV, au.caf.caf_common_atlas_sequence_parameter_set_id());
}

void CommonAtlasDecoder::decodeSuffixNalUnit(AccessUnit &au, const MivBitstream::NalUnit &nu) {
  std::istringstream stream{nu.rbsp()};

  switch (nu.nal_unit_header().nal_unit_type()) {
  case MivBitstream::NalUnitType::NAL_FD:
    return;
  case MivBitstream::NalUnitType::NAL_SUFFIX_ESEI:
  case MivBitstream::NalUnitType::NAL_SUFFIX_NSEI:
    return decodeSei(au, stream);
  default:
    std::cout << "WARNING: Ignoring NAL unit:\n" << nu;
  }
}

void CommonAtlasDecoder::decodeCasps(std::istream &stream) {
  auto casps = MivBitstream::CommonAtlasSequenceParameterSetRBSP::decodeFrom(stream);

  const auto maxCommonAtlasFrmOrderCntLsb = static_cast<std::int32_t>(
      1U << (casps.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4() + 4U));
  VERIFY_MIVBITSTREAM(m_maxCommonAtlasFrmOrderCntLsb == 0 ||
                      m_maxCommonAtlasFrmOrderCntLsb == maxCommonAtlasFrmOrderCntLsb);
  m_maxCommonAtlasFrmOrderCntLsb = maxCommonAtlasFrmOrderCntLsb;

  for (auto &storedCasps : m_caspsV) {
    if (storedCasps.casps_common_atlas_sequence_parameter_set_id() ==
        casps.casps_common_atlas_sequence_parameter_set_id()) {
      storedCasps = std::move(casps);
      return;
    }
  }
  return m_caspsV.push_back(casps);
}

void CommonAtlasDecoder::decodeSei(AccessUnit &au, std::istream &stream) {
  auto sei = MivBitstream::SeiRBSP::decodeFrom(stream);
  for (auto &message : sei.messages()) {
    decodeSeiMessage(au, message);
  }
}

void CommonAtlasDecoder::decodeSeiMessage(AccessUnit &au, const MivBitstream::SeiMessage &message) {
  std::istringstream messageStream{message.payload()};
  Common::InputBitstream bitstream{messageStream};

  switch (message.payloadType()) {
  case MivBitstream::PayloadType::geometry_upscaling_parameters:
    au.gup = MivBitstream::GeometryUpscalingParameters::decodeFrom(bitstream);
    return;
  case MivBitstream::PayloadType::viewing_space:
    au.vs = MivBitstream::ViewingSpace::decodeFrom(bitstream);
    return;
  default:
    return;
    // NOTE(BK): Ignore SEI messages that are not handled by TMIV. (You can still print them out
    // with the Parser executable.)
  }
}
} // namespace TMIV::Decoder
