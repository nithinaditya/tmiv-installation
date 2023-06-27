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

#include <TMIV/Decoder/AtlasDecoder.h>

#include <TMIV/Common/verify.h>

#include "NalUnitSemantics.h"

#include <cassert>
#include <sstream>
#include <utility>

namespace TMIV::Decoder {
AtlasDecoder::AtlasDecoder(V3cUnitSource source, const MivBitstream::V3cUnitHeader &vuh,
                           MivBitstream::V3cParameterSet vps, int32_t foc)
    : m_source{std::move(source)}, m_vuh{vuh}, m_vps{std::move(vps)}, m_foc{foc} {}

auto AtlasDecoder::operator()() -> std::optional<AccessUnit> {
  if (!m_buffer.empty() || decodeAsb()) {
    return decodeAu();
  }
  return {};
}

auto AtlasDecoder::decodeAsb() -> bool {
  if (auto asb = m_source()) {
    assert(m_vuh == asb->v3c_unit_header());
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

auto AtlasDecoder::decodeAu() -> AccessUnit {
  auto au = AccessUnit{};

  const auto nut = [this]() { return m_buffer.front().nal_unit_header().nal_unit_type(); };

  if (!m_buffer.empty() && isAud(nut())) {
    m_buffer.pop_front();
  }

  while (!m_buffer.empty() && isPrefixNalUnit(nut())) {
    decodePrefixNalUnit(au, m_buffer.front());
    m_buffer.pop_front();
  }

  VERIFY_V3CBITSTREAM(!m_buffer.empty() && isAcl(nut()));
  decodeAclNalUnit(au, m_buffer.front());
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

  const auto focLsb = au.atl.atlas_tile_header().ath_atlas_frm_order_cnt_lsb();
  VERIFY_V3CBITSTREAM(focLsb < m_maxAtlasFrmOrderCntLsb);
  while (++m_foc % m_maxAtlasFrmOrderCntLsb != focLsb) {
    // deliberately empty
  }
  std::cout << "Atlas frame: foc=" << m_foc << ", vuh_atlas_id=" << m_vuh.vuh_atlas_id() << '\n';
  au.foc = m_foc;

  return au;
}

void AtlasDecoder::decodePrefixNalUnit(AccessUnit &au, const MivBitstream::NalUnit &nu) {
  std::istringstream stream{nu.rbsp()};

  switch (nu.nal_unit_header().nal_unit_type()) {
  case MivBitstream::NalUnitType::NAL_ASPS:
    return decodeAsps(stream);
  case MivBitstream::NalUnitType::NAL_AFPS:
    return decodeAfps(stream);
  case MivBitstream::NalUnitType::NAL_PREFIX_ESEI:
  case MivBitstream::NalUnitType::NAL_PREFIX_NSEI:
    return decodeSei(au, stream);
  default:
    std::cout << "WARNING: Ignoring NAL unit:\n" << nu;
  }
}

void AtlasDecoder::decodeAclNalUnit(AccessUnit &au, const MivBitstream::NalUnit &nu) {
  std::istringstream stream{nu.rbsp()};
  au.atl =
      MivBitstream::AtlasTileLayerRBSP::decodeFrom(stream, nu.nal_unit_header(), m_aspsV, m_afpsV);
  au.afps = afpsById(m_afpsV, au.atl.atlas_tile_header().ath_atlas_frame_parameter_set_id());
  au.asps = aspsById(m_aspsV, au.afps.afps_atlas_sequence_parameter_set_id());
}

void AtlasDecoder::decodeSuffixNalUnit(AccessUnit &au, const MivBitstream::NalUnit &nu) {
  std::istringstream stream{nu.rbsp()};

  switch (nu.nal_unit_header().nal_unit_type()) {
  case MivBitstream::NalUnitType::NAL_FD:
    return; // Ignore filler data
  case MivBitstream::NalUnitType::NAL_SUFFIX_ESEI:
  case MivBitstream::NalUnitType::NAL_SUFFIX_NSEI:
    return decodeSei(au, stream);
  default:
    std::cout << "WARNING: Ignoring NAL unit:\n" << nu;
  }
}

void AtlasDecoder::decodeAsps(std::istream &stream) {
  auto asps = MivBitstream::AtlasSequenceParameterSetRBSP::decodeFrom(stream, m_vuh, m_vps);

  m_maxAtlasFrmOrderCntLsb = 1U << (asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4U);

  for (auto &x : m_aspsV) {
    if (x.asps_atlas_sequence_parameter_set_id() == asps.asps_atlas_sequence_parameter_set_id()) {
      VERIFY_V3CBITSTREAM(x == asps);
      return;
    }
  }
  return m_aspsV.push_back(asps);
}

void AtlasDecoder::decodeAfps(std::istream &stream) {
  auto afps = MivBitstream::AtlasFrameParameterSetRBSP::decodeFrom(stream, m_aspsV);
  for (auto &x : m_afpsV) {
    if (x.afps_atlas_frame_parameter_set_id() == afps.afps_atlas_frame_parameter_set_id()) {
      x = std::move(afps);
      return;
    }
  }
  return m_afpsV.push_back(afps);
}

void AtlasDecoder::decodeSei(AccessUnit &au, std::istream &stream) {
  const auto sei = MivBitstream::SeiRBSP::decodeFrom(stream);
  for (const auto &message : sei.messages()) {
    decodeSeiMessage(au, message);
  }
}

void AtlasDecoder::decodeSeiMessage(AccessUnit & /* au */,
                                    const MivBitstream::SeiMessage & /* message */) {
  // Currently all SEI messages are ignored
}
} // namespace TMIV::Decoder
