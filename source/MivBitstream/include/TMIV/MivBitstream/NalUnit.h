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

#ifndef _TMIV_MIVBITSTREAM_NALUNIT_H_
#define _TMIV_MIVBITSTREAM_NALUNIT_H_

#include <cstdint>
#include <cstdlib>
#include <iosfwd>
#include <string>

namespace TMIV::MivBitstream {
enum class NalUnitType : std::uint8_t {
  NAL_TRAIL_N,
  NAL_TRAIL_R,
  NAL_TSA_N,
  NAL_TSA_R,
  NAL_STSA_N,
  NAL_STSA_R,
  NAL_RADL_N,
  NAL_RADL_R,
  NAL_RASL_N,
  NAL_RASL_R,
  NAL_SKIP_N,
  NAL_SKIP_R,
  NAL_BLA_W_LP = 16,
  NAL_BLA_W_RADL,
  NAL_BLA_N_LP,
  NAL_GBLA_W_LP,
  NAL_GBLA_W_RADL,
  NAL_GBLA_N_LP,
  NAL_IDR_W_RADL,
  NAL_IDR_N_LP,
  NAL_GIDR_W_RADL,
  NAL_GIDR_N_LP,
  NAL_CRA,
  NAL_GCRA,
  NAL_RSV_IRAP_ACL_29 = 29,
  NAL_RSV_ACL_35 = 35,
  NAL_ASPS = 36,
  NAL_AFPS,
  NAL_AUD,
  NAL_V3C_AUD,
  NAL_EOS,
  NAL_EOB,
  NAL_FD,
  NAL_PREFIX_NSEI,
  NAL_SUFFIX_NSEI,
  NAL_PREFIX_ESEI,
  NAL_SUFFIX_ESEI,
  NAL_AAPS,
  NAL_CASPS,
  NAL_IDR_CAF,
  NAL_CAF,
  NAL_RSV_NACL_51 = 51,
  NAL_RSV_NACL_52,
  NAL_RSV_NACL_53,
  NAL_RSV_NACL_54,
  NAL_RSV_NACL_55,
  NAL_UNSPEC_56 = 56,
  //  NAL_UNSPEC_57,
  //  NAL_UNSPEC_58,
  NAL_UNSPEC_59 = 59,
  NAL_UNSPEC_60,
  NAL_UNSPEC_61,
  NAL_UNSPEC_62,
  //  NAL_UNSPEC_63
};

auto operator<<(std::ostream &stream, NalUnitType x) -> std::ostream &;

// 23090-5: nal_unit_header()
class NalUnitHeader {
public:
  NalUnitHeader(NalUnitType nal_unit_type, int nal_layer_id, int nal_temporal_id_plus1);

  [[nodiscard]] constexpr auto nal_unit_type() const noexcept { return m_nal_unit_type; }
  [[nodiscard]] constexpr auto nal_layer_id() const noexcept { return m_nal_layer_id; }
  [[nodiscard]] constexpr auto nal_temporal_id_plus1() const noexcept {
    return m_nal_temporal_id_plus1;
  }

  friend auto operator<<(std::ostream &stream, const NalUnitHeader &x) -> std::ostream &;

  constexpr auto operator==(const NalUnitHeader &other) const noexcept -> bool {
    return m_nal_unit_type == other.m_nal_unit_type && m_nal_layer_id == other.m_nal_layer_id &&
           m_nal_temporal_id_plus1 == other.m_nal_temporal_id_plus1;
  }

  constexpr auto operator!=(const NalUnitHeader &other) const noexcept -> bool {
    return !operator==(other);
  }

  static auto decodeFrom(std::istream &stream) -> NalUnitHeader;

  void encodeTo(std::ostream &stream) const;

private:
  NalUnitType m_nal_unit_type;
  std::uint8_t m_nal_layer_id{};
  std::uint8_t m_nal_temporal_id_plus1{};
};

// 23090-5: nal_unit(NumBytesInNalUnit)
class NalUnit {
public:
  NalUnit(const NalUnitHeader &nal_unit_header, std::string rbsp);

  [[nodiscard]] constexpr auto nal_unit_header() const noexcept -> auto & {
    return m_nal_unit_header;
  }
  [[nodiscard]] constexpr auto rbsp() const noexcept -> auto & { return m_rbsp; }

  // The size of the NAL unit in bytes w/o zero byte padding
  [[nodiscard]] auto size() const noexcept { return 2 + m_rbsp.size(); }

  friend auto operator<<(std::ostream &stream, const NalUnit &x) -> std::ostream &;

  constexpr auto operator==(const NalUnit &other) const noexcept -> bool {
    return m_nal_unit_header == other.m_nal_unit_header && m_rbsp == other.m_rbsp;
  }

  constexpr auto operator!=(const NalUnit &other) const noexcept -> bool {
    return !operator==(other);
  }

  static auto decodeFrom(std::istream &stream, std::size_t numBytesInNalUnit) -> NalUnit;

  // Returns the size of the NAL unit in bytes w/o zero byte padding
  auto encodeTo(std::ostream &stream) const -> std::size_t;

private:
  NalUnitHeader m_nal_unit_header;
  std::string m_rbsp;
};
} // namespace TMIV::MivBitstream

#endif
