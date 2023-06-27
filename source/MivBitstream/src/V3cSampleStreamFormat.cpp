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

#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/verify.h>

namespace TMIV::MivBitstream {
SampleStreamV3cHeader::SampleStreamV3cHeader(uint8_t ssvh_unit_size_precision_bytes_minus1)
    : m_ssvh_unit_size_precision_bytes_minus1{ssvh_unit_size_precision_bytes_minus1} {
  VERIFY_V3CBITSTREAM(ssvh_unit_size_precision_bytes_minus1 < 8);
}

auto operator<<(std::ostream &stream, const SampleStreamV3cHeader &x) -> std::ostream & {
  return stream << "ssvh_unit_size_precision_bytes_minus1="
                << int{x.ssvh_unit_size_precision_bytes_minus1()} << '\n';
}

auto SampleStreamV3cHeader::decodeFrom(std::istream &stream) -> SampleStreamV3cHeader {
  Common::InputBitstream bitstream{stream};
  const auto ssvh_unit_size_precision_bytes_minus1 = bitstream.readBits<uint8_t>(3);
  return SampleStreamV3cHeader{ssvh_unit_size_precision_bytes_minus1};
}

void SampleStreamV3cHeader::encodeTo(std::ostream &stream) const {
  Common::OutputBitstream bitstream{stream};
  bitstream.writeBits(m_ssvh_unit_size_precision_bytes_minus1, 3);
}

SampleStreamV3cUnit::SampleStreamV3cUnit(std::string ssvu_v3c_unit)
    : m_ssvu_v3c_unit{std::move(ssvu_v3c_unit)} {}

auto operator<<(std::ostream &stream, const SampleStreamV3cUnit &x) -> std::ostream & {
  return stream << "v3c_unit(" << x.ssvu_v3c_unit_size() << ")\n";
}

auto SampleStreamV3cUnit::operator==(const SampleStreamV3cUnit &other) const noexcept -> bool {
  return m_ssvu_v3c_unit == other.m_ssvu_v3c_unit;
}

auto SampleStreamV3cUnit::operator!=(const SampleStreamV3cUnit &other) const noexcept -> bool {
  return !operator==(other);
}

auto SampleStreamV3cUnit::decodeFrom(std::istream &stream, const SampleStreamV3cHeader &header)
    -> SampleStreamV3cUnit {
  const auto ssvu_v3c_unit_size =
      Common::readBytes(stream, header.ssvh_unit_size_precision_bytes_minus1() + size_t{1});
  return SampleStreamV3cUnit{Common::readString(stream, static_cast<size_t>(ssvu_v3c_unit_size))};
}

void SampleStreamV3cUnit::encodeTo(std::ostream &stream,
                                   const SampleStreamV3cHeader &header) const {
  Common::writeBytes(stream, m_ssvu_v3c_unit.size(),
                     header.ssvh_unit_size_precision_bytes_minus1() + size_t{1});
  stream.write(m_ssvu_v3c_unit.data(), m_ssvu_v3c_unit.size());
}
} // namespace TMIV::MivBitstream
