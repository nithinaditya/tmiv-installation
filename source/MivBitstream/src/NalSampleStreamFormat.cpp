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

#include <TMIV/MivBitstream/NalSampleStreamFormat.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/verify.h>

namespace TMIV::MivBitstream {
SampleStreamNalHeader::SampleStreamNalHeader(int ssnh_unit_size_precision_bytes_minus1)
    : m_ssnh_unit_size_precision_bytes_minus1{
          static_cast<uint8_t>(ssnh_unit_size_precision_bytes_minus1)} {
  VERIFY_V3CBITSTREAM(ssnh_unit_size_precision_bytes_minus1 < 8);
}

auto operator<<(std::ostream &stream, const SampleStreamNalHeader &x) -> std::ostream & {
  return stream << "ssnh_unit_size_precision_bytes_minus1="
                << int{x.ssnh_unit_size_precision_bytes_minus1()} << '\n';
}

auto SampleStreamNalHeader::decodeFrom(std::istream &stream) -> SampleStreamNalHeader {
  Common::InputBitstream bitstream{stream};
  const auto ssnh_unit_size_precision_bytes_minus1 = bitstream.readBits<int>(3);
  return SampleStreamNalHeader{ssnh_unit_size_precision_bytes_minus1};
}

void SampleStreamNalHeader::encodeTo(std::ostream &stream) const {
  Common::OutputBitstream bitstream{stream};
  bitstream.writeBits(m_ssnh_unit_size_precision_bytes_minus1, 3);
}

SampleStreamNalUnit::SampleStreamNalUnit(std::string ssnu_nal_unit)
    : m_ssnu_nal_unit{std::move(ssnu_nal_unit)} {}

auto operator<<(std::ostream &stream, const SampleStreamNalUnit &x) -> std::ostream & {
  return stream << "nal_unit(" << (x.ssnu_nal_unit_size() - 2) << ")\n";
}

auto SampleStreamNalUnit::operator==(const SampleStreamNalUnit &other) const noexcept -> bool {
  return m_ssnu_nal_unit == other.m_ssnu_nal_unit;
}

auto SampleStreamNalUnit::operator!=(const SampleStreamNalUnit &other) const noexcept -> bool {
  return !operator==(other);
}

auto SampleStreamNalUnit::decodeFrom(std::istream &stream, const SampleStreamNalHeader &header)
    -> SampleStreamNalUnit {
  const auto ssnu_nal_unit_size =
      Common::readBytes(stream, header.ssnh_unit_size_precision_bytes_minus1() + size_t{1});
  return SampleStreamNalUnit{Common::readString(stream, static_cast<size_t>(ssnu_nal_unit_size))};
}

void SampleStreamNalUnit::encodeTo(std::ostream &stream,
                                   const SampleStreamNalHeader &header) const {
  Common::writeBytes(stream, m_ssnu_nal_unit.size(),
                     header.ssnh_unit_size_precision_bytes_minus1() + size_t{1});
  stream.write(m_ssnu_nal_unit.data(), m_ssnu_nal_unit.size());
}
} // namespace TMIV::MivBitstream
