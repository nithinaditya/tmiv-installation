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

#include <TMIV/MivBitstream/NalSampleStream.h>

#include <TMIV/MivBitstream/NalSampleStreamFormat.h>
#include <TMIV/MivBitstream/NalUnit.h>

#include <sstream>

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, const NalSampleStream &x) -> std::ostream & {
  return stream << x.sample_stream_nal_header();
}

auto NalSampleStream::operator==(const NalSampleStream &other) const noexcept -> bool {
  return sample_stream_nal_header() == other.sample_stream_nal_header() &&
         m_nal_units == other.m_nal_units;
}

auto NalSampleStream::operator!=(const NalSampleStream &other) const noexcept -> bool {
  return !operator==(other);
}

auto NalSampleStream::decodeFrom(std::istream &stream) -> NalSampleStream {
  const auto streamStart = stream.tellg();
  stream.seekg(0, std::ios::end);
  const auto streamEnd = stream.tellg();
  stream.seekg(streamStart);

  auto asb = NalSampleStream{SampleStreamNalHeader::decodeFrom(stream)};

  while (stream.tellg() != streamEnd) {
    const auto ssnu = SampleStreamNalUnit::decodeFrom(stream, asb.sample_stream_nal_header());

    std::istringstream substream{ssnu.ssnu_nal_unit()};
    asb.nal_units().push_back(NalUnit::decodeFrom(substream, ssnu.ssnu_nal_unit().size()));
  }

  return asb;
}

void NalSampleStream::encodeTo(std::ostream &stream) const {
  sample_stream_nal_header().encodeTo(stream);

  for (const auto &nal_unit : m_nal_units) {
    std::ostringstream substream;
    nal_unit.encodeTo(substream);

    const auto ssnu = SampleStreamNalUnit{substream.str()};
    ssnu.encodeTo(stream, sample_stream_nal_header());
  }
}
} // namespace TMIV::MivBitstream
