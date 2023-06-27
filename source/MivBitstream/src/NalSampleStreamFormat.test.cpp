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

#include "test.h"

#include <TMIV/MivBitstream/NalSampleStreamFormat.h>

namespace TMIV::MivBitstream {
TEST_CASE("sample_stream_nal_header", "[NAL sample stream format]") {
  const auto sample_stream_nal_header =
      std::array{SampleStreamNalHeader{0}, SampleStreamNalHeader{7}};

  SECTION("encodeTo/decodeFrom") {
    REQUIRE(byteCodingTest(sample_stream_nal_header[0], 1));
    REQUIRE(byteCodingTest(sample_stream_nal_header[1], 1));
  }
  SECTION("operator <<") {
    REQUIRE(toString(sample_stream_nal_header[0]) == "ssnh_unit_size_precision_bytes_minus1=0\n");
    REQUIRE(toString(sample_stream_nal_header[1]) == "ssnh_unit_size_precision_bytes_minus1=7\n");
  }
}

TEST_CASE("sample_stream_nal_unit", "[NAL sample stream format]") {
  const auto sample_stream_nal_header =
      std::array{SampleStreamNalHeader{0}, SampleStreamNalHeader{7}};
  const auto sample_stream_nal_unit =
      std::array{SampleStreamNalUnit{"Hello, World!"}, SampleStreamNalUnit{std::string(3, '\0')}};

  SECTION("encodeTo/decodeFrom") {
    REQUIRE(byteCodingTest(sample_stream_nal_unit[0], 14, sample_stream_nal_header[0]));
    REQUIRE(byteCodingTest(sample_stream_nal_unit[1], 11, sample_stream_nal_header[1]));
  }
  SECTION("operator <<") {
    REQUIRE(toString(sample_stream_nal_unit[0]) == "nal_unit(11)\n");
    REQUIRE(toString(sample_stream_nal_unit[1]) == "nal_unit(1)\n");
  }
  SECTION("ssnu_nal_unit_size()") {
    REQUIRE(sample_stream_nal_unit[0].ssnu_nal_unit_size() == 13);
    REQUIRE(sample_stream_nal_unit[1].ssnu_nal_unit_size() == 3);
  }
  SECTION("ssnu_nal_unit()") {
    REQUIRE(sample_stream_nal_unit[0].ssnu_nal_unit() == "Hello, World!");
    REQUIRE(sample_stream_nal_unit[1].ssnu_nal_unit() == std::string(3, '\0'));
  }
}

TEST_CASE("NAL sample stream format", "[NAL sample stream format]") {
  const auto header = SampleStreamNalHeader{0};
  const auto units = std::array{SampleStreamNalUnit{"Test"}, SampleStreamNalUnit{"having"},
                                SampleStreamNalUnit{"multiple units"}};
  const auto bytes_ = std::array<char, 29>{"\x0\x4Test\x6having\xEmultiple units"};
  const auto bytes = std::string(std::cbegin(bytes_), std::prev(std::cend(bytes_)));

  SECTION("encodeTo") {
    std::ostringstream stream;
    header.encodeTo(stream);
    for (const auto &unit : units) {
      unit.encodeTo(stream, header);
    }
    REQUIRE(stream.str() == bytes);
  }

  SECTION("decodeFrom") {
    std::istringstream stream(bytes);
    const auto actualHeader = SampleStreamNalHeader::decodeFrom(stream);
    REQUIRE(actualHeader == header);
    for (const auto &unit : units) {
      const auto actualUnit = SampleStreamNalUnit::decodeFrom(stream, actualHeader);
      REQUIRE(actualUnit == unit);
    }
    REQUIRE(stream.tellg() == bytes.size());
  }
}
} // namespace TMIV::MivBitstream
