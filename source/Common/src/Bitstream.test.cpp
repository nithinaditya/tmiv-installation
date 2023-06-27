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

#include <catch2/catch.hpp>

#include <TMIV/Common/Bitstream.h>

#include <array>
#include <utility>

TEST_CASE("Bitstream primitives") {
  std::stringstream stream;
  TMIV::Common::OutputBitstream obitstream{stream};
  TMIV::Common::InputBitstream ibitstream{stream};

  SECTION("u(1)") {
    obitstream.putFlag(true);
    obitstream.zeroAlign();
    const auto actual = ibitstream.getFlag();
    REQUIRE(actual);
  }

  SECTION("bitstream") {
    const auto reference = std::array{false, true,  false, false, true, true, true,  false, true,
                                      true,  false, true,  false, true, true, false, true};
    for (const auto bit : reference) {
      obitstream.putFlag(bit);
    }
    obitstream.zeroAlign();
    REQUIRE(stream.tellp() == 3);
    for (const auto bit : reference) {
      const auto actual = ibitstream.getFlag();
      REQUIRE(actual == bit);
    }
    REQUIRE(stream.tellg() == 3);
  }

  SECTION("u(8)") {
    const auto reference = static_cast<uint8_t>(0x12);
    obitstream.putUint8(reference);
    obitstream.zeroAlign();
    const auto actual = ibitstream.getUint8();
    REQUIRE(actual == reference);
  }

  SECTION("u(16)") {
    const auto reference = static_cast<uint16_t>(0x1234);
    obitstream.putUint16(reference);
    obitstream.zeroAlign();
    const auto actual = ibitstream.getUint16();
    REQUIRE(actual == reference);
  }

  SECTION("u(32)") {
    const auto reference = static_cast<uint32_t>(0x12345678);
    obitstream.putUint32(reference);
    obitstream.zeroAlign();
    const auto actual = ibitstream.getUint32();
    REQUIRE(actual == reference);
  }

  SECTION("u(64)") {
    const auto reference = UINT64_MAX - 3;
    obitstream.putFlag(true);
    obitstream.putUint64(reference);
    obitstream.zeroAlign();
    ibitstream.getFlag();
    const auto actual = ibitstream.getUint64();
    REQUIRE(actual == reference);
  }

  SECTION("float32") {
    const auto reference = 1.F / 42.F;
    obitstream.putFloat32(reference);
    obitstream.zeroAlign();
    const auto actual = ibitstream.getFloat32();
    REQUIRE(actual == reference);
  }

  SECTION("u(v)") {
    const auto referenceSequence =
        std::array{std::tuple{123, 400}, std::tuple{4, 10}, std::tuple{400, 401}, std::tuple{0, 0},
                   std::tuple{0, 1}};
    for (auto [reference, range] : referenceSequence) {
      obitstream.putUVar(reference, range);
    }
    obitstream.zeroAlign();
    for (auto [reference, range] : referenceSequence) {
      auto actual = ibitstream.getUVar<int>(range);
      REQUIRE(actual == reference);
    }
  }

  SECTION("ue(v)") {
    const auto referenceSequence =
        std::array<std::uint64_t, 7>{123, 4, 400, 0, 1, 3, 0x123456789ABC};
    for (auto reference : referenceSequence) {
      obitstream.putUExpGolomb(reference);
    }
    obitstream.zeroAlign();
    for (auto reference : referenceSequence) {
      auto actual = ibitstream.getUExpGolomb<uint64_t>();
      REQUIRE(actual == reference);
    }
  }

  SECTION("se(v)") {
    const auto referenceSequence =
        std::array<std::int64_t, 7>{-123, 4, -400, 0, 1, -3, 0x123456789ABC};
    for (auto reference : referenceSequence) {
      obitstream.putSExpGolomb(reference);
    }
    obitstream.zeroAlign();
    for (auto reference : referenceSequence) {
      auto actual = ibitstream.getSExpGolomb<int64_t>();
      REQUIRE(actual == reference);
    }
  }

  SECTION("rbsp_trailing_bits") {
    for (int prefix = 0; prefix < 8; ++prefix) {
      for (int i = 0; i < prefix; ++i) {
        obitstream.putFlag(i % 2 == 0);
      }
      obitstream.rbspTrailingBits();

      for (int i = 0; i < prefix; ++i) {
        REQUIRE(ibitstream.getFlag() == (i % 2 == 0));
      }
      ibitstream.rbspTrailingBits();
    }
  }
}

TEST_CASE("ceilLog2") {
  using ValuePair = std::pair<std::uint64_t, int>;
  auto values = GENERATE(table<std::uint64_t, int>(
      {ValuePair{0, 0}, ValuePair{1, 0}, ValuePair{2, 1}, ValuePair{10, 4}, ValuePair{21, 5},
       ValuePair{64, 6}, ValuePair{100, 7}}));

  const auto input = std::get<0>(values);
  const auto expected_result = std::get<1>(values);

  REQUIRE(TMIV::Common::ceilLog2(input) == expected_result);
}
