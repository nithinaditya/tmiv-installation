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

#include <TMIV/MivBitstream/NalUnit.h>

namespace TMIV::MivBitstream {
TEST_CASE("nal_unit_header", "[NAL unit]") {
  SECTION("Example 1") {
    const auto nal_unit_header = NalUnitHeader{NalUnitType::NAL_EOS, 0, 4};

    REQUIRE(toString(nal_unit_header) ==
            R"(nal_unit_type=NAL_EOS
nal_layer_id=0
nal_temporal_id_plus1=4
)");

    REQUIRE(byteCodingTest(nal_unit_header, 2));
  }

  SECTION("Example 2") {
    const auto nal_unit_header = NalUnitHeader{NalUnitType::NAL_BLA_W_LP, 2, 2};

    REQUIRE(byteCodingTest(nal_unit_header, 2));

    REQUIRE(toString(nal_unit_header) ==
            R"(nal_unit_type=NAL_BLA_W_LP
nal_layer_id=2
nal_temporal_id_plus1=2
)");
  }
}

TEST_CASE("nal_unit", "[NAL unit]") {
  SECTION("Example 1") {
    const auto nal_unit = NalUnit{NalUnitHeader{NalUnitType::NAL_EOS, 0, 1}, {}};

    REQUIRE(toString(nal_unit) ==
            R"(nal_unit_type=NAL_EOS
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=0
)");

    REQUIRE(unitCodingTest(nal_unit, 2));
  }

  SECTION("Example 2") {
    const auto nal_unit = NalUnit{NalUnitHeader{NalUnitType::NAL_RADL_R, 1, 3}, "payload"};

    REQUIRE(toString(nal_unit) ==
            R"(nal_unit_type=NAL_RADL_R
nal_layer_id=1
nal_temporal_id_plus1=3
NumBytesInRbsp=7
)");

    REQUIRE(unitCodingTest(nal_unit, 9));
  }
}
} // namespace TMIV::MivBitstream
