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

#include <TMIV/MivBitstream/GeometryUpscalingParameters.h>

namespace TMIV::MivBitstream {
TEST_CASE("geometry_upscaling_parameters", "[Geometry upscaling parameters SEI payload syntax]") {
  SECTION("Null example") {
    const auto x = GeometryUpscalingParameters{};
    REQUIRE(toString(x) == R"(gup_type=HVR
gup_erode_threshold=0
gup_delta_threshold=0
gup_max_curvature=0
)");
    REQUIRE(bitCodingTest(x, 21));
  }

  SECTION("GupType HVR") {
    auto x = GeometryUpscalingParameters{};
    x.gup_type(GupType::HVR)
        .gup_max_curvature(7)
        .gup_erode_threshold(TMIV::Common::Half(3.5F))
        .gup_delta_threshold(3);
    REQUIRE(toString(x) == R"(gup_type=HVR
gup_erode_threshold=3.5
gup_delta_threshold=3
gup_max_curvature=7
)");
    REQUIRE(bitCodingTest(x, 25));
  }

  SECTION("GupType unknown") {
    auto x = GeometryUpscalingParameters{};
    x.gup_type(GupType(100));
    REQUIRE(toString(x) == R"(gup_type=[unknown:100]
)");
    REQUIRE(bitCodingTest(x, 13));
    REQUIRE(x.gup_erode_threshold() == 1.F);
    REQUIRE(x.gup_delta_threshold() == 10);
    REQUIRE(x.gup_max_curvature() == 5);
  }
}
} // namespace TMIV::MivBitstream
