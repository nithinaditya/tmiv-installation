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

#include <TMIV/MivBitstream/AtlasAdaptationParameterSetRBSP.h>

namespace TMIV::MivBitstream {
TEST_CASE("atlas_adaptation_parameter_set_rbsp", "[Atlas Adaptation Parameter Set RBSP]") {
  auto x = AtlasAdaptationParameterSetRBSP{};

  REQUIRE(toString(x) == R"(aaps_atlas_adaptation_parameter_set_id=0
aaps_log2_max_afoc_present_flag=false
aaps_extension_present_flag=false
)");

  REQUIRE(byteCodingTest(x, 1));

  SECTION("Example 1") {
    x.aaps_atlas_adaptation_parameter_set_id(63)
        .aaps_log2_max_afoc_present_flag(true)
        .aaps_log2_max_atlas_frame_order_cnt_lsb_minus4(12)
        .aaps_extension_present_flag(true)
        .aaps_vpcc_extension_present_flag(true)
        .aaps_vpcc_extension({})
        .aaps_extension_7bits(127)
        .aapsExtensionData({true});

    REQUIRE(toString(x) == R"(aaps_atlas_adaptation_parameter_set_id=63
aaps_log2_max_afoc_present_flag=true
aaps_log2_max_atlas_frame_order_cnt_lsb_minus4=12
aaps_extension_present_flag=true
aaps_vpcc_extension_present_flag=true
aaps_extension_7bits=127
aaps_vpcc_camera_parameters_present_flag=false
aaps_extension_data_flag=true
)");

    REQUIRE(byteCodingTest(x, 5));
  }

  SECTION("Example 2") {
    x.aaps_atlas_adaptation_parameter_set_id(62)
        .aaps_log2_max_afoc_present_flag(true)
        .aaps_log2_max_atlas_frame_order_cnt_lsb_minus4(10)
        .aaps_extension_present_flag(true)
        .aaps_vpcc_extension_present_flag(true)
        .aaps_vpcc_extension({})
        .aaps_extension_7bits(60)
        .aapsExtensionData({true});

    REQUIRE(toString(x) == R"(aaps_atlas_adaptation_parameter_set_id=62
aaps_log2_max_afoc_present_flag=true
aaps_log2_max_atlas_frame_order_cnt_lsb_minus4=10
aaps_extension_present_flag=true
aaps_vpcc_extension_present_flag=true
aaps_extension_7bits=60
aaps_vpcc_camera_parameters_present_flag=false
aaps_extension_data_flag=true
)");

    REQUIRE(byteCodingTest(x, 4));
  }
}
} // namespace TMIV::MivBitstream
