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

#include <TMIV/MivBitstream/CommonAtlasFrameRBSP.h>

namespace TMIV::MivBitstream {
TEST_CASE("common_atlas_frame_rbsp", "[Common Atlas Frame RBSP]") {
  auto x = CommonAtlasFrameRBSP{};
  const auto nuhIdrCaf = NalUnitHeader{NalUnitType::NAL_IDR_CAF, 0, 1};
  const auto nuhCaf = NalUnitHeader{NalUnitType::NAL_CAF, 0, 1};
  const auto maxCommonAtlasFrmOrderCntLsb = 32;

  auto vps = V3cParameterSet{};
  vps.vps_extension_present_flag(true);
  vps.vps_miv_extension_present_flag(true);
  vps.vps_miv_extension() = {};

  const auto caspsV = [] {
    auto v = std::vector<CommonAtlasSequenceParameterSetRBSP>{};
    for (const std::uint8_t id : {0, 14, 15}) {
      v.emplace_back()
          .casps_common_atlas_sequence_parameter_set_id(id)
          .casps_extension_present_flag(true)
          .casps_miv_extension_present_flag(true)
          .casps_miv_extension()
          .casme_depth_quantization_params_present_flag(id % 2 == 0);
    }
    return v;
  }();

  SECTION("Default constructor") {
    REQUIRE(toString(x) == R"(caf_common_atlas_sequence_parameter_set_id=0
caf_common_atlas_frm_order_cnt_lsb=0
caf_extension_present_flag=false
)");

    REQUIRE(byteCodingTest(x, 2, vps, nuhIdrCaf, caspsV, maxCommonAtlasFrmOrderCntLsb));
  }

  SECTION("Extension present, but no MIV extension") {
    x.caf_common_atlas_sequence_parameter_set_id(15)
        .caf_common_atlas_frm_order_cnt_lsb(31)
        .caf_extension_present_flag(true)
        .caf_miv_extension_present_flag(false)
        .caf_extension_7bits(127)
        .cafExtensionData({true, false});

    REQUIRE(toString(x) == R"(caf_common_atlas_sequence_parameter_set_id=15
caf_common_atlas_frm_order_cnt_lsb=31
caf_extension_present_flag=true
caf_miv_extension_present_flag=false
caf_extension_7bits=127
caf_extension_data_flag=true
caf_extension_data_flag=false
)");

    REQUIRE(byteCodingTest(x, 3, vps, nuhIdrCaf, caspsV, maxCommonAtlasFrmOrderCntLsb));
  }

  SECTION("MIV extension present") {
    x.caf_common_atlas_sequence_parameter_set_id(14)
        .caf_common_atlas_frm_order_cnt_lsb(30)
        .caf_extension_present_flag(true)
        .caf_miv_extension_present_flag(true)
        .caf_miv_extension()
        .came_update_extrinsics_flag(false)
        .came_update_intrinsics_flag(false)
        .came_update_depth_quantization_flag(false);

    REQUIRE(toString(x) == R"(caf_common_atlas_sequence_parameter_set_id=14
caf_common_atlas_frm_order_cnt_lsb=30
caf_extension_present_flag=true
caf_miv_extension_present_flag=true
caf_extension_7bits=0
came_update_extrinsics_flag=false
came_update_intrinsics_flag=false
came_update_depth_quantization_flag=false
)");

    REQUIRE(byteCodingTest(x, 3, vps, nuhCaf, caspsV, maxCommonAtlasFrmOrderCntLsb));
  }
}
} // namespace TMIV::MivBitstream
