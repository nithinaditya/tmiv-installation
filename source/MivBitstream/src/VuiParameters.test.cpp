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

#include <TMIV/MivBitstream/VuiParameters.h>

#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>

namespace TMIV::MivBitstream {
namespace {
constexpr auto openGlCas() noexcept {
  auto x = CoordinateSystemParameters{};
  x.cas_forward_axis(2);           // -z points forward
  x.cas_delta_left_axis_minus1(0); // -x points left
  x.cas_forward_sign(false);       // z points back
  x.cas_left_sign(false);          // x points right
  x.cas_up_sign(false);            // y points down
  return x;
}
} // namespace

TEST_CASE("coordinate_axis_system_params", "[MIV VUI Params]") {
  SECTION("Default construction (OMAF CAS)") {
    constexpr auto x = CoordinateSystemParameters{};

    // Default construction corresponds to the OMAF coordinate axis system
    static_assert(x.isOmafCas());

    REQUIRE(toString(x) == R"(cas_forward_axis=0
cas_delta_left_axis_minus1=0
cas_forward_sign=true
cas_left_sign=true
cas_up_sign=true
)");

    REQUIRE(bitCodingTest(x, 6));
  }

  SECTION("Custom initialization") {
    constexpr auto x = openGlCas();

    // OpenGL uses a different convention than OMAF
    static_assert(!x.isOmafCas());

    REQUIRE(toString(x) == R"(cas_forward_axis=2
cas_delta_left_axis_minus1=0
cas_forward_sign=false
cas_left_sign=false
cas_up_sign=false
)");

    REQUIRE(bitCodingTest(x, 6));
  }
}

TEST_CASE("vui_parameters", "[VUI Parameters]") {
  SECTION("Default construction") {
    constexpr auto x = VuiParameters{};
    REQUIRE(toString(x) == R"(vui_timing_info_present_flag=false
vui_bitstream_restriction_present_flag=false
vui_coordinate_system_parameters_present_flag=false
vui_unit_in_metres_flag=false
vui_display_box_info_present_flag=false
vui_anchor_point_present_flag=false
)");
    REQUIRE(bitCodingTest(x, 6, nullptr));
  }

  SECTION("Enable flags") {
    auto x = VuiParameters{};

    x.vui_timing_info_present_flag(true)
        .vui_num_units_in_tick(1000)
        .vui_time_scale(32521)
        .vui_poc_proportional_to_timing_flag(true)
        .vui_num_ticks_poc_diff_one_minus1(143)
        .vui_hrd_parameters_present_flag(false);

    x.vui_bitstream_restriction_present_flag(true)
        .vui_tiles_fixed_structure_for_atlas_flag(true)
        .vui_tiles_fixed_structure_for_video_substreams_flag(true)
        .vui_constrained_tiles_across_v3c_components_idc(33)
        .vui_max_num_tiles_per_atlas_minus1(17);

    x.vui_coordinate_system_parameters_present_flag(true).coordinate_system_parameters() =
        openGlCas();

    x.vui_unit_in_metres_flag(true);

    x.vui_display_box_info_present_flag(true)
        .vui_display_box_origin(0, 10)
        .vui_display_box_origin(1, 20)
        .vui_display_box_origin(2, 30)
        .vui_display_box_size(0, 40)
        .vui_display_box_size(1, 50)
        .vui_display_box_size(2, 60);

    x.vui_anchor_point_present_flag(true)
        .vui_anchor_point(0, 70)
        .vui_anchor_point(1, 80)
        .vui_anchor_point(2, 90);

    REQUIRE(toString(x) == R"(vui_timing_info_present_flag=true
vui_num_units_in_tick=1000
vui_time_scale=32521
vui_poc_proportional_to_timing_flag=true
vui_num_ticks_poc_diff_one_minus1=143
vui_hrd_parameters_present_flag=false
vui_bitstream_restriction_present_flag=true
vui_tiles_fixed_structure_for_atlas_flag=true
vui_tiles_fixed_structure_for_video_substreams_flag=true
vui_constrained_tiles_across_v3c_components_idc=33
vui_max_num_tiles_per_atlas_minus1=17
vui_coordinate_system_parameters_present_flag=true
cas_forward_axis=2
cas_delta_left_axis_minus1=0
cas_forward_sign=false
cas_left_sign=false
cas_up_sign=false
vui_unit_in_metres_flag=true
vui_display_box_info_present_flag=true
vui_display_box_origin[ 0 ]=10
vui_display_box_size[ 0 ]=40
vui_display_box_origin[ 1 ]=20
vui_display_box_size[ 1 ]=50
vui_display_box_origin[ 2 ]=30
vui_display_box_size[ 2 ]=60
vui_anchor_point_present_flag=true
vui_anchor_point[ 0 ]=70
vui_anchor_point[ 1 ]=80
vui_anchor_point[ 2 ]=90
)");

    auto asps = AtlasSequenceParameterSetRBSP{};
    asps.asps_geometry_3d_bit_depth_minus1(15);

    REQUIRE(bitCodingTest(x, 259, &asps));
  }
}
} // namespace TMIV::MivBitstream
