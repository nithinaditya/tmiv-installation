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

#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>

namespace TMIV::MivBitstream {
TEST_CASE("ref_list_struct", "[Atlas Sequence Parameter Set RBSP]") {
  SECTION("Empty list") {
    auto asps = AtlasSequenceParameterSetRBSP{};
    asps.asps_long_term_ref_atlas_frames_flag(false);

    const auto x = RefListStruct{};
    REQUIRE(x.num_ref_entries() == 0);
    REQUIRE(toString(x, 7) == R"(num_ref_entries( 7 )=0
)");
    REQUIRE(bitCodingTest(x, 1, asps));
  }

  SECTION("Some values") {
    auto asps = AtlasSequenceParameterSetRBSP{};
    asps.asps_long_term_ref_atlas_frames_flag(false);

    const auto x = RefListStruct{{-INT16_MAX, -4, -1, 0, 1, 13, INT16_MAX}};
    REQUIRE(x.num_ref_entries() == 7);
    REQUIRE(toString(x, 3) == R"(num_ref_entries( 3 )=7
DeltaAfocSt( 3, 0 )=-32767
DeltaAfocSt( 3, 1 )=-4
DeltaAfocSt( 3, 2 )=-1
DeltaAfocSt( 3, 3 )=0
DeltaAfocSt( 3, 4 )=1
DeltaAfocSt( 3, 5 )=13
DeltaAfocSt( 3, 6 )=32767
)");
    REQUIRE(bitCodingTest(x, 94, asps));
  }
}

TEST_CASE("asps_vpcc_extension", "[Atlas Sequence Parameter Set RBSP]") {
  auto x = AspsVpccExtension{};

  REQUIRE(toString(x) == R"(asps_vpcc_remove_duplicate_point_enabled_flag=false
)");

  SECTION("Example 1") {
    const auto asps = AtlasSequenceParameterSetRBSP{};
    x.asps_vpcc_remove_duplicate_point_enabled_flag(true);

    REQUIRE(toString(x) == R"(asps_vpcc_remove_duplicate_point_enabled_flag=true
)");

    REQUIRE(bitCodingTest(x, 1, asps));
  }
}

TEST_CASE("atlas_sequence_parameter_set_miv_extension", "[Atlas Sequence Parameter Set RBSP]") {
  SECTION("Default constructor") {
    const auto unit = AspsMivExtension{};
    REQUIRE(toString(unit) == R"(asme_ancillary_atlas_flag=false
asme_embedded_occupancy_enabled_flag=false
asme_geometry_scale_enabled_flag=false
asme_occupancy_scale_enabled_flag=false
asme_patch_constant_depth_flag=false
asme_patch_attribute_offset_enabled_flag=false
asme_max_entity_id=0
asme_inpaint_enabled_flag=false
)");
    REQUIRE(bitCodingTest(unit, 8));
  }

  SECTION("Embedded Occupancy enabled") {
    auto unit = AspsMivExtension{};
    unit.asme_embedded_occupancy_enabled_flag(true)
        .asme_depth_occ_threshold_flag(true)
        .asme_geometry_scale_enabled_flag(true)
        .asme_geometry_scale_factor_x_minus1(1)
        .asme_geometry_scale_factor_y_minus1(2)
        .asme_patch_attribute_offset_enabled_flag(true)
        .asme_patch_attribute_offset_bit_depth_minus1(3)
        .asme_max_entity_id(15)
        .asme_inpaint_enabled_flag(true);

    REQUIRE(toString(unit) == R"(asme_ancillary_atlas_flag=false
asme_embedded_occupancy_enabled_flag=true
asme_depth_occ_map_threshold_flag=true
asme_geometry_scale_enabled_flag=true
asme_geometry_scale_factor_x_minus1=1
asme_geometry_scale_factor_y_minus1=2
asme_patch_constant_depth_flag=false
asme_patch_attribute_offset_enabled_flag=true
asme_patch_attribute_offset_bit_depth_minus1=3
asme_max_entity_id=15
asme_inpaint_enabled_flag=true
)");
    REQUIRE(bitCodingTest(unit, 27));
  }

  SECTION("Embedded occupancy disabled, occupancy scale enabled") {
    auto unit = AspsMivExtension{};
    unit.asme_embedded_occupancy_enabled_flag(false)
        .asme_occupancy_scale_enabled_flag(true)
        .asme_occupancy_scale_factor_x_minus1(2)
        .asme_occupancy_scale_factor_y_minus1(3);

    REQUIRE(toString(unit) == R"(asme_ancillary_atlas_flag=false
asme_embedded_occupancy_enabled_flag=false
asme_geometry_scale_enabled_flag=false
asme_occupancy_scale_enabled_flag=true
asme_occupancy_scale_factor_x_minus1=2
asme_occupancy_scale_factor_y_minus1=3
asme_patch_constant_depth_flag=false
asme_patch_attribute_offset_enabled_flag=false
asme_max_entity_id=0
asme_inpaint_enabled_flag=false
)");
    REQUIRE(bitCodingTest(unit, 16));
  }
}

TEST_CASE("atlas_sequence_parameter_set_rbsp", "[Atlas Sequence Parameter Set RBSP]") {
  auto x = AtlasSequenceParameterSetRBSP{};

  REQUIRE(toString(x) == R"(asps_atlas_sequence_parameter_set_id=0
asps_frame_width=0
asps_frame_height=0
asps_geometry_3d_bit_depth_minus1=0
asps_geometry_2d_bit_depth_minus1=0
asps_log2_max_atlas_frame_order_cnt_lsb_minus4=0
asps_max_dec_atlas_frame_buffering_minus1=0
asps_long_term_ref_atlas_frames_flag=false
asps_num_ref_atlas_frame_lists_in_asps=0
asps_use_eight_orientations_flag=false
asps_extended_projection_enabled_flag=false
asps_normal_axis_limits_quantization_enabled_flag=false
asps_normal_axis_max_delta_value_enabled_flag=false
asps_patch_precedence_order_flag=false
asps_log2_patch_packing_block_size=0
asps_patch_size_quantizer_present_flag=false
asps_map_count_minus1=0
asps_pixel_deinterleaving_enabled_flag=false
asps_raw_patch_enabled_flag=false
asps_eom_patch_enabled_flag=false
asps_plr_enabled_flag=false
asps_vui_parameters_present_flag=false
asps_extension_present_flag=false
)");

  SECTION("Example 1") {
    const auto vuh = V3cUnitHeader{VuhUnitType::V3C_AD};
    auto vps = V3cParameterSet{};
    vps.vps_frame_width({}, 1).vps_frame_height({}, 1);

    x.asps_frame_width(1)
        .asps_frame_height(1)
        .asps_num_ref_atlas_frame_lists_in_asps(2)
        .asps_extension_present_flag(true);

    REQUIRE(toString(x) == R"(asps_atlas_sequence_parameter_set_id=0
asps_frame_width=1
asps_frame_height=1
asps_geometry_3d_bit_depth_minus1=0
asps_geometry_2d_bit_depth_minus1=0
asps_log2_max_atlas_frame_order_cnt_lsb_minus4=0
asps_max_dec_atlas_frame_buffering_minus1=0
asps_long_term_ref_atlas_frames_flag=false
asps_num_ref_atlas_frame_lists_in_asps=2
num_ref_entries( 0 )=0
num_ref_entries( 1 )=0
asps_use_eight_orientations_flag=false
asps_extended_projection_enabled_flag=false
asps_normal_axis_limits_quantization_enabled_flag=false
asps_normal_axis_max_delta_value_enabled_flag=false
asps_patch_precedence_order_flag=false
asps_log2_patch_packing_block_size=0
asps_patch_size_quantizer_present_flag=false
asps_map_count_minus1=0
asps_pixel_deinterleaving_enabled_flag=false
asps_raw_patch_enabled_flag=false
asps_eom_patch_enabled_flag=false
asps_plr_enabled_flag=false
asps_vui_parameters_present_flag=false
asps_extension_present_flag=true
asps_vpcc_extension_present_flag=false
asps_miv_extension_present_flag=false
asps_extension_6bits=0
)");

    REQUIRE(byteCodingTest(x, 7, vuh, vps));
  }

  SECTION("Example 2") {
    const auto vuh = V3cUnitHeader{VuhUnitType::V3C_AD};
    auto vps = V3cParameterSet{};
    vps.vps_atlas_id(0, {})
        .vps_frame_width({}, 0xFFFF)
        .vps_frame_height({}, 0xFFFF)
        .vps_map_count_minus1({}, 1)
        .vps_extension_present_flag(true)
        .vps_miv_extension_present_flag(true)
        .vps_miv_extension()
        .vme_geometry_scale_enabled_flag(true)
        .vme_embedded_occupancy_enabled_flag(true);

    x.asps_atlas_sequence_parameter_set_id(63)
        .asps_frame_width(0xFFFF)
        .asps_frame_height(0xFFFF)
        .asps_geometry_3d_bit_depth_minus1(10)
        .asps_geometry_2d_bit_depth_minus1(13)
        .asps_log2_patch_packing_block_size(7)
        .asps_log2_max_atlas_frame_order_cnt_lsb_minus4(12)
        .asps_max_dec_atlas_frame_buffering_minus1(41)
        .asps_long_term_ref_atlas_frames_flag(true)
        .asps_use_eight_orientations_flag(true)
        .asps_extended_projection_enabled_flag(true)
        .asps_max_number_projections_minus1(33)
        .asps_normal_axis_limits_quantization_enabled_flag(true)
        .asps_normal_axis_max_delta_value_enabled_flag(true)
        .asps_patch_precedence_order_flag(true)
        .asps_patch_size_quantizer_present_flag(true)
        .asps_map_count_minus1(1)
        .asps_extension_present_flag(true)
        .asps_vpcc_extension_present_flag(true)
        .asps_miv_extension_present_flag(true)
        .asps_extension_6bits(63);
    x.asps_vpcc_extension().asps_vpcc_remove_duplicate_point_enabled_flag(true);
    x.asps_miv_extension()
        .asme_ancillary_atlas_flag(true)
        .asme_embedded_occupancy_enabled_flag(true)
        .asme_depth_occ_threshold_flag(true)
        .asme_geometry_scale_enabled_flag(true)
        .asme_geometry_scale_factor_x_minus1(1)
        .asme_geometry_scale_factor_y_minus1(2)
        .asme_patch_constant_depth_flag(true);
    x.aspsExtensionData({false, true, true});

    REQUIRE(toString(x) == R"(asps_atlas_sequence_parameter_set_id=63
asps_frame_width=65535
asps_frame_height=65535
asps_geometry_3d_bit_depth_minus1=10
asps_geometry_2d_bit_depth_minus1=13
asps_log2_max_atlas_frame_order_cnt_lsb_minus4=12
asps_max_dec_atlas_frame_buffering_minus1=41
asps_long_term_ref_atlas_frames_flag=true
asps_num_ref_atlas_frame_lists_in_asps=0
asps_use_eight_orientations_flag=true
asps_extended_projection_enabled_flag=true
asps_max_number_projections_minus1=33
asps_normal_axis_limits_quantization_enabled_flag=true
asps_normal_axis_max_delta_value_enabled_flag=true
asps_patch_precedence_order_flag=true
asps_log2_patch_packing_block_size=7
asps_patch_size_quantizer_present_flag=true
asps_map_count_minus1=1
asps_pixel_deinterleaving_enabled_flag=false
asps_raw_patch_enabled_flag=false
asps_eom_patch_enabled_flag=false
asps_plr_enabled_flag=false
asps_vui_parameters_present_flag=false
asps_extension_present_flag=true
asps_vpcc_extension_present_flag=true
asps_miv_extension_present_flag=true
asps_extension_6bits=63
asps_vpcc_remove_duplicate_point_enabled_flag=true
asme_ancillary_atlas_flag=true
asme_embedded_occupancy_enabled_flag=true
asme_depth_occ_map_threshold_flag=true
asme_geometry_scale_enabled_flag=true
asme_geometry_scale_factor_x_minus1=1
asme_geometry_scale_factor_y_minus1=2
asme_patch_constant_depth_flag=true
asme_patch_attribute_offset_enabled_flag=false
asme_max_entity_id=0
asme_inpaint_enabled_flag=false
asps_extension_data_flag=false
asps_extension_data_flag=true
asps_extension_data_flag=true
)");

    REQUIRE(byteCodingTest(x, 21, vuh, vps));
  }
}
} // namespace TMIV::MivBitstream
