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

#include <TMIV/MivBitstream/AtlasTileLayerRBSP.h>

namespace TMIV::MivBitstream {
TEST_CASE("atlas_tile_header", "[Atlas Tile Layer RBSP]") {
  auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
  aspsV.front().asps_num_ref_atlas_frame_lists_in_asps(1).asps_log2_patch_packing_block_size(7);

  const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);

  auto x = AtlasTileHeader{};

  REQUIRE(toString(x) == R"(ath_atlas_frame_parameter_set_id=0
ath_atlas_adaptation_parameter_set_id=0
ath_id=0
ath_type=P_TILE
ath_atlas_frm_order_cnt_lsb=0
)");

  SECTION("Example 1") {
    x.ath_no_output_of_prior_atlas_frames_flag(true)
        .ath_type(AthType::SKIP_TILE)
        .ath_ref_atlas_frame_list_asps_flag(true);

    const auto nuh = NalUnitHeader{NalUnitType::NAL_IDR_N_LP, 0, 1};

    REQUIRE(toString(x) == R"(ath_no_output_of_prior_atlas_frames_flag=true
ath_atlas_frame_parameter_set_id=0
ath_atlas_adaptation_parameter_set_id=0
ath_id=0
ath_type=SKIP_TILE
ath_atlas_frm_order_cnt_lsb=0
ath_ref_atlas_frame_list_asps_flag=true
)");

    REQUIRE(bitCodingTest(x, 16, nuh, aspsV, afpsV));
  }

  SECTION("Example 2") {
    aspsV.front()
        .asps_patch_size_quantizer_present_flag(true)
        .asps_normal_axis_limits_quantization_enabled_flag(true)
        .asps_normal_axis_max_delta_value_enabled_flag(true);

    x.ath_type(AthType::I_TILE)
        .ath_ref_atlas_frame_list_asps_flag(true)
        .ath_patch_size_x_info_quantizer(6)
        .ath_patch_size_y_info_quantizer(5)
        .ath_atlas_adaptation_parameter_set_id(63)
        .ath_pos_min_d_quantizer(3)
        .ath_pos_delta_max_d_quantizer(7);

    const auto nuh = NalUnitHeader{NalUnitType::NAL_TSA_R, 0, 2};

    REQUIRE(toString(x) == R"(ath_atlas_frame_parameter_set_id=0
ath_atlas_adaptation_parameter_set_id=63
ath_id=0
ath_type=I_TILE
ath_atlas_frm_order_cnt_lsb=0
ath_ref_atlas_frame_list_asps_flag=true
ath_pos_min_d_quantizer=3
ath_pos_delta_max_d_quantizer=7
ath_patch_size_x_info_quantizer=6
ath_patch_size_y_info_quantizer=5
)");

    REQUIRE(bitCodingTest(x, 40, nuh, aspsV, afpsV));
  }
}

TEST_CASE("skip_patch_data_unit", "[Atlas Tile Layer RBSP]") {
  auto x = SkipPatchDataUnit{};
  REQUIRE(toString(x).empty());
  REQUIRE(bitCodingTest(x, 0));
}

TEST_CASE("patch_data_unit", "[Atlas Tile Layer RBSP]") {
  auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
  const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);
  const auto ath = AtlasTileHeader{};

  auto x = PatchDataUnit{};

  REQUIRE(toString(x, 0, 101) == R"(pdu_2d_pos_x[ 0 ][ 101 ]=0
pdu_2d_pos_y[ 0 ][ 101 ]=0
pdu_2d_size_x_minus1[ 0 ][ 101 ]=0
pdu_2d_size_y_minus1[ 0 ][ 101 ]=0
pdu_3d_offset_u[ 0 ][ 101 ]=0
pdu_3d_offset_v[ 0 ][ 101 ]=0
pdu_3d_offset_d[ 0 ][ 101 ]=0
pdu_projection_id[ 0 ][ 101 ]=0
pdu_orientation_index[ 0 ][ 101 ]=FPO_NULL
)");

  REQUIRE(bitCodingTest(x, 11, aspsV, afpsV, ath));

  SECTION("Example 1") {
    aspsV.front()
        .asps_geometry_3d_bit_depth_minus1(14)
        .asps_geometry_2d_bit_depth_minus1(9)
        .asps_use_eight_orientations_flag(true)
        .asps_normal_axis_max_delta_value_enabled_flag(true)
        .asps_extended_projection_enabled_flag(true)
        .asps_max_number_projections_minus1(511)
        .asps_extension_present_flag(true)
        .asps_miv_extension_present_flag(true)
        .asps_miv_extension()
        .asme_embedded_occupancy_enabled_flag(true)
        .asme_depth_occ_threshold_flag(true)
        .asme_max_entity_id(100)
        .asme_patch_attribute_offset_enabled_flag(true)
        .asme_patch_attribute_offset_bit_depth_minus1(5)
        .asme_inpaint_enabled_flag(true);

    x.pdu_2d_pos_x(34)
        .pdu_2d_pos_y(57)
        .pdu_2d_size_x_minus1(1234)
        .pdu_2d_size_y_minus1(1002)
        .pdu_3d_offset_u(1234)
        .pdu_3d_offset_v(21345)
        .pdu_3d_offset_d(623)
        .pdu_3d_range_d(789)
        .pdu_projection_id(300)
        .pdu_orientation_index(FlexiblePatchOrientation::FPO_MROT180)
        .pdu_miv_extension()
        .pdu_entity_id(35)
        .pdu_depth_occ_threshold(600)
        .pdu_attribute_offset(Common::Vec3w{4, 5, 6})
        .pdu_inpaint_flag(false);

    REQUIRE(toString(x, 12, 102) == R"(pdu_2d_pos_x[ 12 ][ 102 ]=34
pdu_2d_pos_y[ 12 ][ 102 ]=57
pdu_2d_size_x_minus1[ 12 ][ 102 ]=1234
pdu_2d_size_y_minus1[ 12 ][ 102 ]=1002
pdu_3d_offset_u[ 12 ][ 102 ]=1234
pdu_3d_offset_v[ 12 ][ 102 ]=21345
pdu_3d_offset_d[ 12 ][ 102 ]=623
pdu_3d_range_d[ 12 ][ 102 ]=789
pdu_projection_id[ 12 ][ 102 ]=300
pdu_orientation_index[ 12 ][ 102 ]=FPO_MROT180
pdu_entity_id[ 12 ][ 102 ]=35
pdu_depth_occ_threshold[ 12 ][ 102 ]=600
pdu_attribute_offset[ 12 ][ 102 ][ 0 ]=4
pdu_attribute_offset[ 12 ][ 102 ][ 1 ]=5
pdu_attribute_offset[ 12 ][ 102 ][ 2 ]=6
pdu_inpaint_flag[ 12 ][ 102 ]=false
)");

    REQUIRE(bitCodingTest(x, 165, aspsV, afpsV, ath));
  }

  SECTION("Extend with only pdu_entity_id") {
    aspsV.front()
        .asps_geometry_2d_bit_depth_minus1(9)
        .asps_geometry_3d_bit_depth_minus1(14)
        .asps_use_eight_orientations_flag(true)
        .asps_normal_axis_max_delta_value_enabled_flag(true)
        .asps_extended_projection_enabled_flag(true)
        .asps_max_number_projections_minus1(511)
        .asps_extension_present_flag(true)
        .asps_miv_extension_present_flag(true)
        .asps_miv_extension()
        .asme_max_entity_id(100);

    // Create ASME with default values
    static_cast<void>(aspsV.front().asps_miv_extension());

    x.pdu_2d_pos_x(34)
        .pdu_2d_pos_y(57)
        .pdu_2d_size_x_minus1(1234)
        .pdu_2d_size_y_minus1(1002)
        .pdu_3d_offset_u(1234)
        .pdu_3d_offset_v(21345)
        .pdu_3d_offset_d(623)
        .pdu_3d_range_d(789)
        .pdu_projection_id(300)
        .pdu_orientation_index(FlexiblePatchOrientation::FPO_MROT180)
        .pdu_miv_extension()
        .pdu_entity_id(35);

    REQUIRE(toString(x, 12, 102) == R"(pdu_2d_pos_x[ 12 ][ 102 ]=34
pdu_2d_pos_y[ 12 ][ 102 ]=57
pdu_2d_size_x_minus1[ 12 ][ 102 ]=1234
pdu_2d_size_y_minus1[ 12 ][ 102 ]=1002
pdu_3d_offset_u[ 12 ][ 102 ]=1234
pdu_3d_offset_v[ 12 ][ 102 ]=21345
pdu_3d_offset_d[ 12 ][ 102 ]=623
pdu_3d_range_d[ 12 ][ 102 ]=789
pdu_projection_id[ 12 ][ 102 ]=300
pdu_orientation_index[ 12 ][ 102 ]=FPO_MROT180
pdu_entity_id[ 12 ][ 102 ]=35
)");

    REQUIRE(bitCodingTest(x, 136, aspsV, afpsV, ath));
  }
}

TEST_CASE("patch_information_data", "[Atlas Tile Layer RBSP]") {
  auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
  aspsV.front().asps_frame_width(4000).asps_frame_height(2000);

  const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);

  const auto pdu = PatchDataUnit{};

  auto x = PatchInformationData{};

  REQUIRE(toString(x, 0, 77) == R"([unknown]
)");

  SECTION("Example skip_patch_data_unit") {
    auto ath = AtlasTileHeader{};
    ath.ath_type(AthType::SKIP_TILE);
    const auto patchMode = AtduPatchMode::P_SKIP;
    x = PatchInformationData{SkipPatchDataUnit{}};

    REQUIRE(toString(x, 77, 88).empty());
    REQUIRE(bitCodingTest(x, 0, aspsV, afpsV, ath, patchMode));
  }

  SECTION("Example patch_data_unit") {
    auto ath = AtlasTileHeader{};
    ath.ath_type(AthType::I_TILE);
    const auto patchMode = AtduPatchMode::I_INTRA;
    x = PatchInformationData{pdu};

    REQUIRE(toString(x, 13, 99) == R"(pdu_2d_pos_x[ 13 ][ 99 ]=0
pdu_2d_pos_y[ 13 ][ 99 ]=0
pdu_2d_size_x_minus1[ 13 ][ 99 ]=0
pdu_2d_size_y_minus1[ 13 ][ 99 ]=0
pdu_3d_offset_u[ 13 ][ 99 ]=0
pdu_3d_offset_v[ 13 ][ 99 ]=0
pdu_3d_offset_d[ 13 ][ 99 ]=0
pdu_projection_id[ 13 ][ 99 ]=0
pdu_orientation_index[ 13 ][ 99 ]=FPO_NULL
)");
    REQUIRE(bitCodingTest(x, 11, aspsV, afpsV, ath, patchMode));
  }
}

TEST_CASE("atlas_tile_data_unit", "[Atlas Tile Layer RBSP]") {
  SECTION("Empty") {
    auto ath = AtlasTileHeader{};
    ath.ath_type(AthType::I_TILE);

    const auto x = AtlasTileDataUnit{};
    REQUIRE(toString(x, ath).empty());

    const auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
    const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);

    REQUIRE(bitCodingTest(x, 7, aspsV, afpsV, ath));
  }

  SECTION("P_TILE") {
    auto vec = AtlasTileDataUnit::Vector{
        {AtduPatchMode::P_SKIP, PatchInformationData{SkipPatchDataUnit{}}},
        {AtduPatchMode::P_SKIP, PatchInformationData{SkipPatchDataUnit{}}},
        {AtduPatchMode::P_INTRA, PatchInformationData{PatchDataUnit{}}},
        {AtduPatchMode::P_SKIP, PatchInformationData{SkipPatchDataUnit{}}}};

    auto ath = AtlasTileHeader{};
    ath.ath_type(AthType::P_TILE);

    const auto x = AtlasTileDataUnit{vec};

    REQUIRE(toString(x, ath) == R"(atdu_patch_mode[ 0 ]=P_SKIP
atdu_patch_mode[ 1 ]=P_SKIP
atdu_patch_mode[ 2 ]=P_INTRA
pdu_2d_pos_x[ 0 ][ 2 ]=0
pdu_2d_pos_y[ 0 ][ 2 ]=0
pdu_2d_size_x_minus1[ 0 ][ 2 ]=0
pdu_2d_size_y_minus1[ 0 ][ 2 ]=0
pdu_3d_offset_u[ 0 ][ 2 ]=0
pdu_3d_offset_v[ 0 ][ 2 ]=0
pdu_3d_offset_d[ 0 ][ 2 ]=0
pdu_projection_id[ 0 ][ 2 ]=0
pdu_orientation_index[ 0 ][ 2 ]=FPO_NULL
atdu_patch_mode[ 3 ]=P_SKIP
)");
  }

  SECTION("I_TILE") {
    const auto pdu = PatchDataUnit{};

    auto vec = AtlasTileDataUnit::Vector{{AtduPatchMode::I_INTRA, PatchInformationData{pdu}},
                                         {AtduPatchMode::I_INTRA, PatchInformationData{pdu}}};

    auto ath = AtlasTileHeader{};
    ath.ath_type(AthType::I_TILE).ath_id(7);

    const auto x = AtlasTileDataUnit{vec};

    REQUIRE(toString(x, ath) == R"(atdu_patch_mode[ 0 ]=I_INTRA
pdu_2d_pos_x[ 7 ][ 0 ]=0
pdu_2d_pos_y[ 7 ][ 0 ]=0
pdu_2d_size_x_minus1[ 7 ][ 0 ]=0
pdu_2d_size_y_minus1[ 7 ][ 0 ]=0
pdu_3d_offset_u[ 7 ][ 0 ]=0
pdu_3d_offset_v[ 7 ][ 0 ]=0
pdu_3d_offset_d[ 7 ][ 0 ]=0
pdu_projection_id[ 7 ][ 0 ]=0
pdu_orientation_index[ 7 ][ 0 ]=FPO_NULL
atdu_patch_mode[ 1 ]=I_INTRA
pdu_2d_pos_x[ 7 ][ 1 ]=0
pdu_2d_pos_y[ 7 ][ 1 ]=0
pdu_2d_size_x_minus1[ 7 ][ 1 ]=0
pdu_2d_size_y_minus1[ 7 ][ 1 ]=0
pdu_3d_offset_u[ 7 ][ 1 ]=0
pdu_3d_offset_v[ 7 ][ 1 ]=0
pdu_3d_offset_d[ 7 ][ 1 ]=0
pdu_projection_id[ 7 ][ 1 ]=0
pdu_orientation_index[ 7 ][ 1 ]=FPO_NULL
)");

    auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
    aspsV.front().asps_frame_width(4000).asps_frame_height(2000);

    const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);

    REQUIRE(bitCodingTest(x, 31, aspsV, afpsV, ath));
  }
}

TEST_CASE("atlas_tile_layer_rbsp", "[Atlas Tile Layer RBSP]") {
  SECTION("SKIP_TILE") {
    auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
    aspsV.front()
        .asps_frame_width(4000)
        .asps_frame_height(2000)
        .asps_num_ref_atlas_frame_lists_in_asps(1);

    const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);

    const auto nuh = NalUnitHeader{NalUnitType::NAL_TSA_R, 0, 2};

    auto x = AtlasTileLayerRBSP{};
    x.atlas_tile_header().ath_type(AthType::SKIP_TILE).ath_ref_atlas_frame_list_asps_flag(true);

    REQUIRE(toString(x) == R"(ath_atlas_frame_parameter_set_id=0
ath_atlas_adaptation_parameter_set_id=0
ath_id=0
ath_type=SKIP_TILE
ath_atlas_frm_order_cnt_lsb=0
ath_ref_atlas_frame_list_asps_flag=true
)");
    REQUIRE(byteCodingTest(x, 3, nuh, aspsV, afpsV));
  }

  SECTION("I_TILE") {
    auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
    aspsV.front()
        .asps_frame_width(4000)
        .asps_frame_height(2000)
        .asps_num_ref_atlas_frame_lists_in_asps(1);

    auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);
    afpsV.front().afps_lod_mode_enabled_flag(true);

    auto pdu1 = PatchDataUnit{};
    pdu1.pdu_2d_size_x_minus1(10).pdu_2d_size_y_minus1(20).pdu_lod_enabled_flag(false);
    auto pdu2 = PatchDataUnit{};
    pdu2.pdu_2d_size_x_minus1(30).pdu_2d_size_y_minus1(40).pdu_lod_enabled_flag(false);
    auto pdu3 = PatchDataUnit{};
    pdu3.pdu_2d_size_x_minus1(50)
        .pdu_2d_size_y_minus1(60)
        .pdu_lod_enabled_flag(true)
        .pdu_lod_scale_x_minus1(1)
        .pdu_lod_scale_y_idc(3);

    const auto nuh = NalUnitHeader{NalUnitType::NAL_RSV_ACL_35, 0, 2};

    auto x = AtlasTileLayerRBSP{};
    x.atlas_tile_header().ath_type(AthType::I_TILE).ath_ref_atlas_frame_list_asps_flag(true);
    x.atlas_tile_data_unit() =
        AtlasTileDataUnit{std::pair{AtduPatchMode::I_INTRA, PatchInformationData{pdu1}},
                          std::pair{AtduPatchMode::I_INTRA, PatchInformationData{pdu2}},
                          std::pair{AtduPatchMode::I_INTRA, PatchInformationData{pdu3}}};

    REQUIRE(toString(x) == R"(ath_atlas_frame_parameter_set_id=0
ath_atlas_adaptation_parameter_set_id=0
ath_id=0
ath_type=I_TILE
ath_atlas_frm_order_cnt_lsb=0
ath_ref_atlas_frame_list_asps_flag=true
atdu_patch_mode[ 0 ]=I_INTRA
pdu_2d_pos_x[ 0 ][ 0 ]=0
pdu_2d_pos_y[ 0 ][ 0 ]=0
pdu_2d_size_x_minus1[ 0 ][ 0 ]=10
pdu_2d_size_y_minus1[ 0 ][ 0 ]=20
pdu_3d_offset_u[ 0 ][ 0 ]=0
pdu_3d_offset_v[ 0 ][ 0 ]=0
pdu_3d_offset_d[ 0 ][ 0 ]=0
pdu_projection_id[ 0 ][ 0 ]=0
pdu_orientation_index[ 0 ][ 0 ]=FPO_NULL
pdu_lod_enabled_flag[ 0 ][ 0 ]=false
atdu_patch_mode[ 1 ]=I_INTRA
pdu_2d_pos_x[ 0 ][ 1 ]=0
pdu_2d_pos_y[ 0 ][ 1 ]=0
pdu_2d_size_x_minus1[ 0 ][ 1 ]=30
pdu_2d_size_y_minus1[ 0 ][ 1 ]=40
pdu_3d_offset_u[ 0 ][ 1 ]=0
pdu_3d_offset_v[ 0 ][ 1 ]=0
pdu_3d_offset_d[ 0 ][ 1 ]=0
pdu_projection_id[ 0 ][ 1 ]=0
pdu_orientation_index[ 0 ][ 1 ]=FPO_NULL
pdu_lod_enabled_flag[ 0 ][ 1 ]=false
atdu_patch_mode[ 2 ]=I_INTRA
pdu_2d_pos_x[ 0 ][ 2 ]=0
pdu_2d_pos_y[ 0 ][ 2 ]=0
pdu_2d_size_x_minus1[ 0 ][ 2 ]=50
pdu_2d_size_y_minus1[ 0 ][ 2 ]=60
pdu_3d_offset_u[ 0 ][ 2 ]=0
pdu_3d_offset_v[ 0 ][ 2 ]=0
pdu_3d_offset_d[ 0 ][ 2 ]=0
pdu_projection_id[ 0 ][ 2 ]=0
pdu_orientation_index[ 0 ][ 2 ]=FPO_NULL
pdu_lod_enabled_flag[ 0 ][ 2 ]=true
pdu_lod_scale_x_minus1[ 0 ][ 2 ]=1
pdu_lod_scale_y_idc[ 0 ][ 2 ]=3
)");
    REQUIRE(byteCodingTest(x, 16, nuh, aspsV, afpsV));
  }

  SECTION("I_TILE with quantizers") {
    auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);
    aspsV.front()
        .asps_geometry_2d_bit_depth_minus1(11)
        .asps_geometry_3d_bit_depth_minus1(11)
        .asps_num_ref_atlas_frame_lists_in_asps(1)
        .asps_normal_axis_limits_quantization_enabled_flag(true)
        .asps_normal_axis_max_delta_value_enabled_flag(true);

    const auto afpsV = std::vector<AtlasFrameParameterSetRBSP>(1);

    auto pdu1 = PatchDataUnit{};
    pdu1.pdu_2d_size_x_minus1(10).pdu_2d_size_y_minus1(20).pdu_3d_offset_d(31).pdu_3d_range_d(127);

    const auto nuh = NalUnitHeader{NalUnitType::NAL_BLA_W_LP, 0, 1};

    auto x = AtlasTileLayerRBSP{};
    x.atlas_tile_header()
        .ath_no_output_of_prior_atlas_frames_flag(false)
        .ath_type(AthType::I_TILE)
        .ath_ref_atlas_frame_list_asps_flag(true)
        .ath_pos_min_d_quantizer(7)
        .ath_pos_delta_max_d_quantizer(5);
    x.atlas_tile_data_unit() =
        AtlasTileDataUnit{std::pair{AtduPatchMode::I_INTRA, PatchInformationData{pdu1}}};

    REQUIRE(toString(x) == R"(ath_no_output_of_prior_atlas_frames_flag=false
ath_atlas_frame_parameter_set_id=0
ath_atlas_adaptation_parameter_set_id=0
ath_id=0
ath_type=I_TILE
ath_atlas_frm_order_cnt_lsb=0
ath_ref_atlas_frame_list_asps_flag=true
ath_pos_min_d_quantizer=7
ath_pos_delta_max_d_quantizer=5
atdu_patch_mode[ 0 ]=I_INTRA
pdu_2d_pos_x[ 0 ][ 0 ]=0
pdu_2d_pos_y[ 0 ][ 0 ]=0
pdu_2d_size_x_minus1[ 0 ][ 0 ]=10
pdu_2d_size_y_minus1[ 0 ][ 0 ]=20
pdu_3d_offset_u[ 0 ][ 0 ]=0
pdu_3d_offset_v[ 0 ][ 0 ]=0
pdu_3d_offset_d[ 0 ][ 0 ]=31
pdu_3d_range_d[ 0 ][ 0 ]=127
pdu_projection_id[ 0 ][ 0 ]=0
pdu_orientation_index[ 0 ][ 0 ]=FPO_NULL
)");
    REQUIRE(byteCodingTest(x, 12, nuh, aspsV, afpsV));
  }
}
} // namespace TMIV::MivBitstream
