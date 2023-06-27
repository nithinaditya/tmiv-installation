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

#include <TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h>

namespace TMIV::MivBitstream {
TEST_CASE("atlas_frame_tile_information", "[Atlas Frame Parameter Set RBSP]") {
  const auto x = AtlasFrameTileInformation{};
  REQUIRE(toString(x) == R"(afti_single_tile_in_atlas_frame_flag=true
afti_signalled_tile_id_flag=false
)");
  REQUIRE(bitCodingTest(x, 2));
}

TEST_CASE("atlas_frame_parameter_set_rbsp", "[Atlas Frame Parameter Set RBSP]") {
  auto x = AtlasFrameParameterSetRBSP{};

  REQUIRE(toString(x) == R"(afps_atlas_frame_parameter_set_id=0
afps_atlas_sequence_parameter_set_id=0
afti_single_tile_in_atlas_frame_flag=true
afti_signalled_tile_id_flag=false
afps_output_flag_present_flag=false
afps_num_ref_idx_default_active_minus1=0
afps_additional_lt_afoc_lsb_len=0
afps_lod_mode_enabled_flag=false
afps_raw_3d_offset_bit_count_explicit_mode_flag=false
afps_extension_present_flag=false
)");

  SECTION("Example 1") {
    const auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(1);

    REQUIRE(toString(x) == R"(afps_atlas_frame_parameter_set_id=0
afps_atlas_sequence_parameter_set_id=0
afti_single_tile_in_atlas_frame_flag=true
afti_signalled_tile_id_flag=false
afps_output_flag_present_flag=false
afps_num_ref_idx_default_active_minus1=0
afps_additional_lt_afoc_lsb_len=0
afps_lod_mode_enabled_flag=false
afps_raw_3d_offset_bit_count_explicit_mode_flag=false
afps_extension_present_flag=false
)");

    REQUIRE(byteCodingTest(x, 2, aspsV));
  }

  SECTION("Example 2") {
    auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(3);
    aspsV.back()
        .asps_atlas_sequence_parameter_set_id(63)
        .asps_geometry_3d_bit_depth_minus1(31)
        .asps_log2_max_atlas_frame_order_cnt_lsb_minus4(3)
        .asps_long_term_ref_atlas_frames_flag(true);

    x.afps_atlas_frame_parameter_set_id(63)
        .afps_atlas_sequence_parameter_set_id(63)
        .afps_output_flag_present_flag(true)
        .afps_num_ref_idx_default_active_minus1(14)
        .afps_additional_lt_afoc_lsb_len(25)
        .afps_lod_mode_enabled_flag(true)
        .afps_raw_3d_offset_bit_count_explicit_mode_flag(true)
        .afps_extension_present_flag(true)
        .afps_miv_extension_present_flag(true)
        .afps_extension_7bits(127)
        .afpsExtensionData({false, true})
        .afps_miv_extension() = {};

    REQUIRE(toString(x) == R"(afps_atlas_frame_parameter_set_id=63
afps_atlas_sequence_parameter_set_id=63
afti_single_tile_in_atlas_frame_flag=true
afti_signalled_tile_id_flag=false
afps_output_flag_present_flag=true
afps_num_ref_idx_default_active_minus1=14
afps_additional_lt_afoc_lsb_len=25
afps_lod_mode_enabled_flag=true
afps_raw_3d_offset_bit_count_explicit_mode_flag=true
afps_extension_present_flag=true
afps_miv_extension_present_flag=true
afps_extension_7bits=127
afps_extension_data_flag=false
afps_extension_data_flag=true
)");

    REQUIRE(byteCodingTest(x, 8, aspsV));
  }

  SECTION("Example 3") {
    auto aspsV = std::vector<AtlasSequenceParameterSetRBSP>(3);
    aspsV.back()
        .asps_atlas_sequence_parameter_set_id(63)
        .asps_geometry_3d_bit_depth_minus1(31)
        .asps_log2_max_atlas_frame_order_cnt_lsb_minus4(3)
        .asps_long_term_ref_atlas_frames_flag(true);

    x.afps_atlas_frame_parameter_set_id(63)
        .afps_atlas_sequence_parameter_set_id(63)
        .afps_output_flag_present_flag(true)
        .afps_num_ref_idx_default_active_minus1(14)
        .afps_additional_lt_afoc_lsb_len(25)
        .afps_lod_mode_enabled_flag(false)
        .afps_raw_3d_offset_bit_count_explicit_mode_flag(true)
        .afps_extension_present_flag(true)
        .afps_miv_extension_present_flag(true)
        .afps_miv_extension()
        .afme_inpaint_lod_enabled_flag(true)
        .afme_inpaint_lod_scale_x_minus1(4)
        .afme_inpaint_lod_scale_y_idc(13);

    REQUIRE(toString(x) == R"(afps_atlas_frame_parameter_set_id=63
afps_atlas_sequence_parameter_set_id=63
afti_single_tile_in_atlas_frame_flag=true
afti_signalled_tile_id_flag=false
afps_output_flag_present_flag=true
afps_num_ref_idx_default_active_minus1=14
afps_additional_lt_afoc_lsb_len=25
afps_lod_mode_enabled_flag=false
afps_raw_3d_offset_bit_count_explicit_mode_flag=true
afps_extension_present_flag=true
afps_miv_extension_present_flag=true
afps_extension_7bits=0
afme_inpaint_lod_enabled_flag=true
afme_inpaint_lod_scale_x_minus1=4
afme_inpaint_lod_scale_y_idc=13
)");

    REQUIRE(byteCodingTest(x, 9, aspsV));
  }
}
} // namespace TMIV::MivBitstream
