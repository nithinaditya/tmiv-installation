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

#include <TMIV/MivBitstream/PackedIndependentRegions.h>

namespace TMIV::MivBitstream {
TEST_CASE("packed_independent_regions", "[Packed Independent Regions SEI payload syntax]") {
  auto expected_number_of_bits = 5; // pir_num_packed_frames_minus1

  SECTION("Default constructor") {
    const PackedIndependentRegions unit{};
    REQUIRE(toString(unit) == R"(pir_num_packed_frames_minus1=0
pir_packed_frame_id(0)=0
pir_description_type_idc(0)=0
pir_num_regions_minus1(0)=0
pir_top_left_tile_idx(0,0)=0
pir_bottom_right_tile_idx(0,0)=0
)");
    expected_number_of_bits += 1 *  // pir_num_packed_frames_minus1 + 1
                               (5   // pir_packed_frame_id
                                + 2 // pir_description_type_idc
                                + 8 // pir_num_regions_minus1
                                + 1 // pir_top_left_tile_idx
                                + 1 // pir_bottom_right_tile_idx
                               );
    REQUIRE(bitCodingTest(unit, expected_number_of_bits));
  }

  SECTION("Frames with tile and subpic regions") {
    PackedIndependentRegions unit{};
    const std::uint8_t number_of_frames = 3;
    unit.pir_num_packed_frames_minus1(number_of_frames - 1);
    for (std::uint8_t j = 0; j < number_of_frames; ++j) {
      unit.pir_packed_frame_id(j, static_cast<uint8_t>(number_of_frames - j - 1));
      const auto k = unit.pir_packed_frame_id(j);
      unit.pir_description_type_idc(k, k % 2);
      unit.pir_num_regions_minus1(k, k % 3);
      for (std::uint8_t i = 0; i <= unit.pir_num_regions_minus1(k); ++i) {
        if (unit.pir_description_type_idc(k) == 0) {
          unit.pir_top_left_tile_idx(k, i, (k + size_t{1}) * (i + size_t{1}));
          unit.pir_bottom_right_tile_idx(k, i, (k + size_t{2}) * (i + size_t{3}));
        } else {
          unit.pir_subpic_id(k, i, (k + size_t{1}) * (i + size_t{1}));
        }
      }
    }

    REQUIRE(toString(unit) == R"(pir_num_packed_frames_minus1=2
pir_packed_frame_id(0)=2
pir_description_type_idc(2)=0
pir_num_regions_minus1(2)=2
pir_top_left_tile_idx(2,0)=3
pir_bottom_right_tile_idx(2,0)=12
pir_top_left_tile_idx(2,1)=6
pir_bottom_right_tile_idx(2,1)=16
pir_top_left_tile_idx(2,2)=9
pir_bottom_right_tile_idx(2,2)=20
pir_packed_frame_id(1)=1
pir_description_type_idc(1)=1
pir_num_regions_minus1(1)=1
pir_subpic_id(1,0)=2
pir_subpic_id(1,1)=4
pir_packed_frame_id(2)=0
pir_description_type_idc(0)=0
pir_num_regions_minus1(0)=0
pir_top_left_tile_idx(0,0)=1
pir_bottom_right_tile_idx(0,0)=6
)");

    expected_number_of_bits += number_of_frames * (5    // pir_packed_frame_id
                                                   + 2  // pir_description_type_idc
                                                   + 2  // pir_description_type_idc
                                                   + 8) // pir_num_regions_minus1
                               + 52; // variable number of bits for pir_top_left_tile_idx,
                                     // pir_bottom_right_tile_idx, and pir_subpic_id
    REQUIRE(bitCodingTest(unit, expected_number_of_bits));
  }
}
} // namespace TMIV::MivBitstream
