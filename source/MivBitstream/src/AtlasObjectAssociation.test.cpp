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

#include <TMIV/MivBitstream/AtlasObjectAssociation.h>

namespace TMIV::MivBitstream {
TEST_CASE("atlas_object_association", "[Atlas object association SEI payload syntax]") {
  SECTION("Default constructor") {
    const auto unit = AtlasObjectAssociation{};
    REQUIRE(toString(unit) == R"(aoa_persistence_flag=false
aoa_reset_flag=false
aoa_num_atlases_minus1=0
aoa_num_updates=0
)");
    const std::size_t expected_number_of_bits = 1    // aoa_persistence_flag
                                                + 1  // aoa_reset_flag
                                                + 6  // aoa_num_atlases_minus1
                                                + 1; // aoa_num_updates
    REQUIRE(bitCodingTest(unit, expected_number_of_bits));
  }

  SECTION("Set all fields") {
    AtlasObjectAssociationUpdateParameters aoa_parameters{};
    aoa_parameters.aoa_object_in_atlas = {{true, false}, {true, true}, {false, false}};
    aoa_parameters.aoa_log2_max_object_idx_tracked_minus1 = 2;
    aoa_parameters.aoa_object_idx = {2, 0, 1};
    aoa_parameters.aoa_atlas_id = {1, 0};
    const auto unit = AtlasObjectAssociation(true, false, std::move(aoa_parameters));
    REQUIRE(toString(unit) == R"(aoa_persistence_flag=true
aoa_reset_flag=false
aoa_num_atlases_minus1=1
aoa_num_updates=3
aoa_log2_max_object_idx_tracked_minus1=2
aoa_atlas_id(0)=1
aoa_atlas_id(1)=0
aoa_object_idx(0)=2
aoa_object_in_atlas(0, 0)=false
aoa_object_in_atlas(0, 1)=false
aoa_object_idx(1)=0
aoa_object_in_atlas(1, 0)=false
aoa_object_in_atlas(1, 1)=true
aoa_object_idx(2)=1
aoa_object_in_atlas(2, 0)=true
aoa_object_in_atlas(2, 1)=true
)");
    const std::size_t expected_number_of_bits = 1    // aoa_persistence_flag
                                                + 1  // aoa_reset_flag
                                                + 6  // aoa_num_atlases_minus1
                                                + 5  // aoa_num_updates
                                                + 5  // aoa_log2_max_object_idx_tracked_minus1
                                                + 12 // aoa_atlas_id
                                                + 9  // aoa_object_idx
                                                + 6; // aoa_object_in_atlas
    REQUIRE(bitCodingTest(unit, expected_number_of_bits));
  }
}

} // namespace TMIV::MivBitstream
