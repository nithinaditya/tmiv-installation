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

#include <TMIV/MivBitstream/RecViewport.h>

namespace TMIV::MivBitstream {
TEST_CASE("rec_viewport", "[Rec Viewport SEI payload syntax]") {
  SECTION("Example 1") {
    RecViewport x = RecViewport{0, true};
    REQUIRE(toString(x) == R"(rec_viewport_id=0
rec_viewport_cancel_flag=true
)");
    REQUIRE(bitCodingTest(x, 11));
  }

  SECTION("Example 2") {
    auto x = RecViewport{};
    x.rec_viewport_id(1)
        .rec_viewport_cancel_flag(false)
        .rec_viewport_persistence_flag(false)
        .rec_viewport_center_view_flag(true)
        .rec_viewport_pos_x(0.2F)
        .rec_viewport_pos_y(1.45F)
        .rec_viewport_pos_z(-0.79F)
        .rec_viewport_quat_x(-0.91F)
        .rec_viewport_quat_y(0.F)
        .rec_viewport_quat_z(1.2F)
        .rec_viewport_hor_range(90.F)
        .rec_viewport_ver_range(60.4F);
    REQUIRE(toString(x) == R"(rec_viewport_id=1
rec_viewport_cancel_flag=false
rec_viewport_persistence_flag=false
rec_viewport_center_view_flag=true
rec_viewport_pos_x=0.2
rec_viewport_pos_y=1.45
rec_viewport_pos_z=-0.79
rec_viewport_quat_x=-0.91
rec_viewport_quat_y=0
rec_viewport_quat_z=1.2
rec_viewport_hor_range=90
rec_viewport_ver_range=60.4
)");
    REQUIRE(bitCodingTest(x, 269));
  }

  SECTION("Example 3") {
    auto x = RecViewport{};
    x.rec_viewport_id(2)
        .rec_viewport_cancel_flag(false)
        .rec_viewport_persistence_flag(false)
        .rec_viewport_center_view_flag(false)
        .rec_viewport_left_view_flag(true)
        .rec_viewport_pos_x(0.2F)
        .rec_viewport_pos_y(1.45F)
        .rec_viewport_pos_z(-0.79F)
        .rec_viewport_quat_x(-0.91F)
        .rec_viewport_quat_y(0.F)
        .rec_viewport_quat_z(1.2F)
        .rec_viewport_hor_range(90.F)
        .rec_viewport_ver_range(60.4F);
    REQUIRE(toString(x) == R"(rec_viewport_id=2
rec_viewport_cancel_flag=false
rec_viewport_persistence_flag=false
rec_viewport_center_view_flag=false
rec_viewport_left_view_flag=true
rec_viewport_pos_x=0.2
rec_viewport_pos_y=1.45
rec_viewport_pos_z=-0.79
rec_viewport_quat_x=-0.91
rec_viewport_quat_y=0
rec_viewport_quat_z=1.2
rec_viewport_hor_range=90
rec_viewport_ver_range=60.4
)");
    REQUIRE(bitCodingTest(x, 270));
  }
}
} // namespace TMIV::MivBitstream
