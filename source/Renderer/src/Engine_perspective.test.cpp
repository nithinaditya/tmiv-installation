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

#include <catch2/catch.hpp>

#include <TMIV/Renderer/Engine.h>

using Catch::Matchers::WithinAbs;
using TMIV::Common::Vec2f;
using TMIV::Common::Vec3f;
using TMIV::MivBitstream::CameraIntrinsics;
using TMIV::MivBitstream::CiCamType;
using TMIV::Renderer::SceneVertexDescriptor;

TEST_CASE("Engine<perspective>") {
  const auto unit = []() {
    auto ci = CameraIntrinsics{};
    ci.ci_cam_type(CiCamType::perspective)
        .ci_projection_plane_width_minus1(999)
        .ci_projection_plane_height_minus1(499)
        .ci_perspective_center_hor(250.F)
        .ci_perspective_center_ver(200.F)
        .ci_perspective_focal_hor(500.F)
        .ci_perspective_focal_ver(2000.F);
    return TMIV::Renderer::Engine<TMIV::MivBitstream::CiCamType::perspective>{ci};
  }();

  SECTION("Reprojection accuracy test") {
    const auto depth = GENERATE(0.1F, 10.F);
    int count = 0;

    for (int i = 0; i < 500; ++i) {
      for (int j = 0; j < 1000; ++j) {
        if (i * j % 1547 == 0) {
          const auto position =
              Vec2f{0.5F + static_cast<float>(j), 0.5F + static_cast<float>(i) + 0.5F};
          const auto p = unit.unprojectVertex(position, depth);
          const auto q = unit.projectVertex(SceneVertexDescriptor{p, 0.F});

          REQUIRE(q.depth == depth);
          REQUIRE_THAT(q.position.x(), WithinAbs(position.x(), 1E-4));
          REQUIRE_THAT(q.position.y(), WithinAbs(position.y(), 5E-5));

          // NOTE(BK, #148): On an internal branch with homogeneous coordinates, the margin could be
          // set to (1E-5, 1E-5) instead of (1E-4, 5E-5). This change has not yet been applied.

          ++count;
        }
      }
    }

    REQUIRE(count == 3103);
  }
}