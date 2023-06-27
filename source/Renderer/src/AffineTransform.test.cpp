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

#include <TMIV/Renderer/reprojectPoints.h>

using TMIV::Common::Vec3f;
using TMIV::Renderer::AffineTransform;

TEST_CASE("AffineTransform") {
  const auto neutral = TMIV::MivBitstream::CameraExtrinsics{};

  auto translated = neutral;
  translated.ce_view_pos_x(1.F).ce_view_pos_y(2.F).ce_view_pos_z(3.F);

  auto rotated = neutral;
  rotated.ce_view_quat_x(0.1F).ce_view_quat_y(0.3F).ce_view_quat_z(-0.3F);

  SECTION("Construction") {
    auto nn = AffineTransform(neutral, neutral);
    auto nt = AffineTransform(neutral, translated);
    auto tn = AffineTransform(translated, neutral);
    auto nr = AffineTransform(neutral, rotated);
    auto rn = AffineTransform(rotated, neutral);

    SECTION("Translation vector") {
      REQUIRE(nn.translation() == Vec3f{});
      REQUIRE(nt.translation() == -translated.position());
      REQUIRE(tn.translation() == translated.position());
      REQUIRE(nr.translation() == Vec3f{});
      REQUIRE(rn.translation() == Vec3f{});
    }

    SECTION("Cartesian transformation") {
      const auto points = {Vec3f{},                  // Origin
                           Vec3f{1.F, 2.F, 3.F},     // Example point 1
                           Vec3f{-1.F, 0.3F, 0.4F}}; // Example point 2

      SECTION("Translation") {
        for (const auto x : points) {
          REQUIRE(nn(x) == x);
          REQUIRE(nt(x) == x - translated.position());
          REQUIRE(tn(x) == x + translated.position());
        }
      }

      SECTION("Rotation") {
        for (const auto x : points) {
          const auto nr_x_ref = rotate(x, conj(rotated.rotation()));
          const auto rn_x_ref = rotate(x, rotated.rotation());

          for (int d = 0; d < 3; ++d) {
            REQUIRE(nr(x)[d] == Approx(nr_x_ref[d]));
            REQUIRE(rn(x)[d] == Approx(rn_x_ref[d]));
          }
        }
      }
    }
  }
}
