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

#include <TMIV/Renderer/MpiRasterizer.h>

using TMIV::Common::Mat;
using TMIV::Common::Vec2f;
using TMIV::Common::Vec2i;
using TMIV::Renderer::ImageVertexDescriptorList;
using TMIV::Renderer::MpiRasterizer;
using TMIV::Renderer::PixelAttributes;
using TMIV::Renderer::TriangleDescriptorList;
using TMIV::Renderer::ViewportPosition2D;

SCENARIO("MPI rastering meshes with Common::Vec2f as attribute", "[Rasterizer]") {
  GIVEN("A new MPI rasterizer") {
    MpiRasterizer<Vec2f, float, unsigned, unsigned> rasterizer(Vec2i{8, 4});
    Mat<float> atlasColor({2, 2});
    std::fill(atlasColor.begin(), atlasColor.end(), 0.F);
    atlasColor(0, 0) = 10.F;
    atlasColor(0, 1) = 20.F;
    atlasColor(1, 1) = 30.F;
    atlasColor(1, 0) = 40.F;
    Mat<float> blendedOutput({4, 8}, 0.F);
    Mat<float> blendedDepth({4, 8}, -1.F);

    WHEN("Rastering a quad for color and depth") {
      // Rectangle of 2 x 6 pixels within the 4 x 8 pixels frame
      // Depth values : not used
      // Ray angles : not used
      ImageVertexDescriptorList vs{
          {{1, 1}, 0.F, 0.F}, {{7, 1}, 0.F, 0.F}, {{7, 3}, 0.F, 0.F}, {{1, 3}, 0.F, 0.F}};
      // Two clockwise triangles
      TriangleDescriptorList ts{{{0, 1, 2}, 6.F}, {{0, 2, 3}, 6.F}};

      // Attribute #0 -> Atlas position
      std::vector<Vec2f> as0{{0.0F, 0.0F}, {1.0F, 0.0F}, {1.0F, 1.0F}, {0.0F, 1.0F}};
      // Attribute #1 -> Viewport depth
      std::vector<float> as1{10.0F, 20.0F, 30.0F, 40.0F};
      // Attribute #2 -> Patch ID
      std::vector<unsigned> as2{0U, 0U, 0U, 0U};
      // Attribute #3 -> Atlas ID
      std::vector<unsigned> as3{0U, 0U, 0U, 0U};
      auto as = std::tuple{as0, as1, as2, as3};
      using PixelAttribute = PixelAttributes<Vec2f, float, unsigned, unsigned>;

      rasterizer.submit(vs, as, ts);
      rasterizer.run([&](const ViewportPosition2D &viewport, const std::array<float, 3> &weights,
                         const std::array<PixelAttribute, 3> pixelAttributes) {
        auto texCoord = weights[0] * std::get<0>(pixelAttributes[0]) +
                        weights[1] * std::get<0>(pixelAttributes[1]) +
                        weights[2] * std::get<0>(pixelAttributes[2]);
        auto z = weights[0] * std::get<1>(pixelAttributes[0]) +
                 weights[1] * std::get<1>(pixelAttributes[1]) +
                 weights[2] * std::get<1>(pixelAttributes[2]);
        int x0 = std::clamp(static_cast<int>(std::llround(texCoord.x())), 0, 1);
        int y0 = std::clamp(static_cast<int>(std::llround(texCoord.y())), 0, 1);
        blendedOutput(viewport.y, viewport.x) = atlasColor(y0, x0);
        blendedDepth(viewport.y, viewport.x) = z;
        return 0;
      });
      THEN("The blended color has known values") {
        REQUIRE(blendedOutput(0, 0) == Approx(0.F));
        REQUIRE(blendedOutput(1, 1) == Approx(10.F));
        REQUIRE(blendedOutput(1, 2) == Approx(10.F));
        REQUIRE(blendedOutput(1, 3) == Approx(10.F));
        REQUIRE(blendedOutput(1, 4) == Approx(20.F));
        REQUIRE(blendedOutput(1, 5) == Approx(20.F));
        REQUIRE(blendedOutput(1, 6) == Approx(20.F));
        REQUIRE(blendedOutput(2, 1) == Approx(40.F));
        REQUIRE(blendedOutput(2, 2) == Approx(40.F));
        REQUIRE(blendedOutput(2, 3) == Approx(40.F));
        REQUIRE(blendedOutput(2, 4) == Approx(30.F));
        REQUIRE(blendedOutput(2, 5) == Approx(30.F));
        REQUIRE(blendedOutput(2, 6) == Approx(30.F));
      }
      THEN("The blended depth has known values") {
        REQUIRE(blendedDepth(0, 0) == Approx(-1.F));
        REQUIRE(blendedDepth(1, 1) == Approx(16.6666F));
        REQUIRE(blendedDepth(1, 2) == Approx(15.F));
        REQUIRE(blendedDepth(1, 3) == Approx(16.6666F));
        REQUIRE(blendedDepth(1, 4) == Approx(18.3333F));
        REQUIRE(blendedDepth(1, 5) == Approx(20.F));
        REQUIRE(blendedDepth(1, 6) == Approx(21.6666F));
        REQUIRE(blendedDepth(2, 1) == Approx(31.6666F));
        REQUIRE(blendedDepth(2, 2) == Approx(30.F));
        REQUIRE(blendedDepth(2, 3) == Approx(28.3333F));
        REQUIRE(blendedDepth(2, 4) == Approx(26.6666F));
        REQUIRE(blendedDepth(2, 5) == Approx(25.F));
        REQUIRE(blendedDepth(2, 6) == Approx(26.6666F));
      }
    }
  }
}
