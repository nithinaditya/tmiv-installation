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

#include <TMIV/Common/Frame.h>

namespace TMIV::Common {
namespace {
void checkLuminancePlaneContents(const Frame<YUV420P8> &unit, const int expectedPixelValue = 0) {
  const auto &yPlane = unit.getPlane(0);
  REQUIRE(!yPlane.empty());
  REQUIRE(yPlane.width() == unit.getWidth());
  REQUIRE(yPlane.height() == unit.getHeight());
  REQUIRE(yPlane.size() == unit.getWidth() * unit.getHeight());
  for (const auto pixel : yPlane) {
    REQUIRE(pixel == expectedPixelValue);
  }
}

void checkChrominancePlaneContents(const Frame<YUV420P8> &unit, int planeId,
                                   const int expectedPixelValue = 0) {
  const auto &uPlane = unit.getPlane(planeId);
  REQUIRE(!uPlane.empty());
  REQUIRE(uPlane.width() == unit.getWidth() / 2);
  REQUIRE(uPlane.height() == unit.getHeight() / 2);
  REQUIRE(uPlane.size() == unit.getWidth() * unit.getHeight() / 4);
  for (const auto pixel : uPlane) {
    REQUIRE(pixel == expectedPixelValue);
  }
}
void checkIfPlanesContainColor(const Frame<YUV420P8> &unit, const int expectedColor) {
  checkLuminancePlaneContents(unit, expectedColor);
  checkChrominancePlaneContents(unit, 1, expectedColor);
  checkChrominancePlaneContents(unit, 2, expectedColor);
}
} // namespace

TEST_CASE("0x0 frame through default construction") {
  const Frame<YUV420P8> unit{};

  REQUIRE(unit.empty());
  REQUIRE(unit.getHeight() == 0);
  REQUIRE(unit.getWidth() == 0);
  REQUIRE(unit.getSize() == Vec2i{0, 0});
  REQUIRE(unit.getMemorySize() == 0);
  REQUIRE(unit.getDiskSize() == 0);
  REQUIRE(unit.getBitDepth() == 8);
  REQUIRE(unit.neutralColor() == 0x80);

  REQUIRE(unit.getNumberOfPlanes() == 3);
  REQUIRE(unit.getPlanes().size() == Frame<YUV420P8>::getNumberOfPlanes());
  for (int plane_id = 0; plane_id < 3; ++plane_id) {
    REQUIRE(unit.getPlane(plane_id).empty());
  }
}

TEST_CASE("4x2 frame") {
  const Frame<YUV420P8> unit{4, 2};

  REQUIRE(!unit.empty());
  REQUIRE(unit.getHeight() == 2);
  REQUIRE(unit.getWidth() == 4);
  REQUIRE(unit.getSize() == Vec2i{4, 2});
  REQUIRE(unit.getMemorySize() == 12);
  REQUIRE(unit.getDiskSize() == 12);
  REQUIRE(unit.getBitDepth() == 8);
  REQUIRE(unit.neutralColor() == 0x80);

  REQUIRE(unit.getNumberOfPlanes() == 3);
  REQUIRE(unit.getPlanes().size() == Frame<YUV420P8>::getNumberOfPlanes());

  checkLuminancePlaneContents(unit);
  checkChrominancePlaneContents(unit, 1);
  checkChrominancePlaneContents(unit, 2);
}

TEST_CASE("4x2 frame with neutral color value") {
  Frame<YUV420P8> unit{4, 2};
  unit.fillNeutral();
  checkIfPlanesContainColor(unit, 0x80);
}

TEST_CASE("4x2 frame with color value one") {
  Frame<YUV420P8> unit{4, 2};
  unit.fillOne();

  checkIfPlanesContainColor(unit, 1);
}

TEST_CASE("4x2 frame reset to zero") {
  Frame<YUV420P8> unit{4, 2};
  unit.fillOne();
  unit.fillZero();

  checkIfPlanesContainColor(unit, 0);
}

TEST_CASE("4x2 frame fillInvalidWithNeutral") {
  Frame<YUV444P10> unit{4, 2};

  Frame<YUV420P8> depthFrame{4, 2};
  depthFrame.fillOne();
  auto &yPlane = depthFrame.getPlane(0);
  yPlane[3] = 0;

  unit.fillInvalidWithNeutral(depthFrame);

  for (const auto &plane : unit.getPlanes()) {
    for (unsigned pixel_index = 0; pixel_index < plane.size(); ++pixel_index) {
      if (pixel_index == 3) {
        REQUIRE(plane[pixel_index] == unit.neutralColor());
      } else {
        REQUIRE(plane[pixel_index] == 0);
      }
    }
  }
}

TEST_CASE("Resizing 0x0 frame to 4x2") {
  Frame<YUV420P8> unit{};
  REQUIRE(unit.getWidth() == 0);
  REQUIRE(unit.getHeight() == 0);

  unit.resize(4, 2);
  REQUIRE(unit.getWidth() == 4);
  REQUIRE(unit.getHeight() == 2);
}

TEST_CASE("Convert Yuv444P10 to YUV420P10") {
  Frame<YUV444P10> sourceFrame{2, 2};
  sourceFrame.fillNeutral();
  int offset = 0;
  for (auto &plane : sourceFrame.getPlanes()) {
    for (auto &pixel : plane) {
      pixel += offset++;
    }
  }

  const Frame<YUV420P10> destinationFrame{yuv420p(sourceFrame)};

  const auto &yPlane = destinationFrame.getPlane(0);
  REQUIRE(yPlane[0] == 512);
  REQUIRE(yPlane[1] == 513);
  REQUIRE(yPlane[2] == 514);
  REQUIRE(yPlane[3] == 515);
  const auto &uPlane = destinationFrame.getPlane(1);
  REQUIRE(uPlane[0] == 518);
  const auto &vPlane = destinationFrame.getPlane(2);
  REQUIRE(vPlane[0] == 522);
}

TEST_CASE("Convert YUV420P10 to Yuv444P10 ") {
  Frame<YUV420P10> sourceFrame{2, 2};
  sourceFrame.fillNeutral();
  int offset = 0;
  for (auto &plane : sourceFrame.getPlanes()) {
    for (auto &pixel : plane) {
      pixel += offset++;
    }
  }

  const Frame<YUV444P10> destinationFrame{yuv444p(sourceFrame)};

  const auto &yPlane = destinationFrame.getPlane(0);
  REQUIRE(yPlane[0] == 512);
  REQUIRE(yPlane[1] == 513);
  REQUIRE(yPlane[2] == 514);
  REQUIRE(yPlane[3] == 515);
  for (const auto pixel : destinationFrame.getPlane(1)) {
    REQUIRE(pixel == 516);
  }
  for (const auto pixel : destinationFrame.getPlane(2)) {
    REQUIRE(pixel == 517);
  }
}

TEST_CASE("AnyFrame to YUV444P16") {
  const AnyFrame unit{};
  const auto destinationFrame{unit.as<YUV420P8>()};
  REQUIRE(destinationFrame.getSize() == Vec2i{0, 0});
}

TEST_CASE("Expand texture with zeroes") {
  const auto result = expandTexture(Frame<YUV444P10>{4, 2});

  REQUIRE(result.height() == 2);
  REQUIRE(result.width() == 4);
  for (const auto &pixel : result) {
    REQUIRE(pixel == Vec3f{0.F, 0.F, 0.F});
  }
}

TEST_CASE("Expand texture with ones") {
  Frame<YUV444P10> frame{4, 2};
  frame.fillOne();

  const auto result = expandTexture(frame);

  for (const auto &pixel : result) {
    for (const auto color_channel : pixel) {
      REQUIRE(color_channel == Approx(0.000977517F).epsilon(1e-5F));
    }
  }
}

TEST_CASE("Expand luma with ones") {
  Frame<YUV420P10> frame{4, 2};
  frame.fillOne();

  const auto result = expandLuma(frame);

  REQUIRE(result.height() == 2);
  REQUIRE(result.width() == 4);
  for (const auto &pixel : result) {
    REQUIRE(pixel == Approx(98e-5F).epsilon(1e-2F));
  }
}

TEST_CASE("Quantize texture") {
  Mat<Vec3f> flatFrame{};
  flatFrame.resize(2, 4);
  float offset = 0.F;
  for (unsigned row_index = 0; row_index < flatFrame.height(); ++row_index) {
    for (unsigned col_index = 0; col_index < flatFrame.width(); ++col_index) {
      offset += 0.1F;
      flatFrame(row_index, col_index) = Vec3f{offset, offset + 0.02F, offset + 0.04F};
    }
  }

  const auto result = quantizeTexture(flatFrame);

  REQUIRE(result.getPlane(0)(0, 0) == 102);
  REQUIRE(result.getPlane(1)(0, 0) == 123);
  REQUIRE(result.getPlane(2)(0, 0) == 143);

  REQUIRE(result.getPlane(0)(1, 1) == 614);
  REQUIRE(result.getPlane(1)(1, 1) == 634);
  REQUIRE(result.getPlane(2)(1, 1) == 655);

  REQUIRE(result.getPlane(0)(1, 3) == 818);
  REQUIRE(result.getPlane(1)(1, 3) == 839);
  REQUIRE(result.getPlane(2)(1, 3) == 859);
}
} // namespace TMIV::Common
