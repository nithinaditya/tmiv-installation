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

#include <TMIV/Common/Filter.h>

namespace {
auto exampleInput(int rows, int cols) {
  auto in = TMIV::Common::Mat<uint16_t>{{static_cast<size_t>(rows), static_cast<size_t>(cols)}};

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      in(i, j) = (i * j) ^ 13;
    }
  }

  return in;
}

auto sumPixelsInRectangle(const TMIV::Common::Mat<uint16_t> &in, const TMIV::Common::Vec2i &p_1,
                          const TMIV::Common::Vec2i &p_2) {
  auto sum = intmax_t{};

  for (int i = p_1.y(); i < p_2.y(); ++i) {
    for (int j = p_1.x(); j < p_2.x(); ++j) {
      sum += in(i, j);
    }
  }
  return sum;
}

auto countPixelsInRectangle(const TMIV::Common::Vec2i &p_1, const TMIV::Common::Vec2i &p_2) {
  auto count = intmax_t{};

  for (int i = p_1.y(); i < p_2.y(); ++i) {
    for (int j = p_1.x(); j < p_2.x(); ++j) {
      ++count;
    }
  }
  return count;
}

auto averagePixels(const TMIV::Common::Mat<uint16_t> &in, int i, int j, int k) {
  using TMIV::Common::Vec2i;
  const auto rows = static_cast<int>(in.height());
  const auto cols = static_cast<int>(in.width());
  const auto p_1 = Vec2i{std::clamp(j - k, 0, cols), std::clamp(i - k, 0, rows)};
  const auto p_2 = Vec2i{std::clamp(j + k + 1, 0, cols), std::clamp(i + k + 1, 0, rows)};

  const auto sum = sumPixelsInRectangle(in, p_1, p_2);
  const auto count = countPixelsInRectangle(p_1, p_2);
  assert(0 <= sum);
  return (sum + count / 2) / count;
}
} // namespace

TEST_CASE("integralImage") {
  const auto rows = GENERATE(0, 1, 5);
  const auto cols = GENERATE(0, 1, 4);
  const auto in = exampleInput(rows, cols);

  using SumType = int32_t;
  const auto out = TMIV::Common::integralImage<SumType>(in);

  REQUIRE(out.width() == cols + size_t{1});
  REQUIRE(out.height() == rows + size_t{1});

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      REQUIRE(out(i, j) == sumPixelsInRectangle(in, {}, {j, i}));
    }
  }
}

TEST_CASE("sumRect") {
  const auto rows = 5;
  const auto cols = 4;
  const auto in = exampleInput(rows, cols);
  const auto ii = TMIV::Common::integralImage<int32_t>(in);

  SECTION("In bounds") {
    const auto i1 = GENERATE(0, 1, 2, 3, 4, 5);
    const auto i2 = GENERATE(0, 1, 2, 3, 4, 5);

    if (i1 <= i2 && i2 <= rows) {
      const auto j1 = GENERATE(0, 1, 2, 3, 4);
      const auto j2 = GENERATE(0, 1, 2, 3, 4);

      if (j1 <= j2 && j2 <= cols) {
        const auto ji_1 = TMIV::Common::Vec2i{j1, i1};
        const auto ji_2 = TMIV::Common::Vec2i{j2, i2};
        REQUIRE(sumRect(ii, ji_1, ji_2) == sumPixelsInRectangle(in, ji_1, ji_2));
      }
    }
  }

  SECTION("Out of bounds rectangle is clipped") {
    REQUIRE(TMIV::Common::sumRect(ii, {-5, -4}, {-2, 2}) == 0);
    REQUIRE(TMIV::Common::sumRect(ii, {-5, -4}, {10, 30}) == sumRect(ii, {}, {cols, rows}));
  }
}

TEST_CASE("boxBlur") {
  const auto rows = 3;
  const auto cols = 7;
  const auto in = exampleInput(rows, cols);

  const auto k = GENERATE(0, 1, 2);
  const auto out = TMIV::Common::boxBlur<int32_t>(in, k);

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      REQUIRE(out(i, j) == averagePixels(in, i, j, k));
    }
  }
}
