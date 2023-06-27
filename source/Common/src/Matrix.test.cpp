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

#include <algorithm>
//#define CATCH_CONFIG_ENABLE_BENCHMARKING // Uncomment me to run benchmarks
// Alternatively, to control benchmark running from cmake, use
// target_compile_definitions(${TEST_CREATOR_TARGET} PUBLIC CATCH_CONFIG_ENABLE_BENCHMARKING)
// in test_creation.cmake, potentially in an if clause controlled by a cmake option
#include <catch2/catch.hpp>

#include <TMIV/Common/Matrix.h>
namespace TMIV::Common {
TEST_CASE("Stack Matrix 2x2 default constructor") {
  const Mat2x2f unit{};

  REQUIRE(unit.height() == 2);
  REQUIRE(unit.width() == 2);
  REQUIRE(unit.m() == 2);
  REQUIRE(unit.n() == 2);

  for (Mat2x2f::size_type index = 0; index < 2; ++index) {
    std::for_each(unit.row_begin(index), unit.row_end(index),
                  [](const auto element) { REQUIRE(element == Approx(0.0F)); });
  }
  REQUIRE(!unit.isRow());
  REQUIRE(!unit.isColumn());
  REQUIRE(!unit.isPositive());
}

TEST_CASE("Positive, symmetric 3x3 matrix") {
  const Mat3x3f unit{1.F, 2.F, 3.F, 2.F, 5.F, 0.1F, 3.F, 0.1F, 4.F};
  REQUIRE(unit.isPositive());
}

TEST_CASE("Asymmetric, non-positive 3x3 matrix") {
  const Mat3x3f unit{1.F, 2.F, 3.F, 2.F, 5.F, -0.1F, 3.F, 0.1F, 4.F};
  REQUIRE(!unit.isPositive());
}

TEST_CASE("Non-quadratic matrix is not symmetric") {
  const stack::Matrix<int, 2, 3> unit{0, 1, 2, 3, 4, 5};
  REQUIRE(!unit.isPositive());
}

TEST_CASE("Stack Matrix 3x3 with custom elements") {
  Mat3x3f unit{};
  for (Mat3x3f::size_type index = 0; index < 3; ++index) {
    std::iota(unit.row_begin(index), unit.row_end(index), static_cast<float>(3 * index));
  }

  REQUIRE(unit(0, 0) == Approx(0.0F));
  REQUIRE(unit(0, 1) == Approx(1.0F));
  REQUIRE(unit(0, 2) == Approx(2.0F));

  REQUIRE(unit(1, 0) == Approx(3.0F));
  REQUIRE(unit(1, 1) == Approx(4.0F));
  REQUIRE(unit(1, 2) == Approx(5.0F));

  REQUIRE(unit(2, 0) == Approx(6.0F));
  REQUIRE(unit(2, 1) == Approx(7.0F));
  REQUIRE(unit(2, 2) == Approx(8.0F));
}

TEST_CASE("Transpose 2x2 Heap Matrix") {
  Mat<int> input_matrix{};
  input_matrix.resize(2, 2);
  input_matrix(0, 0) = 0;
  input_matrix(0, 1) = 1;
  input_matrix(1, 0) = 2;
  input_matrix(1, 1) = 3;

  const auto result = transpose(input_matrix);

  REQUIRE(result(0, 0) == 0);
  REQUIRE(result(0, 1) == 2);
  REQUIRE(result(1, 0) == 1);
  REQUIRE(result(1, 1) == 3);
}

TEST_CASE("Transpose 2x1 Heap Matrix") {
  Mat<int> input_matrix{};
  input_matrix.resize(2, 1);
  input_matrix(0, 0) = 0;
  input_matrix(1, 0) = 1;
  REQUIRE(input_matrix.isColumn());

  const auto result = transpose(input_matrix);
  REQUIRE(result.isRow());

  REQUIRE(result(0, 0) == 0);
  REQUIRE(result(0, 1) == 1);
}

TEST_CASE("Matrix arithmetics") {
  const Mat2x2f matrixA = {1.F, 2.F, 3.F, 4.F};
  const Mat2x2f matrixB = {-1.F, 3.F, 1.F, 2.F};

  REQUIRE(matrixA * 1.F == matrixA);
  REQUIRE(1.F * matrixB == matrixB);
  REQUIRE(2.F * matrixA == Mat2x2f{2.F, 4.F, 6.F, 8.F});
  REQUIRE(matrixA / 2.F == Mat2x2f{0.5F, 1.F, 1.5F, 2.F});
  REQUIRE(matrixA + matrixB == Mat2x2f{0.F, 5.F, 4.F, 6.F});
  REQUIRE(matrixA - matrixB == Mat2x2f{2.F, -1.F, 2.F, 2.F});
}

#ifdef CATCH_CONFIG_ENABLE_BENCHMARKING
TEST_CASE("Benchmark: transpose") {
  stack::Matrix<int, 200, 200> asymmetric_matrix{};
  asymmetric_matrix(100, 150) = 1;
  BENCHMARK("200x200 asymmetric stack matrix") {
    const auto result = transpose(asymmetric_matrix);
    REQUIRE(result(150, 100) == 1);
  };

  stack::Matrix<int, 200, 200> symmetric_matrix{};
  BENCHMARK("200x200 symmetric stack matrix") {
    const auto result = transpose(symmetric_matrix);
    REQUIRE(result(150, 100) == 0);
  };
}
#endif

} // namespace TMIV::Common
