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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

using Catch::Matchers::Contains;

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Common.h>
#include <TMIV/Common/Half.h>
#include <TMIV/Common/LinAlg.h>

#include <iostream>
#include <sstream>

using namespace std::string_literals;
using namespace std::string_view_literals;

namespace TMIV::Common {
TEST_CASE("Array, Vector, Matrix, LinAlg") {
  const float EPS = 1e-5F;

  heap::Array<3, float> A({3, 5, 10});
  Mat3x3f m1 = {1.F, 2.F, 3.F, 4.F, 5.F, 6.F, 7.F, 8.F, -9.F};
  Mat<float> m2({4, 3}, {1.F, 2.F, 3.F, 4.F, 5.F, 6.F, 7.F, 8.F, 9.F, 10.F, 11.F, 12.F});
  Vec3f v1({1.F, 2.F, 3.F});
  Vec3f v2({3.F, -1.F, 6.F});

  SECTION("Array")
  REQUIRE(A.size() == 150);

  SECTION("Vector norm")
  REQUIRE(std::fabs(norm(v1) - 3.7416573F) < EPS);

  SECTION("Unit vector")
  REQUIRE(std::fabs(norm(unit(v1)) - 1.F) < EPS);

  SECTION("Vector dot product")
  REQUIRE(std::fabs(dot(v1, v1) - 14.F) < EPS);

  SECTION("Vector cross product")
  REQUIRE(norm_inf(cross(v1, v2) - Vec3f({15.F, 3.F, -7.F})) < EPS);

  SECTION("Vector-scalar product")
  REQUIRE(Vec3f({1.F, 2.F, 4.F}) * 2.F == Vec3f({2.F, 4.F, 8.F}));
  REQUIRE(2.F * Vec3f({1.F, 2.F, 4.F}) == Vec3f({2.F, 4.F, 8.F}));

  SECTION("Vector-scalar min/max")
  REQUIRE(min(v1, 2.F) == Vec3f({1.F, 2.F, 2.F}));
  REQUIRE(min(v1, 2.F) == min(2.F, v1));
  REQUIRE(max(v1, 2.F) == Vec3f({2.F, 2.F, 3.F}));
  REQUIRE(max(v1, 2.F) == max(2.F, v1));

  SECTION("Matrix determinant")
  REQUIRE(std::fabs(det(m1) - 54.F) < EPS);

  SECTION("Matrix / Matrix product")
  REQUIRE(norm_inf((m2 * m1) - Mat<float>({4, 3}, {30.F, 36.F, -12.F, 66.F, 81.F, -12.F, 102.F,
                                                   126.F, -12.F, 138.F, 171.F, -12.F})) < EPS);

  SECTION("Matrix inverse")
  REQUIRE(norm_inf((m1 * inv(m1)) - Mat3x3f::eye()) < EPS);

  SECTION("Vector-vector min/max")
  REQUIRE(min(v1, v2) == Vec3f({1.F, -1.F, 3.F}));
  REQUIRE(max(v1, v2) == Vec3f({3.F, 2.F, 6.F}));
  REQUIRE(min(v1, v2) == min(v2, v1));
  REQUIRE(max(v1, v2) == max(v2, v1));

  SECTION("Vector abs")
  REQUIRE(abs(v2) == Vec3f({3.F, 1.F, 6.F}));

  SECTION("Matrix / Vector product")
  REQUIRE(norm_inf((m1 * v1) - Vec3f({14.F, 32.F, -4.F})) < EPS);

  SECTION("Linear system (right)")
  REQUIRE(norm_inf(mrdivide(m2, m1) - Mat<float>({4, 3}, {1.F, 0.F, 0.F, 0.F, 1.F, 0.F, -1.F, 2.F,
                                                          0.F, -2.F, 3.F, 0.F})) < EPS);

  SECTION("Linear system (left)")
  REQUIRE(norm_inf(mldivide(m1, v1) - Vec3f({-1.F, 2.F, 0.F}) / 3.F) < EPS);
}

namespace {
class Fake : public Application {
public:
  explicit Fake(char const *tool, std::vector<const char *> argv, Options options)
      : Application{tool, std::move(argv), std::move(options)} {}

  using Application::json;
  using Application::optionValues;

  void run() override {}
};
} // namespace

TEST_CASE("Parsing the command-line", "[Application]") {
  SECTION("Empty command-line returns usage instructions") {
    try {
      Fake app{"Fake", {"command"}, {}};
      REQUIRE(false);
    } catch (std::runtime_error &e) {
      REQUIRE_THAT(e.what(), Contains("Usage"));
      REQUIRE_THAT(e.what(), Contains("Fake"));
    }
  }

  SECTION("Specifying parameters with -p KEY VALUE") {
    Fake app{"Fake", {"command", "-p", "Color", "green", "-p", "Shape", "circular"}, {}};
    REQUIRE(app.json().require("Color").as<std::string>() == "green"s);
    REQUIRE(app.json().require("Shape").as<std::string>() == "circular"s);
  }

  SECTION("Right has preference over left") {
    Fake app{"Fake", {"command", "-p", "Color", "5", "-p", "Color", "7"}, {}};
    REQUIRE(app.json().require("Color").as<int>() == 7);
  }

  SECTION("Structured values") {
    Fake app{"Fake", {"command", "-p", "Size", "[20, 30]", "-p", "Params", "{ \"a\": 3 }"}, {}};
    CHECK(app.json().require("Size").asVec<int, 2>() == Vec2i{20, 30});
    CHECK(app.json().require("Params").require("a").as<int>() == 3);
  }

  SECTION("A single-occurance application-specific option needs to be specified exactly once") {
    const auto opts = Fake::Options{{"-s", "single", false}};
    REQUIRE_THROWS(Fake{"Fake", {"fake", "-p", "a", "b"}, opts});
    REQUIRE_NOTHROW(Fake{"Fake", {"fake", "-p", "a", "b", "-s", "1"}, opts});
    REQUIRE_THROWS(Fake{"Fake", {"fake", "-p", "a", "b", "-s", "1", "-s", "2"}, opts});
  }

  SECTION("A multiple-occurrence application-specific option may be specified zero or more times") {
    const auto opts = Fake::Options{{"-m", "multiple", true}};
    REQUIRE_NOTHROW(Fake{"Fake", {"fake", "-p", "a", "b"}, opts});
    REQUIRE_NOTHROW(Fake{"Fake", {"fake", "-p", "a", "b", "-m", "1"}, opts});
    REQUIRE_NOTHROW(Fake{"Fake", {"fake", "-p", "a", "b", "-m", "1", "-m", "2"}, opts});
  }

  SECTION("Application-specific option sets are available to the application") {
    const auto app = Fake{"Fake",
                          {"fake", "-p", "a", "b", "-s", "single", "-m", "first", "-m", "second"},
                          {{"-s", "single", false}, {"-m", "multiple", true}}};
    REQUIRE(app.optionValues("-s") == std::vector{"single"s});
    REQUIRE(app.optionValues("-m") == std::vector{"first"s, "second"s});
  }
}

TEST_CASE("Assignment of std::fixed size matrix N x 1 to fixed size vector") {
  stack::Matrix<double, 6, 1> A;
  std::fill(std::begin(A), std::end(A), 1.);
  stack::Vector<double, 6> b;
  b = A;
  REQUIRE(b[5] == 1);
}

TEST_CASE("Half") {
  // Examples taken from https://en.wikipedia.org/wiki/Half-precision_floating-point_format

  SECTION("Decode") {
    REQUIRE_THROWS_AS(Half::decode(0x0001), HalfError); // the WD excludes subnormals
    REQUIRE_THROWS_AS(Half::decode(0x03FF), HalfError); // the WD excludes subnormals
    REQUIRE_NOTHROW(Half::decode(0x0400));              // smallest positive normal number
    REQUIRE_NOTHROW(Half::decode(0x7BFF));              // largest normal number
    REQUIRE_NOTHROW(Half::decode(0x3BFF));              // largest number less than one
    REQUIRE_NOTHROW(Half::decode(0x3C00));              // one
    REQUIRE_NOTHROW(Half::decode(0x3C01));              // smallest number larger then one
    REQUIRE_NOTHROW(Half::decode(0x3555));              // one third (rounded down)
    REQUIRE_NOTHROW(Half::decode(0xC000));              // minus two
    REQUIRE_NOTHROW(Half::decode(0x0000));              // positive zero
    REQUIRE_NOTHROW(Half::decode(0x8000));              // negative zero
    REQUIRE_THROWS_AS(Half::decode(0x7C00), HalfError); // the WD excludes infinitity
    REQUIRE_THROWS_AS(Half::decode(0xFC00), HalfError); // the WD excludes -infinitity
    REQUIRE_THROWS_AS(Half::decode(0x7C01), HalfError); // the WD excludes NaN values
  }

  SECTION("Encode") {
    for (uint16_t code :
         {0x0400u, 0x7BFFu, 0x3BFFu, 0x3C00u, 0x3C01u, 0x3555u, 0xC000u, 0x0000u, 0x8000u}) {
      REQUIRE(Half::decode(code).encode() == code);
    }
  }

  SECTION("Implicit conversion to float (lossless)") {
    REQUIRE(Half::decode(0x0400) == 0x1.p-14F);   // smallest positive normal number
    REQUIRE(Half::decode(0x7BFF) == 65504.F);     // largest normal number
    REQUIRE(Half::decode(0x3BFF) == 0x1.FFCp-1F); // largest number less than one
    REQUIRE(Half::decode(0x3C00) == 1.F);         // one
    REQUIRE(Half::decode(0x3C01) == 0x1.004p0F);  // smallest number larger then one
    REQUIRE(Half::decode(0x3555) == 0x1.554p-2F); // one third (rounded down)
    REQUIRE(Half::decode(0xC000) == -2.F);        // minus two
    REQUIRE(Half::decode(0x0000) == 0.F);         // positive zero
    REQUIRE(Half::decode(0x8000) == -0.F);        // negative zero
  }

  SECTION("Explicit conversion from float (lossy)") {
    REQUIRE_THROWS_AS(Half(std::nextafter(65504.F, 1e6F)), HalfError);
    REQUIRE_THROWS_AS(Half(std::nextafter(-65504.F, -1e6F)), HalfError);
    REQUIRE_THROWS_AS(Half(NAN), HalfError);
    REQUIRE_THROWS_AS(Half(INFINITY), HalfError);
    REQUIRE(Half(0x1.p-14F).encode() == 0x0400);   // smallest positive normal number
    REQUIRE(Half(65504.F).encode() == 0x7BFF);     // largest normal number
    REQUIRE(Half(0x1.FFCp-1F).encode() == 0x3BFF); // largest number less than one
    REQUIRE(Half(1.F).encode() == 0x3C00);         // one
    REQUIRE(Half(0x1.004p0F).encode() == 0x3C01);  // smallest number larger then one
    REQUIRE(Half(0x1.554p-2F).encode() == 0x3555); // one third (rounded down)
    REQUIRE(Half(-2.F).encode() == 0xC000);        // minus two
    REQUIRE(Half(0.F).encode() == 0x0000);         // positive zero
    REQUIRE(Half(-0.F).encode() == 0x8000);        // negative zero
  }
}

TEST_CASE("maxlevel", "[quantize_and_expand]") {
  REQUIRE(maxLevel(8U) == 255U);
  REQUIRE(maxLevel(10U) == 1023U);
  REQUIRE(maxLevel(16U) == 65535U);
}

TEST_CASE("expandValue", "[quantize_and_expand]") {
  REQUIRE(expandValue(0, 10) == 0.F);
  REQUIRE(expandValue(128, 8) == 128.F / 255.F);
  REQUIRE(expandValue(1023, 10) == 1.F);
  REQUIRE(expandValue(40000, 16) == 40000.F / 65535.F);
}

TEST_CASE("quantizeValue", "[quantize_and_expand]") {
  REQUIRE(quantizeValue(NAN, 10) == 0U);
  REQUIRE(quantizeValue(INFINITY, 10) == 1023U);
  REQUIRE(quantizeValue(1e20F, 10) == 1023U);
}
} // namespace TMIV::Common
