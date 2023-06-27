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

#include <TMIV/Common/Quaternion.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/LinAlg.h>

#include <cmath>

namespace TMIV::Common {
TEST_CASE("Quanternion<T>", "[quaternion]") {
  static_assert(std::is_same_v<QuatF, Quaternion<float>>);
  static_assert(std::is_same_v<QuatD, Quaternion<double>>);
  static_assert(std::is_same_v<QuatF, Vec4f>);
  static_assert(std::is_same_v<QuatD, Vec4d>);

  const auto p = QuatF{1.F, -2.F, 3.F, -4.F};     // some quaternion
  const auto q = QuatF{1.F, 3.F, 4.F, 7.F};       // another quaternion
  const auto r = QuatF{-0.2F, 0.4F, 0.4F, -0.8F}; // some versor
  const auto u = QuatF{0.F, 0.F, 0.F, 1.F};       // zero rotation versor

  SECTION("Quaternion convention (x, y, z, w)") {
    // p = w + ix + jz + kz
    REQUIRE(p.x() == 1.F);
    REQUIRE(p.y() == -2.F);
    REQUIRE(p.z() == 3.F);
    REQUIRE(p.w() == -4.F);
  }

  SECTION("Quaternion norm") {
    REQUIRE(norm(u) == 1.F);
    REQUIRE(norm(r) == 1.F);
  }

  SECTION("Unit quaternion test") {
    REQUIRE(!normalized(p));
    REQUIRE(!normalized(q));
    REQUIRE(normalized(u));
    REQUIRE(normalized(r));
  }

  SECTION("Quaternion multiplication") {
    REQUIRE(p * conj(p) == QuatF{0.F, 0.F, 0.F, norm2(p)});
    REQUIRE(q * conj(q) == QuatF{0.F, 0.F, 0.F, norm2(q)});
    REQUIRE(p * q == QuatF{-14.F, -27.F, 10.F, -35.F});
    REQUIRE(q * p == QuatF{20.F, -25.F, 0.F, -35.F});
  }

  SECTION("Vector quaternion") { REQUIRE(quat(Vec3d{1., 2., 3.}) == QuatD{1., 2., 3., 0.}); }

  const auto eps = 1e-5F;

  SECTION("Rotate a vector with a versor") {
    const auto v = Vec3f{6.F, 3.F, 2.F};
    REQUIRE(rotate(v, u) == Vec3f{6.F, 3.F, 2.F});
    REQUIRE(norm(rotate(v, r) - Vec3f{2.F, -3.F, 6.F}) < eps);
    REQUIRE(norm(rotate(v, conj(r)) - Vec3f{0.72F, 5.96F, -3.6F}) < eps);
  }

  SECTION("Convert Euler angles to quaternion") {
    // Below examples are derived from the equation at
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion

    REQUIRE(norm(euler2quat(Vec3d{2., 0., 0.}) - QuatD{0., 0., std::sin(1.), std::cos(1.)}) < eps);
    REQUIRE(norm(euler2quat(Vec3d{0., 2., 0.}) - QuatD{0., std::sin(1.), 0., std::cos(1.)}) < eps);
    REQUIRE(norm(euler2quat(Vec3d{0., 0., 2.}) - QuatD{std::sin(1.), 0., 0., std::cos(1.)}) < eps);
    REQUIRE(norm(euler2quat(Vec3d{quarterCycle, quarterCycle, quarterCycle}) -
                 QuatD{0., std::sqrt(0.5), 0., std::sqrt(0.5)}) < eps);
    REQUIRE(norm(euler2quat(Vec3d{quarterCycle, -quarterCycle, quarterCycle}) -
                 QuatD{std::sqrt(0.5), 0., std::sqrt(0.5), 0.}) < eps);
  }

  SECTION("Convert Euler angles to quaternion (2)") {
    const auto yaw_rad = -0.4764713951;
    const auto pitch_rad = 0.0344346480;
    const auto roll_rad = 0.0204419943;
    const auto quat = euler2quat(Vec3d{yaw_rad, pitch_rad, roll_rad});

    REQUIRE(quat.x() == Approx(0.0139933465964437));
    REQUIRE(quat.y() == Approx(0.0143176961628196));
    REQUIRE(quat.z() == Approx(-0.2361122181516230));
    REQUIRE(quat.w() == Approx(0.9715195367398130));
  }

  SECTION("Convert quaternion to Euler angles") {
    const auto euler = quat2euler(
        QuatD{0.0139933465964437, 0.0143176961628196, -0.2361122181516230, 0.9715195367398130});

    CHECK(euler.x() == Approx(-0.4764713951)); // yaw [rad]
    CHECK(euler.y() == Approx(0.0344346480));  // pitch [rad]
    CHECK(euler.z() == Approx(0.0204419943));  // roll [rad]

    const auto euler2 = quat2euler(QuatD{-0.5, 0.5, 0.5, 0.5});

    CHECK(euler2.x() == Approx(0));                    // yaw [rad]
    CHECK(euler2.y() == Approx(1.570796326794896558)); // pitch [rad]
    CHECK(euler2.z() == Approx(0));                    // roll [rad]
  }

  SECTION("Convert quaternion to rotation matrix") {
    REQUIRE(rotationMatrix(u) == Mat3x3f::eye());

    const auto R = rotationMatrix(r);

    SECTION("Check matrix cells using another method") {
      for (int i = 0; i < 3; ++i) {
        const auto p = Vec3d{static_cast<double>(i == 0), static_cast<double>(i == 1),
                             static_cast<double>(i == 2)};
        const auto rotate_p_by_r = rotate(p, conj(r));
        for (int j = 0; j < 3; ++j) {
          REQUIRE(R(i, j) == Approx(rotate_p_by_r[j]));
        }
      }
    }

    SECTION("Trial rotation") {
      const auto v = Vec3f{12.F, -7.F, 9.F};

      REQUIRE(norm(R * v - rotate(v, r)) < eps);
    }
  }
}
} // namespace TMIV::Common
