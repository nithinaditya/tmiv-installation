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

#include <TMIV/Common/Common.h>
#include <TMIV/ViewingSpace/SignedDistance.h>
#include <TMIV/ViewingSpace/ViewingSpaceEvaluator.h>

namespace TMIV::ViewingSpace {
TEST_CASE("Signed distance functions") {
  SECTION("Distance sign") {
    REQUIRE(SignedDistance(-1.F).isInside());
    REQUIRE(SignedDistance(1.F).isOutside());
  }
  const MivBitstream::Cuboid cuboid{{1.F, 2.F, 3.F}, {11.F, 13.F, 17.F}};
  const MivBitstream::Spheroid spheroid{{5.F, 4.F, 3.F}, {2.F, 3.F, 4.F}};
  const Common::QuatF noRotation{0.F, 0.F, 0.F, 1.F};
  SECTION("Cuboid") {
    REQUIRE(signedDistance(cuboid, noRotation, *cuboid.center).isInside());
    REQUIRE(signedDistance(cuboid, noRotation, *cuboid.center + 0.49F * cuboid.size).isInside());
    REQUIRE(signedDistance(cuboid, euler2quat(Common::radperdeg * Common::Vec3f({180.F, 0.F, 0.F})),
                           *cuboid.center - 0.49F * cuboid.size)
                .isInside());
    REQUIRE(signedDistance(cuboid, noRotation, *cuboid.center + 0.51F * cuboid.size).isOutside());
    REQUIRE(signedDistance(cuboid,
                           euler2quat(Common::radperdeg * Common::Vec3f({180.F, 180.F, 180.F})),
                           *cuboid.center - 0.51F * cuboid.size)
                .isOutside());
  }
  SECTION("Spheroid") {
    REQUIRE(signedDistance(spheroid, noRotation, *spheroid.center).isInside());
    REQUIRE(
        signedDistance(spheroid, noRotation,
                       *spheroid.center + Common::Vec3f({0.49F * spheroid.radius.x(), 0.F, 0.F}))
            .isInside());
    REQUIRE(
        signedDistance(spheroid, euler2quat(Common::radperdeg * Common::Vec3f({0.F, 90.F, 0.F})),
                       *spheroid.center + Common::Vec3f({0.F, 0.99F * spheroid.radius.y(), 0.F}))
            .isInside());
    REQUIRE(
        signedDistance(spheroid, euler2quat(Common::radperdeg * Common::Vec3f({0.F, 90.F, 0.F})),
                       *spheroid.center + Common::Vec3f({0.F, 1.01F * spheroid.radius.y(), 0.F}))
            .isOutside());
    REQUIRE(signedDistance(spheroid, noRotation, *spheroid.center + 1.01F * spheroid.radius)
                .isOutside());
    REQUIRE(signedDistance(spheroid, noRotation, *spheroid.center - 1.01F * spheroid.radius)
                .isOutside());
  }
  SECTION("Halfspace") {
    const MivBitstream::Halfspace hs1({{1.F, 0.F, 0.F}, 10.F});
    REQUIRE(signedDistance(hs1, noRotation, Common::Vec3f({9.F, 10.F, 1.0e6F})).isInside());
    REQUIRE(signedDistance(hs1, noRotation, Common::Vec3f({11.F, -100.F, 0.F})).isOutside());
    const MivBitstream::Halfspace hs2({{0.F, 0.707F, -0.707F}, 0.F});
    REQUIRE(signedDistance(hs1, noRotation, 0.99F * hs1.distance * hs1.normal).isInside());
    REQUIRE(signedDistance(hs2, noRotation, -0.1F * hs2.normal).isInside());
    REQUIRE(signedDistance(hs1, noRotation, 1.01F * hs1.distance * hs1.normal).isOutside());
    REQUIRE(signedDistance(hs2, noRotation, 0.1F * hs2.normal).isOutside());
  }

  SECTION("Addition, subtraction, and intersection") {
    {
      const auto point = *cuboid.center - 0.49F * cuboid.size;
      const auto sd1 = signedDistance(cuboid, noRotation, point);
      const auto sd2 = signedDistance(spheroid, noRotation, point);
      REQUIRE(sd1.isInside());
      REQUIRE(sd2.isOutside());
      REQUIRE((sd1 + sd2).isInside());
      REQUIRE((sd1 - sd2).isInside());
      REQUIRE((sd2 - sd1).isOutside());
      REQUIRE((sd1 & sd2).isOutside());
      REQUIRE((sd2 & sd1).isOutside());
    }
    {
      const auto point = *spheroid.center;
      const auto sd1 = signedDistance(cuboid, noRotation, point);
      const auto sd2 = signedDistance(spheroid, noRotation, point);
      REQUIRE(sd1.isInside());
      REQUIRE(sd2.isInside());
      REQUIRE((sd1 + sd2).isInside());
      REQUIRE((sd1 - sd2).isOutside());
      REQUIRE((sd2 - sd1).isOutside());
      REQUIRE((sd1 & sd2).isInside());
      REQUIRE((sd2 & sd1).isInside());
    }
  }
}

TEST_CASE("Viewing space evaluation") {
  constexpr auto computeInclusion = &ViewingSpaceEvaluator::computeInclusion;
  using MivBitstream::ElementaryShape;
  using MivBitstream::ElementaryShapeOperation;
  using MivBitstream::PrimitiveShape;

  const PrimitiveShape cuboid = {
      MivBitstream::Cuboid{{-1.F, -1.F, -1.F}, {5.F, 5.F, 5.F}}, 1.F, {}, {}};
  const PrimitiveShape spheroid = {
      MivBitstream::Spheroid{{1.F, 1.F, 1.F}, {5.F, 5.F, 5.F}},
      1.F,
      {},
      PrimitiveShape::ViewingDirectionConstraint{
          30.F, euler2quat(Common::radperdeg * Common::Vec3f{90.F, -30.F, 0.F}), 90.F, 65.F}};
  SECTION("Guard band") {
    const MivBitstream::ViewingSpace vs1 = {
        {{ElementaryShapeOperation::add, ElementaryShape{{cuboid}}}}};
    const auto vpi = ViewingParams{{0.F, 0.F, 0.F},
                                   euler2quat(Common::radperdeg * Common::Vec3f{0.F, 0.F, 0.F})};
    const float inside = computeInclusion(vs1, vpi);
    const auto vp1 = ViewingParams{{1.F, 0.F, 0.F},
                                   euler2quat(Common::radperdeg * Common::Vec3f{0.F, 0.F, 0.F})};
    const float guardband1 = computeInclusion(vs1, vp1);
    const auto vp2 = ViewingParams{{1.F, 0.F, 0.F},
                                   euler2quat(Common::radperdeg * Common::Vec3f{180.F, 90.F, 0.F})};
    const float guardband2 = computeInclusion(vs1, vp2);
    const auto vpo = ViewingParams{
        {2.F, 0.F, 0.F}, euler2quat(Common::radperdeg * Common::Vec3f{-90.F, -30.F, 0.F})};
    const float outside = computeInclusion(vs1, vpo);
    REQUIRE(inside == 1.F);
    REQUIRE(guardband1 > 0.F);
    REQUIRE(guardband1 < 1.F);
    REQUIRE(guardband2 == guardband1);
    REQUIRE(outside == 0.F);
  }
  SECTION("Direction guard band") {
    const MivBitstream::ViewingSpace vs2 = {
        {{ElementaryShapeOperation::add, ElementaryShape{{spheroid}}}}};
    const Common::Vec3f pos = {1.F, 1.F, 1.F};
    const auto vpOutside =
        ViewingParams{{pos}, euler2quat(Common::radperdeg * Common::Vec3f{40.F, 0.F, 0.F})};
    const float outside = computeInclusion(vs2, vpOutside);
    const auto vpInside =
        ViewingParams{{pos}, euler2quat(Common::radperdeg * Common::Vec3f{90.F, -30.F, 0.F})};
    const float inside = computeInclusion(vs2, vpInside);
    const auto vpGuardband =
        ViewingParams{{pos}, euler2quat(Common::radperdeg * Common::Vec3f{50.F, -50.F, 0.F})};
    const float guardband = computeInclusion(vs2, vpGuardband);
    REQUIRE(inside == 1.F);
    REQUIRE(guardband > 0.F);
    REQUIRE(guardband < 1.F);
    REQUIRE(outside == 0.F);
  }
  SECTION("Additive elementary shape") {
    const MivBitstream::ViewingSpace vs3 = {
        {{ElementaryShapeOperation::add, ElementaryShape{{cuboid}}},
         {ElementaryShapeOperation::add, ElementaryShape{{spheroid}}}}};
    const ViewingParams poseInside = {
        {0.F, 0.F, 0.F}, euler2quat(Common::radperdeg * Common::Vec3f{90.F, -30.F, 0.F})};
    const ViewingParams poseGuard = {
        {0.F, 0.F, 0.F}, euler2quat(Common::radperdeg * Common::Vec3f{160.F, -60.F, 0.F})};
    REQUIRE(computeInclusion(vs3, poseInside) == 1.F);
    REQUIRE(Common::inRange(computeInclusion(vs3, poseGuard), 0.1F, 0.9F));
  }
  SECTION("Subtractive elementary shape") {
    const MivBitstream::ViewingSpace vs4 = {
        {{ElementaryShapeOperation::add, ElementaryShape{{cuboid}}},
         {ElementaryShapeOperation::subtract, ElementaryShape{{spheroid}}}}};
    const Common::Vec3f posOutside = {0.F, 0.F, 0.F};
    REQUIRE(computeInclusion(
                vs4, {posOutside,
                      euler2quat(Common::radperdeg * Common::Vec3f{90.F, -30.F, 0.F})}) == 0.F);
  }
  SECTION("Direction constraint blending") {
    const PrimitiveShape cuboid2 = {
        MivBitstream::Cuboid{{-1.F, -1.F, -1.F}, {5.F, 5.F, 5.F}},
        1.F,
        {},
        PrimitiveShape::ViewingDirectionConstraint{
            30.F, euler2quat(Common::radperdeg * Common::Vec3f{260.F, 0.F, 0.F}), 300.F, 160.F}};
    const MivBitstream::ViewingSpace vs5 = {
        {{ElementaryShapeOperation::add, ElementaryShape{{cuboid2}}},
         {ElementaryShapeOperation::add, ElementaryShape{{spheroid}}}}};
    const ViewingParams poseInside = {
        {0.F, 0.F, 0.F}, euler2quat(Common::radperdeg * Common::Vec3f{180.F, -30.F, 0.F})};
    REQUIRE(computeInclusion(vs5, poseInside) == 1.F);
  }
}
} // namespace TMIV::ViewingSpace
