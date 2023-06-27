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

#include <TMIV/Common/Common.h>
#include <TMIV/MivBitstream/ViewParamsList.h>
#include <TMIV/MivBitstream/ViewingSpace.h>

namespace TMIV::MivBitstream {
namespace examples {
inline auto deg2HalfQuat(const float yawDeg, const float pitchDeg, const float rollDeg = 0.F)
    -> Common::QuatF {
  Common::QuatF q = euler2quat(Common::radperdeg * Common::Vec3f{yawDeg, pitchDeg, rollDeg});
  q.x() = Common::Half(q.x());
  q.y() = Common::Half(q.y());
  q.z() = Common::Half(q.z());
  q.w() = 0.F;
  q.w() = Common::Half(std::sqrt(1.F - norm2(q)));
  return q;
}

const std::vector<int> myInferringViews{1};

const auto viewingSpace = std::array{
    ViewingSpace{
        {{ElementaryShapeOperation::add, ElementaryShape{{PrimitiveShape{
                                                             Cuboid{{}, {}}, // primitive
                                                             {},             // guard band size
                                                             {},             // orientation
                                                             {} // viewing direction constraint
                                                         }},
                                                         {}}}}},
    ViewingSpace{{{ElementaryShapeOperation::subtract,
                   ElementaryShape{{PrimitiveShape{Spheroid{{}, {}}, {}, {}, {}}},
                                   PrimitiveShapeOperation::interpolate}},
                  {ElementaryShapeOperation::add,
                   ElementaryShape{{PrimitiveShape{Halfspace{{}, {}}, {}, {}, {}}}}}}},
    ViewingSpace{{{ElementaryShapeOperation::add, ElementaryShape{{PrimitiveShape{
                                                      Cuboid{{}, {}},
                                                      1.F, // guard band size
                                                      deg2HalfQuat(30.F, 60.F, 90.F), // orientation
                                                      {} // viewing direction constraint
                                                  }}}}}},
    ViewingSpace{
        {{ElementaryShapeOperation::add,
          ElementaryShape{{PrimitiveShape{Cuboid{{}, {}},
                                          {},
                                          {},
                                          PrimitiveShape::ViewingDirectionConstraint{
                                              {},
                                              deg2HalfQuat(90.F, 45.F), // viewing_direction
                                              30.F,                     // yaw_range,
                                              60.F                      // pitch_range
                                          }}}}}}},
    ViewingSpace{
        {{ElementaryShapeOperation::intersect,
          ElementaryShape{{PrimitiveShape{
              Cuboid{{}, {}}, 1.F, euler2quat(Common::radperdeg *Common::Vec3f{30.F, 45.F, 60.F}),
              PrimitiveShape::ViewingDirectionConstraint{
                  15.F,                          // guard_band_direction_size
                  deg2HalfQuat(90.F, 45.F, 0.F), // viewing_direction
                  30.F,                          // yaw_range,
                  60.F                           // pitch_range
              }}}}},
         {ElementaryShapeOperation::subtract,
          ElementaryShape{{PrimitiveShape{Cuboid{{-1.F, 0.F, 1.F}, {1.F, 2.F, 3.F}}, {}, {}, {}},
                           PrimitiveShape{Spheroid{{-2.F, 2.F, 2.F}, {3.F, 2.F, 1.F}}, {}, {}, {}},
                           PrimitiveShape{Halfspace{{3.F, 3.F, 3.F}, -1.F}, {}, {}, {}}},
                          PrimitiveShapeOperation::interpolate}}}},

    ViewingSpace{
        {{ElementaryShapeOperation::add,
          ElementaryShape{{PrimitiveShape{Spheroid{{1.F, 2.F, 3.F}, {4.F, 5.F, 6.F}}, // primitive
                                          {}, // guard band size
                                          {}, // orientation
                                          {}}},
                          {}}}}},

    ViewingSpace{{{ElementaryShapeOperation::add,
                   ElementaryShape{
                       {PrimitiveShape{Spheroid{{1.F, 2.F, 3.F}, {4.F, 5.F, 6.F}}, // primitive
                                       15.F, // guard band size
                                       euler2quat(Common::radperdeg *Common::Vec3f{
                                           30.F, 45.F, 60.F}), // orientation
                                       //{}}},
                                       PrimitiveShape::ViewingDirectionConstraint{
                                           // viewing direction constraint
                                           10.F, // guard_band_direction_size
                                           deg2HalfQuat(28.08F, 1.814F, 0.F), // viewing_direction
                                           30.F,                              // yaw_range,
                                           60.F                               // pitch_range
                                       }}},
                       PrimitiveShapeOperation::interpolate}}}},

    ViewingSpace{{{ElementaryShapeOperation::add,
                   ElementaryShape{{PrimitiveShape{Spheroid{{}}, // primitive without center
                                                   {},           // guard band size
                                                   {},           // orientation
                                                   {}}},
                                   {},
                                   myInferringViews}}}}

};

const auto viewingSpaceJson = std::array{
    "{\"ElementaryShapes\":[{\"ElementaryShapeOperation\":\"add\",\"ElementaryShape\": "
    "{\"PrimitiveShapeOperation\": \"add\",\"PrimitiveShapes\": [{\"PrimitiveShapeType\": "
    "\"cuboid\",\"Center\":[0,0,0],\"Size\":[0,0,0]}]}}]}",

    "{\"ElementaryShapes\":[{\"ElementaryShapeOperation\":\"intersect\",\"ElementaryShape\":{"
    "\"PrimitiveShapeOperation\":\"add\",\"PrimitiveShapes\":[{\"PrimitiveShapeType\":\"Cuboid\","
    "\"GuardBandSize\":1.0,\"Rotation\":[30,45,60],\"ViewingDirectionConstraint\":{"
    "\"GuardBandDirectionSize\":15.0,\"YawCenter\":90.0,\"YawRange\":30.0,\"PitchCenter\":45.0,"
    "\"PitchRange\":60.0}}]}},{\"ElementaryShapeOperation\":\"subtract\",\"ElementaryShape\":{"
    "\"PrimitiveShapes\":[{\"PrimitiveShapeType\":\"cuboid\",\"Center\":[-1.0,0.0,1.0],\"Size\":[1."
    "0,2.0,3.0]},{\"PrimitiveShapeType\":\"spheroid\",\"Center\":[-2.0,2.0,2.0],\"Radius\":[3.0,2."
    "0,1.0]},{\"PrimitiveShapeType\":\"halfspace\",\"Normal\":[3.0,3.0,3.0],\"Distance\":-1.0}],"
    "\"PrimitiveShapeOperation\":\"interpolate\"}}]}",

    "{\"ElementaryShapes\":[{"
    "   \"ElementaryShapeOperation\":\"add\","
    "   \"ElementaryShape\":{"
    "       \"InferringViews\":[\"v01\"],"
    "       \"PrimitiveShapeOperation\":\"interpolate\","
    "       \"PrimitiveShapes\":[{"
    "           \"PrimitiveShapeType\":\"spheroid\","
    "           \"Radius\":[0,0,0]"
    "       }]"
    "   }"
    "}]}"};

const auto configJson = std::array{R"({"SourceCameraNames": ["v00","v01","v02"]})"};

} // namespace examples

namespace {
template <typename Type>
auto loadJson(const std::string &strNode, const std::string &strConfig) -> Type {
  std::istringstream streamNode(strNode);
  const auto jsonNode = Common::Json::loadFrom(streamNode);
  std::istringstream streamConfig(strConfig);
  const auto jsonConfig = Common::Json::loadFrom(streamConfig);
  return Type::loadFromJson(jsonNode, jsonConfig);
}
} // namespace

TEST_CASE("Viewing space coding") {
  REQUIRE(bitCodingTest(examples::viewingSpace[0], 115));
  REQUIRE(bitCodingTest(examples::viewingSpace[1], 199));
  REQUIRE(bitCodingTest(examples::viewingSpace[2], 179));
  REQUIRE(bitCodingTest(examples::viewingSpace[3], 195));
  REQUIRE(bitCodingTest(examples::viewingSpace[4], 555));
  REQUIRE(bitCodingTest(examples::viewingSpace[5], 115));
  REQUIRE(bitCodingTest(examples::viewingSpace[6], 275));
  REQUIRE(bitCodingTest(examples::viewingSpace[7], 83));
}

TEST_CASE("Viewing space JSON") {
  REQUIRE(loadJson<ViewingSpace>(examples::viewingSpaceJson[0], examples::configJson[0]) ==
          examples::viewingSpace[0]);
  REQUIRE(loadJson<ViewingSpace>(examples::viewingSpaceJson[1], examples::configJson[0]) ==
          examples::viewingSpace[4]);
  REQUIRE(loadJson<ViewingSpace>(examples::viewingSpaceJson[2], examples::configJson[0]) ==
          examples::viewingSpace[7]);
}
} // namespace TMIV::MivBitstream
