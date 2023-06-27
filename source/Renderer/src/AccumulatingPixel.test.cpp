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

#include <TMIV/Renderer/AccumulatingPixel.h>

SCENARIO("Pixel can be blended", "[AccumulatingPixel]") {
  using Pixel = TMIV::Renderer::AccumulatingPixel<TMIV::Common::Vec3f>;
  using Acc = TMIV::Renderer::PixelAccumulator<TMIV::Common::Vec3f>;
  using Value = TMIV::Renderer::PixelValue<TMIV::Common::Vec3f>;

  GIVEN("A default-constructed accumulator") {
    Acc acc;
    Pixel pixel{1.F, 1.F, 1.F, 10.F};

    THEN("The attributes are zero")
    REQUIRE(std::get<0>(acc.attributes()).x() == 0.F);
    REQUIRE(std::get<0>(acc.attributes()).y() == 0.F);
    REQUIRE(std::get<0>(acc.attributes()).z() == 0.F);

    WHEN("Averaging") {
      auto val = pixel.average(acc);

      THEN("The attributes are zero") {
        REQUIRE(std::get<0>(val.attributes()).x() == 0.F);
        REQUIRE(std::get<0>(val.attributes()).y() == 0.F);
        REQUIRE(std::get<0>(val.attributes()).z() == 0.F);
      }
    }
  }

  GIVEN("A pixel accumulator that is constructed from a pixel value") {
    float const ray_angle_param = 1.5F;
    float const depth_param = 60.7F;
    float const stretching_param = 3.2F;
    float const max_stretching = 10.F;
    float const ray_angle = 0.01F;
    float const stretching = 3.F;
    Pixel pixel{ray_angle_param, depth_param, stretching_param, max_stretching};

    float const ray_angle_weight = pixel.rayAngleWeight(ray_angle);
    float const stretching_weight = pixel.stretchingWeight(stretching);

    Value reference{{{0.3F, 0.7F, 0.1F}}, 0.53F, ray_angle_weight * stretching_weight, stretching};

    Acc accum = pixel.construct(reference.attributes(), reference.normDisp, ray_angle, stretching);

    WHEN("The pixel value is directly computed") {
      Value actual = pixel.average(accum);

      THEN("The average is the pixel value") {
        REQUIRE(std::get<0>(actual.attributes())[0] == std::get<0>(reference.attributes())[0]);
        REQUIRE(std::get<0>(actual.attributes())[1] == std::get<0>(reference.attributes())[1]);
        REQUIRE(std::get<0>(actual.attributes())[2] == std::get<0>(reference.attributes())[2]);
        REQUIRE(actual.normDisp == reference.normDisp);
        REQUIRE(actual.normWeight == reference.normWeight);
      }
    }

    WHEN("The pixel is blended with itself") {
      Acc accum2 = pixel.blend(accum, accum);
      Value actual = pixel.average(accum2);

      THEN("The average is the same but with double quality") {
        REQUIRE(std::get<0>(actual.attributes())[0] == std::get<0>(reference.attributes())[0]);
        REQUIRE(std::get<0>(actual.attributes())[1] == std::get<0>(reference.attributes())[1]);
        REQUIRE(std::get<0>(actual.attributes())[2] == std::get<0>(reference.attributes())[2]);
        REQUIRE(actual.normDisp == reference.normDisp);
        REQUIRE(actual.normWeight == 2 * reference.normWeight);
      }
    }

    WHEN("The pixel is blended with another pixel that has invalid depth") {
      Acc accumInvalid = pixel.construct(reference.attributes(), 0.F, ray_angle, stretching);
      Value actual = pixel.average(pixel.blend(accum, accumInvalid));

      THEN("It is af the pixel has not been blended") {
        REQUIRE(std::get<0>(actual.attributes())[0] == std::get<0>(reference.attributes())[0]);
        REQUIRE(std::get<0>(actual.attributes())[1] == std::get<0>(reference.attributes())[1]);
        REQUIRE(std::get<0>(actual.attributes())[2] == std::get<0>(reference.attributes())[2]);
        REQUIRE(actual.normDisp == reference.normDisp);
        REQUIRE(actual.normWeight == reference.normWeight);
      }
    }
  }
}
