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

#include <TMIV/Renderer/Front/mapInputToOutputFrames.h>

TEST_CASE("Map input to output frames") {
  using TMIV::Renderer::Front::FrameMapping;
  using TMIV::Renderer::Front::mapInputToOutputFrames;

  static_assert(std::is_same_v<FrameMapping, decltype(mapInputToOutputFrames(0, 0))>);

  GIVEN("a negative number of input frames") {
    const std::int32_t numberOfInputFrames = GENERATE(INT32_MIN, -1);
    const std::int32_t numberOfOutputFrames = GENERATE(0, 1, 2);

    THEN("a runtime error is thrown") {
      REQUIRE_THROWS(mapInputToOutputFrames(numberOfInputFrames, numberOfOutputFrames));
    }
  }

  GIVEN("a negative number of output frames") {
    const std::int32_t numberOfInputFrames = GENERATE(0, 1, 2);
    const std::int32_t numberOfOutputFrames = GENERATE(INT32_MIN, -1);

    THEN("a runtime error is thrown") {
      REQUIRE_THROWS(mapInputToOutputFrames(numberOfInputFrames, numberOfOutputFrames));
    }
  }

  GIVEN("zero input frames") {
    const auto numberOfInputFrames = 0;

    WHEN("there are zero output frames") {
      const auto numberOfOutputFrames = 0;

      THEN("the frame mapping is empty") {
        CHECK(mapInputToOutputFrames(numberOfInputFrames, numberOfOutputFrames).empty());
      }
    }

    WHEN("there are output frames") {
      const auto numberOfOutputFrames = GENERATE(1, 13);

      THEN("a runtime error is thrown") {
        REQUIRE_THROWS(mapInputToOutputFrames(numberOfInputFrames, numberOfOutputFrames));
      }
    }
  }

  GIVEN("a positive number of input frames") {
    const auto numberOfInputFrames = GENERATE(1, 2, 9);

    WHEN("there are as much output frames") {
      const auto numberOfOutputFrames = numberOfInputFrames;

      THEN("The frame mapping is the identity mapping") {
        const auto x = mapInputToOutputFrames(numberOfInputFrames, numberOfOutputFrames);
        CHECK(x.size() == numberOfOutputFrames);

        auto i = 0;
        for (auto [first, second] : x) {
          CHECK(i == first);
          CHECK(i == second);
          ++i;
        }
      }
    }

    WHEN("there are less output frames") {
      const auto numberOfOutputFrames = std::max(1, numberOfInputFrames / 2);

      THEN("The frame mapping is the identify mapping") {
        const auto x = mapInputToOutputFrames(numberOfInputFrames, numberOfOutputFrames);
        CHECK(x.size() == numberOfOutputFrames);

        auto i = 0;
        for (auto [first, second] : x) {
          CHECK(i == first);
          CHECK(i == second);
          ++i;
        }
      }
    }

    WHEN("there are more output frames") {
      const auto numberOfOutputFrames = numberOfInputFrames + GENERATE(1, 3, 40);

      THEN("The frame mapping zigzags") {
        const auto actual = mapInputToOutputFrames(numberOfInputFrames, numberOfOutputFrames);
        CHECK(actual.size() == numberOfOutputFrames);

        auto reference = FrameMapping{};

        auto i = 0;
        auto di = 1;

        for (auto j = 0; j < numberOfOutputFrames; ++j) {
          reference.emplace(i, j);

          i += di;

          if (i == numberOfInputFrames) {
            di = -1;
            i = numberOfInputFrames - 1;
          } else if (i == -1) {
            di = 1;
            i = 0;
          }
        }

        CHECK(actual == reference);
      }
    }
  }
}
