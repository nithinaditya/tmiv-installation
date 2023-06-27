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

#include <TMIV/Common/Factory.h>
#include <TMIV/Common/Json.h>
#include <TMIV/Renderer/PushPullInpainter.h>

TEST_CASE("Push-pull inpainter") {
  SECTION("Construction") {
    const auto rootNode = TMIV::Common::Json{};
    const auto componentNode = TMIV::Common::Json{};
    const auto inpainter = TMIV::Renderer::PushPullInpainter{rootNode, componentNode};
  }

  auto &factory = TMIV::Common::Factory<TMIV::Renderer::IInpainter>::getInstance();
  factory.registerAs<TMIV::Renderer::PushPullInpainter>("PushPullInpainter");

  SECTION("Factory creation") {
    const auto rootNode = TMIV::Common::Json{};
    const auto componentNode = TMIV::Common::Json::parse(R"({
    "InpainterMethod": "PushPullInpainter",
    "PushPullInpainter": {}
}
)");
    const auto inpainter = factory.create("Inpainter", rootNode, componentNode);
  }

  SECTION("Inpaint frame") {
    const auto w = 8;
    const auto h = 5;

    auto frame = std::pair{TMIV::Common::Texture444Frame{w, h}, TMIV::Common::Depth16Frame{w, h}};

    frame.first.getPlane(0)(h - 1, 0) = 100;
    frame.first.getPlane(1)(h - 1, 0) = 300;
    frame.first.getPlane(2)(h - 1, 0) = 900;
    frame.second.getPlane(0)(h - 1, 0) = 500;

    frame.first.getPlane(0)(0, w - 1) = 200;
    frame.first.getPlane(1)(0, w - 1) = 600;
    frame.first.getPlane(2)(0, w - 1) = 0;
    frame.second.getPlane(0)(0, w - 1) = 400;

    TMIV::Renderer::PushPullInpainter{{}, {}}.inplaceInpaint(frame, {});

    REQUIRE(std::all_of(frame.first.getPlane(0).cbegin(), frame.first.getPlane(0).cend(),
                        [](auto x) { return 100 <= x && x <= 200; }));
    REQUIRE(std::all_of(frame.first.getPlane(1).cbegin(), frame.first.getPlane(1).cend(),
                        [](auto x) { return 300 <= x && x <= 600; }));
    REQUIRE(std::all_of(frame.first.getPlane(2).cbegin(), frame.first.getPlane(2).cend(),
                        [](auto x) { return x <= 900; }));
    REQUIRE(std::all_of(frame.second.getPlane(0).cbegin(), frame.second.getPlane(0).cend(),
                        [](auto x) { return 400 <= x && x <= 500; }));
  }
}
