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

#include <TMIV/Renderer/Front/MultipleFrameRenderer.h>

#include <TMIV/Common/Factory.h>
#include <TMIV/Renderer/ICuller.h>
#include <TMIV/Renderer/IRenderer.h>

using namespace std::string_view_literals;

// NOTE(BK): The MultipleFrameRenderer conducts file IO which cannot be unit-tested. Testing is
// limited to object construction and the trivial case of _not_ rendering any frames.
namespace {
struct FakeCuller : public TMIV::Renderer::ICuller {
  FakeCuller(const TMIV::Common::Json & /* rootNode */,
             const TMIV::Common::Json & /* componentNode */) {}

  [[nodiscard]] auto
  filterBlockToPatchMap(const TMIV::MivBitstream::AccessUnit & /* frame */,
                        const TMIV::MivBitstream::AtlasAccessUnit & /* atlas */,
                        const TMIV::MivBitstream::ViewParams & /* viewportParams */) const
      -> TMIV::Common::BlockToPatchMap override {
    throw std::logic_error("Unexpected call to filterBlockToPatchMap");
  }
};

struct FakeRenderer : public TMIV::Renderer::IRenderer {
  FakeRenderer(const TMIV::Common::Json & /* rootNode */,
               const TMIV::Common::Json & /* componentNode */) {}

  [[nodiscard]] auto renderFrame(const TMIV::MivBitstream::AccessUnit & /* frame */,
                                 const TMIV::MivBitstream::ViewParams & /* viewportParams */) const
      -> TMIV::Common::Texture444Depth16Frame override {
    throw std::logic_error("Unexpected call to renderFrame");
  }
};

} // namespace

TEST_CASE("Render multiple frames") {
  using TMIV::Renderer::Front::MultipleFrameRenderer;

  TMIV::Common::Factory<TMIV::Renderer::ICuller>::getInstance().registerAs<FakeCuller>(
      "FakeCuller");
  TMIV::Common::Factory<TMIV::Renderer::IRenderer>::getInstance().registerAs<FakeRenderer>(
      "FakeRenderer");

  SECTION("Construction") {
    const auto rootNode = TMIV::Common::Json::parse(R"(
{
    "CullerMethod": "FakeCuller",
    "FakeCuller": {},
    "RendererMethod": "FakeRenderer",
    "FakeRenderer": {}
}
)"sv);
    const auto outputCameraNames = std::vector<std::string>{};
    const auto outputPoseTraceNames = std::vector<std::string>{};
    const auto placeholders = TMIV::IO::Placeholders{};
    const auto unit =
        MultipleFrameRenderer{rootNode, outputCameraNames, outputPoseTraceNames, placeholders};

    SECTION("Rendering no frames") {
      const auto frame = TMIV::MivBitstream::AccessUnit{};

      SECTION("Empty input frame range") {
        const auto mapping = TMIV::Renderer::Front::FrameMapping{};
        unit.renderMultipleFrames(frame, mapping.begin(), mapping.end());
      }

      SECTION("No output cameras") {
        const auto mapping = TMIV::Renderer::Front::FrameMapping{{0, 0}, {1, 1}, {2, 0}};
        unit.renderMultipleFrames(frame, mapping.begin(), mapping.end());
      }
    }
  }
}