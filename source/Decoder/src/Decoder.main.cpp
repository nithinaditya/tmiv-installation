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

#include <TMIV/Decoder/IDecoder.h>

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Decoder/MivDecoder.h>
#include <TMIV/Decoder/V3cSampleStreamDecoder.h>
#include <TMIV/IO/IO.h>
#include <TMIV/MivBitstream/SequenceConfig.h>
#include <TMIV/Renderer/Front/MultipleFrameRenderer.h>
#include <TMIV/Renderer/Front/mapInputToOutputFrames.h>
#include <TMIV/Renderer/RecoverPrunedViews.h>

#include <fstream>
#include <iostream>
#include <memory>

using namespace std::string_view_literals;

namespace TMIV::Decoder {
void registerComponents();

class Application : public Common::Application {
private:
  std::unique_ptr<IDecoder> m_decoder;
  IO::Placeholders m_placeholders;
  Renderer::Front::MultipleFrameRenderer m_renderer;
  std::multimap<int, int> m_inputToOutputFrameIdMap;
  std::filesystem::path m_inputBitstreamPath;
  std::ifstream m_inputBitstream;
  Decoder::V3cSampleStreamDecoder m_vssDecoder;
  Decoder::MivDecoder m_mivDecoder;
  MivBitstream::SequenceConfig m_outputSequenceConfig;

public:
  explicit Application(std::vector<const char *> argv)
      : Common::Application{"Decoder", std::move(argv),
                            Common::Application::Options{
                                {"-s", "Content ID (e.g. B for Museum)", false},
                                {"-n", "Number of input frames (e.g. 97)", false},
                                {"-N", "Number of output frames (e.g. 300)", false},
                                {"-r", "Test point (e.g. QP3 or R0)", false},
                                {"-v", "Source view to render (e.g. v11)", true},
                                {"-P", "Pose trace to render (e.g. p02)", true}}}
      , m_decoder{create<IDecoder>("Decoder")}
      , m_placeholders{optionValues("-s").front(), optionValues("-r").front(),
                       std::stoi(optionValues("-n"sv).front()),
                       std::stoi(optionValues("-N"sv).front())}
      , m_renderer{json(), optionValues("-v"), optionValues("-P"), m_placeholders}
      , m_inputToOutputFrameIdMap{Renderer::Front::mapInputToOutputFrames(
            m_placeholders.numberOfInputFrames, m_placeholders.numberOfOutputFrames)}
      , m_inputBitstreamPath{IO::inputBitstreamPath(json(), m_placeholders)}
      , m_inputBitstream{m_inputBitstreamPath, std::ios::binary}
      , m_vssDecoder{createVssDecoder()}
      , m_mivDecoder{[this]() { return m_vssDecoder(); }} {
    setFrameServers(m_mivDecoder);
  }

  void run() override {
    while (auto frame = m_mivDecoder()) {
      // Check which frames to render if we would
      const auto range = m_inputToOutputFrameIdMap.equal_range(frame->foc);
      if (range.first == range.second) {
        return;
      }

      auto &ptl = frame->vps.profile_tier_level();
      if (m_renderer.isOptimizedForRestrictedGeometry()) {
        if (ptl.ptl_profile_toolset_idc() != MivBitstream::PtlProfilePccToolsetIdc::MIV_Extended ||
            !ptl.ptl_toolset_constraints_present_flag() ||
            !ptl.ptl_profile_toolset_constraints_information().ptc_restricted_geometry_flag()) {
          throw std::runtime_error("Restricted geometry only renderer.");
        }
      }

      // Recover geometry, occupancy, and filter blockToPatchMap
      m_decoder->recoverFrame(*frame);

      if (json().optional(IO::outputSequenceConfigPathFmt)) {
        outputSequenceConfig(frame->sequenceConfig(), frame->foc);
      }

      if (json().optional(IO::outputBlockToPatchMapPathFmt)) {
        IO::saveBlockToPatchMaps(json(), m_placeholders, frame->foc, *frame);
      }

      if (json().optional(IO::outputMultiviewTexturePathFmt) ||
          json().optional(IO::outputMultiviewGeometryPathFmt) ||
          json().optional(IO::outputMultiviewOccupancyPathFmt)) {
        IO::savePrunedFrame(json(), m_placeholders, frame->foc,
                            Renderer::recoverPrunedViewAndMask(*frame));
      }

      m_renderer.renderMultipleFrames(*frame, range.first, range.second);
    }
  }

private:
  auto createVssDecoder() -> V3cSampleStreamDecoder {
    if (!m_inputBitstream.good()) {
      throw std::runtime_error(fmt::format("Failed to open {} for reading", m_inputBitstreamPath));
    }
    return V3cSampleStreamDecoder{m_inputBitstream};
  }

  void setFrameServers(MivDecoder &mivDecoder) const {
    mivDecoder.setOccFrameServer([this](MivBitstream::AtlasId atlasId, int32_t frameIndex,
                                        Common::Vec2i frameSize) -> Common::Occupancy10Frame {
      if (frameIndex < m_placeholders.numberOfInputFrames) {
        return IO::loadOccupancyVideoFrame(json(), m_placeholders, atlasId, frameIndex, frameSize);
      }
      return Common::Depth10Frame{};
    });
    mivDecoder.setGeoFrameServer([this](MivBitstream::AtlasId atlasId, int32_t frameIndex,
                                        Common::Vec2i frameSize) -> Common::Depth10Frame {
      if (frameIndex < m_placeholders.numberOfInputFrames) {
        return IO::loadGeometryVideoFrame(json(), m_placeholders, atlasId, frameIndex, frameSize);
      }
      return Common::Depth10Frame{};
    });
    mivDecoder.setTextureFrameServer([this](MivBitstream::AtlasId atlasId, int32_t frameIndex,
                                            Common::Vec2i frameSize) -> Common::Texture444Frame {
      if (frameIndex < m_placeholders.numberOfInputFrames) {
        return IO::loadTextureVideoFrame(json(), m_placeholders, atlasId, frameIndex, frameSize);
      }
      return Common::Texture444Frame{};
    });
    mivDecoder.setTransparencyFrameServer(
        [this](MivBitstream::AtlasId atlasId, int32_t frameIndex,
               Common::Vec2i frameSize) -> Common::Transparency10Frame {
          if (frameIndex < m_placeholders.numberOfInputFrames) {
            return IO::loadTransparencyVideoFrame(json(), m_placeholders, atlasId, frameIndex,
                                                  frameSize);
          }
          return Common::Transparency10Frame{};
        });
  }

  void outputSequenceConfig(MivBitstream::SequenceConfig sc, std::int32_t foc) {
    // NOTE(#463): Inject the frame count into the sequence configuration, because the decoder
    // library does not have that knowledge.
    sc.numberOfFrames = std::stoi(optionValues("-n"sv).front());

    if (m_outputSequenceConfig != sc) {
      m_outputSequenceConfig = std::move(sc);
      IO::saveSequenceConfig(json(), m_placeholders, foc, m_outputSequenceConfig);
    }
  }
};
} // namespace TMIV::Decoder

auto main(int argc, char *argv[]) -> int {
  try {
    TMIV::Decoder::registerComponents();
    TMIV::Decoder::Application app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  } catch (std::logic_error &e) {
    std::cerr << e.what() << std::endl;
    return 3;
  }
}
