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

#include <TMIV/Encoder/IMpiEncoder.h>

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Encoder/MivEncoder.h>
#include <TMIV/IO/IO.h>

#include <fstream>
#include <iostream>

using namespace std::string_view_literals;

using Mat1w = TMIV::Common::heap::Matrix<uint16_t>;

namespace TMIV::Encoder {
void registerComponents();

// TODO(BK): Code duplication with Encoder.main.cpp
class Application : public Common::Application {
private:
  std::unique_ptr<IMpiEncoder> m_encoder;
  const std::string &m_contentId;
  std::int32_t m_numberOfInputFrames;
  std::int32_t m_intraPeriod;

  MivBitstream::SequenceConfig m_inputSequenceConfig;
  std::filesystem::path m_outputBitstreamPath;
  std::ofstream m_outputBitstream;
  std::unique_ptr<Encoder::MivEncoder> m_mivEncoder;

  [[nodiscard]] auto placeholders() const {
    auto x = IO::Placeholders{};
    x.contentId = m_contentId;
    x.numberOfInputFrames = m_numberOfInputFrames;
    return x;
  }

public:
  explicit Application(std::vector<const char *> argv)
      : Common::Application{"MpiEncoder", std::move(argv),
                            Common::Application::Options{
                                {"-s", "Content ID (e.g. B for Museum)", false},
                                {"-n", "Number of input frames (e.g. 97)", false}}}
      , m_encoder{create<IMpiEncoder>("Encoder")}
      , m_contentId{optionValues("-s"sv).front()}
      , m_numberOfInputFrames{std::stoi(optionValues("-n"sv).front())}
      , m_intraPeriod{json().require("intraPeriod").as<std::int32_t>()}
      , m_inputSequenceConfig{IO::loadSequenceConfig(json(), placeholders(), 0)}
      , m_outputBitstreamPath{IO::outputBitstreamPath(json(), placeholders())}
      , m_outputBitstream{m_outputBitstreamPath, std::ios::binary} {
    if (!m_outputBitstream.good()) {
      throw std::runtime_error(fmt::format("Failed to open {} for writing", m_outputBitstreamPath));
    }

    // Support experiments that use a subset of the source cameras
    if (const auto &node = json().optional(IO::inputCameraNames)) {
      std::cout << "WARNING: Source camera names are derived from the sequence configuration. This "
                   "functionality to override source camera names is only for internal testing, "
                   "e.g. to test with a subset of views.\n";
      m_inputSequenceConfig.sourceCameraNames = node.asVector<std::string>();
    }

    m_mivEncoder = std::make_unique<MivEncoder>(m_outputBitstream);
  }

  void run() override {
    auto sourceParams = loadSourceParams(json(), m_inputSequenceConfig);

    if (1 < m_inputSequenceConfig.sourceViewParams().size()) {
      throw std::runtime_error("Only one input MPI camera is allowed with current version of MPI "
                               "encoder. Please change inputCameraNames field in json !!!");
    }

    const auto &mpiViewParams = sourceParams.viewParamsList.front();
    Common::Vec2i mpiSize{static_cast<int>(mpiViewParams.ci.ci_projection_plane_width_minus1()) + 1,
                          static_cast<int>(mpiViewParams.ci.ci_projection_plane_height_minus1()) +
                              1};

    m_encoder->setTextureMpiLayerReader(
        [&](int frameIndex, int mpiLayerIndex) -> Common::TextureFrame {
          return IO::loadMpiTextureMpiLayer(json(), placeholders(), m_inputSequenceConfig,
                                            frameIndex, mpiLayerIndex, mpiViewParams.nbMpiLayers);
        });

    m_encoder->setTransparencyMpiLayerReader([&](int frameIndex,
                                                 int mpiLayerIndex) -> Common::Transparency10Frame {
      return IO::loadMpiTransparencyMpiLayer(json(), placeholders(), m_inputSequenceConfig,
                                             frameIndex, mpiLayerIndex, mpiViewParams.nbMpiLayers);
    });

    m_encoder->prepareSequence(sourceParams);

    for (int i = 0; i < m_numberOfInputFrames; i += m_intraPeriod) {
      int lastFrame = std::min(m_numberOfInputFrames, i + m_intraPeriod);
      encodeAccessUnit(i, lastFrame);
    }

    reportSummary(m_outputBitstream.tellp());
  }

private:
  // TODO(BK): Move this to EncoderLib
  static auto loadSourceParams(const Common::Json &config,
                               const MivBitstream::SequenceConfig &sequenceConfig)
      -> MivBitstream::EncoderParams {
    const auto haveGeometryVideo = config.require("haveGeometryVideo").as<bool>();
    const auto haveOccupancyVideo = config.require("haveOccupancyVideo").as<bool>();

    if (haveOccupancyVideo) {
      throw std::runtime_error("No occupancy is allowed with current version of MPI "
                               "encoder. Please use haveOccupancyVideo = false !!!");
    }
    if (haveGeometryVideo) {
      throw std::runtime_error("No geometry is allowed with current version of MPI "
                               "encoder. Please use haveGeometryVideo = false !!!");
    }

    std::vector<Common::Vec2i> m_overrideAtlasFrameSizes{};

    // Enforce user-specified atlas size
    if (auto node = config.require("overrideAtlasFrameSizes")) {
      for (const auto &subnode : node.as<Common::Json::Array>()) {
        m_overrideAtlasFrameSizes.push_back(subnode.asVec<int, 2>());
      }
    }

    auto x = MivBitstream::EncoderParams{m_overrideAtlasFrameSizes, 10, 0, 0, 10};

    x.viewParamsList = sequenceConfig.sourceViewParams();
    x.frameRate = sequenceConfig.frameRate;

    if (const auto &node = config.optional("depthLowQualityFlag")) {
      x.casme().casme_depth_low_quality_flag(node.as<bool>());
    }

    if (const auto &subnode = config.optional("ViewingSpace")) {
      x.viewingSpace = MivBitstream::ViewingSpace::loadFromJson(subnode, config);
    }

    return x;
  }

  void encodeAccessUnit(int firstFrame, int lastFrame) {
    std::cout << "Access unit: [" << firstFrame << ", " << lastFrame << ")\n";
    m_mivEncoder->writeAccessUnit(m_encoder->processAccessUnit(firstFrame, lastFrame));
    popAtlases(firstFrame, lastFrame);
  }

  void popAtlases(int firstFrame, int lastFrame) {
    for (int i = firstFrame; i < lastFrame; ++i) {
      IO::saveAtlasFrame(json(), placeholders(), i, m_encoder->popAtlas(i));
    }
  }

  void reportSummary(std::streampos bytesWritten) const {
    fmt::print("Maximum luma samples per frame is {}\n", m_encoder->maxLumaSamplesPerFrame());
    fmt::print("Total size is {} B ({} kb)\n", bytesWritten, 8e-3 * bytesWritten);
    fmt::print("Frame count is {}\n", m_numberOfInputFrames);
    fmt::print("Frame rate is {} Hz\n", m_inputSequenceConfig.frameRate);
    fmt::print("Total bitrate is {} kbps\n",
               8e-3 * bytesWritten * m_inputSequenceConfig.frameRate / m_numberOfInputFrames);
  }
};
} // namespace TMIV::Encoder

auto main(int argc, char *argv[]) -> int {
  try {
    TMIV::Encoder::registerComponents();
    TMIV::Encoder::Application app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
