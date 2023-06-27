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

#include <TMIV/IO/IO.h>

#include <fstream>
#include <regex>

using namespace std::string_literals;

namespace TMIV::IO {
const std::string configDirectory = "configDirectory";
const std::string inputBitstreamPathFmt = "inputBitstreamPathFmt";
const std::string inputCameraNames = "inputCameraNames";
const std::string inputDirectory = "inputDirectory";
const std::string inputEntityPathFmt = "inputEntityPathFmt";
const std::string inputGeometryPathFmt = "inputGeometryPathFmt";
const std::string inputGeometryVideoFramePathFmt = "inputGeometryVideoFramePathFmt"s;
const std::string inputGeometryVsbPathFmt = "inputGeometryVideoSubBitstreamPathFmt";
const std::string inputMaterialIdVsbPathFmt = "inputMaterialIdVideoSubBitstreamPathFmt";
const std::string inputNormalVsbPathFmt = "inputNormalVideoSubBitstreamPathFmt";
const std::string inputOccupancyVideoFramePathFmt = "inputOccupancyVideoFramePathFmt";
const std::string inputOccupancyVsbPathFmt = "inputOccupancyVideoSubBitstreamPathFmt";
const std::string inputPoseTracePathFmt = "inputPoseTracePathFmt";
const std::string inputReflectanceVsbPathFmt = "inputReflectanceVideoSubBitstreamPathFmt";
const std::string inputSequenceConfigPathFmt = "inputSequenceConfigPathFmt";
const std::string inputTexturePathFmt = "inputTexturePathFmt";
const std::string inputTextureVideoFramePathFmt = "inputTextureVideoFramePathFmt";
const std::string inputTextureVsbPathFmt = "inputTextureVideoSubBitstreamPathFmt";
const std::string inputTransparencyPathFmt = "inputTransparencyPathFmt";
const std::string inputTransparencyVideoFramePathFmt = "inputTransparencyVideoFramePathFmt";
const std::string inputTransparencyVsbPathFmt = "inputTransparencyVideoSubBitstreamPathFmt";
const std::string inputViewportParamsPathFmt = "inputViewportParamsPathFmt"s;

template <typename FORMAT>
auto loadFrame(const std::filesystem::path &path, std::int32_t frameIndex, Common::Vec2i frameSize)
    -> Common::Frame<FORMAT> {
  auto result = Common::Frame<FORMAT>(frameSize.x(), frameSize.y());

  std::ifstream stream{path, std::ios::binary};
  if (!stream.good()) {
    throw std::runtime_error(fmt::format("Failed to open {} for reading", path));
  }

  stream.seekg(std::streampos(frameIndex) * result.getDiskSize());
  if (!stream.good()) {
    throw std::runtime_error(
        fmt::format("Failed to seek for reading to frame {} from {}", frameIndex, path));
  }

  result.read(stream);
  if (!stream.good()) {
    throw std::runtime_error(fmt::format("Failed to read frame {} from {}", frameIndex, path));
  }

  return result;
}

template auto loadFrame(const std::filesystem::path &path, std::int32_t frameIndex,
                        Common::Vec2i frameSize) -> Common::Frame<Common::YUV400P10>;

namespace {
template <typename FORMAT>
auto loadSourceDepth_(int bits, const std::filesystem::path &path, std::int32_t frameIndex,
                      const Common::Vec2i &frameSize) {
  auto depth16 = Common::Depth16Frame{frameSize.x(), frameSize.y()};

  const auto depth = loadFrame<FORMAT>(path, frameIndex, frameSize);

  std::transform(std::begin(depth.getPlane(0)), std::end(depth.getPlane(0)),
                 std::begin(depth16.getPlane(0)), [bits](unsigned x) {
                   const auto x_max = Common::maxLevel(bits);
                   assert(0 <= x && x <= x_max);
                   const auto y = (0xFFFF * x + x_max / 2) / x_max;
                   assert(0 <= y && y <= UINT16_MAX);
                   return static_cast<uint16_t>(y);
                 });

  return depth16;
}

auto loadSourceDepth(int bits, const std::filesystem::path &path, std::int32_t frameIndex,
                     const Common::Vec2i &frameSize) {
  if (0 < bits && bits <= 8) {
    return loadSourceDepth_<Common::YUV400P8>(bits, path, frameIndex, frameSize);
  }
  if (8 < bits && bits <= 16) {
    return loadSourceDepth_<Common::YUV400P16>(bits, path, frameIndex, frameSize);
  }
  throw std::runtime_error("Invalid source geoemetry bit depth");
}

template <typename FORMAT>
auto loadSourceEntities_(const std::filesystem::path &path, std::int32_t frameIndex,
                         const Common::Vec2i &frameSize) {
  auto entities16 = Common::EntityMap{frameSize.x(), frameSize.y()};

  const auto entities = loadFrame<FORMAT>(path, frameIndex, frameSize);

  std::copy(entities.getPlane(0).begin(), entities.getPlane(0).end(),
            entities16.getPlane(0).begin());

  return entities16;
}

auto loadSourceEntities(int bits, const std::filesystem::path &path, std::int32_t frameIndex,
                        const Common::Vec2i frameSize) {
  if (0 < bits && bits <= 8) {
    return loadSourceEntities_<Common::YUV400P8>(path, frameIndex, frameSize);
  }
  if (8 < bits && bits <= 16) {
    return loadSourceEntities_<Common::YUV400P16>(path, frameIndex, frameSize);
  }
  throw std::runtime_error("Invalid source entity bit depth");
}
} // namespace

auto loadMultiviewFrame(const Common::Json &config, const Placeholders &placeholders,
                        const MivBitstream::SequenceConfig &sc, std::int32_t frameIndex)
    -> Common::MVD16Frame {
  auto frame = Common::MVD16Frame(sc.sourceCameraNames.size());

  const auto inputDir = config.require(inputDirectory).as<std::filesystem::path>();
  const auto startFrame = sc.startFrameGiven(placeholders.numberOfInputFrames);
  fmt::print("Loading multiview frame {0} with start frame offset {1} (= {2}).\n", frameIndex,
             startFrame, frameIndex + startFrame);

  for (size_t v = 0; v < frame.size(); ++v) {
    const auto name = sc.sourceCameraNames[v];
    const auto camera = sc.cameraByName(name);
    const auto &vp = camera.viewParams;
    const auto frameSize = vp.ci.projectionPlaneSize();

    if (const auto &node = config.optional(inputTexturePathFmt)) {
      const auto path =
          inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                 placeholders.contentId, placeholders.testId, name, frameSize.x(),
                                 frameSize.y(), camera.textureVideoFormat());
      frame[v].texture = loadFrame<Common::YUV420P10>(path, startFrame + frameIndex, frameSize);
    }

    if (const auto &node = config.optional(inputTransparencyPathFmt)) {
      const auto path =
          inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                 placeholders.contentId, placeholders.testId, name, frameSize.x(),
                                 frameSize.y(), camera.transparencyVideoFormat());
      frame[v].transparency =
          loadFrame<Common::YUV400P10>(path, startFrame + frameIndex, frameSize);
    }

    if (const auto &node = config.optional(inputGeometryPathFmt)) {
      const auto path =
          inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                 placeholders.contentId, placeholders.testId, name, frameSize.x(),
                                 frameSize.y(), camera.geometryVideoFormat());
      frame[v].depth =
          loadSourceDepth(camera.bitDepthDepth, path, startFrame + frameIndex, frameSize);
    }

    if (const auto &node = config.optional(inputEntityPathFmt)) {
      const auto path =
          inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                 placeholders.contentId, placeholders.testId, name, frameSize.x(),
                                 frameSize.y(), camera.entitiesVideoFormat());
      frame[v].entities =
          loadSourceEntities(camera.bitDepthEntities, path, startFrame + frameIndex, frameSize);
    }
  }

  return frame;
}

auto loadMpiTextureMpiLayer(const Common::Json &config, const Placeholders &placeholders,
                            const MivBitstream::SequenceConfig &sc, std::int32_t frameIndex,
                            int mpiLayerIndex, int nbMpiLayers) -> Common::TextureFrame {
  const auto inputDir = config.require(inputDirectory).as<std::filesystem::path>();
  const auto &node = config.require(inputTexturePathFmt);

  // NOTE(FT): one single source camera as MPI
  const auto name = sc.sourceCameraNames[0];

  const auto camera = sc.cameraByName(name);
  const auto &vp = camera.viewParams;
  const auto frameSize = vp.ci.projectionPlaneSize();

  const auto path =
      inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                             placeholders.contentId, placeholders.testId, name, frameSize.x(),
                             frameSize.y(), camera.textureVideoFormat());
  auto texture =
      loadFrame<Common::YUV420P10>(path, frameIndex * nbMpiLayers + mpiLayerIndex, frameSize);

  return texture;
}

auto loadMpiTransparencyMpiLayer(const Common::Json &config, const Placeholders &placeholders,
                                 const MivBitstream::SequenceConfig &sc, std::int32_t frameIndex,
                                 int mpiLayerIndex, int nbMpiLayers)
    -> Common::Transparency10Frame {
  const auto inputDir = config.require(inputDirectory).as<std::filesystem::path>();
  const auto &node = config.require(inputTransparencyPathFmt);

  // NOTE(FT): one single source camera as MPI
  const auto name = sc.sourceCameraNames[0];

  const auto camera = sc.cameraByName(name);
  const auto &vp = camera.viewParams;
  const auto frameSize = vp.ci.projectionPlaneSize();

  const auto path =
      inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                             placeholders.contentId, placeholders.testId, name, frameSize.x(),
                             frameSize.y(), camera.transparencyVideoFormat());

  auto transparency =
      loadFrame<Common::YUV400P8>(path, frameIndex * nbMpiLayers + mpiLayerIndex, frameSize);

  auto transparency10 = Common::Transparency10Frame{frameSize.x(), frameSize.y()};
  std::transform(std::begin(transparency.getPlane(0)), std::end(transparency.getPlane(0)),
                 std::begin(transparency10.getPlane(0)), [](unsigned x) {
                   const auto x_max = 255U;
                   assert(0 <= x && x <= x_max);
                   const auto y = (0x03FF * x + x_max / 2) / x_max;
                   assert(0 <= y && y <= 1023U);
                   return static_cast<uint16_t>(y);
                 });

  return transparency10;
}

namespace {
struct Pose {
  Common::Vec3f position;
  Common::Vec3f rotation;
};

auto loadPoseFromCSV(std::istream &stream, std::int32_t frameIndex) -> Pose {
  std::string line;
  getline(stream, line);

  std::regex re_header(R"(\s*X\s*,\s*Y\s*,\s*Z\s*,\s*Yaw\s*,\s*Pitch\s*,\s*Roll\s*)");
  if (!std::regex_match(line, re_header)) {
    throw std::runtime_error("Format error in the pose trace header");
  }

  int currentFrameIndex = 0;
  std::regex re_row("([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+)");
  std::regex re_empty("\\s*");
  bool trailing_empty_lines = false;

  while (getline(stream, line)) {
    std::smatch match;
    if (!trailing_empty_lines && std::regex_match(line, match, re_row)) {
      if (currentFrameIndex == frameIndex) {
        return {Common::Vec3f({stof(match[1].str()), stof(match[2].str()), stof(match[3].str())}),
                Common::Vec3f({stof(match[4].str()), stof(match[5].str()), stof(match[6].str())})};
      }
      { currentFrameIndex++; }
    } else if (std::regex_match(line, re_empty)) {
      trailing_empty_lines = true;
    } else {
      throw std::runtime_error("Format error in a pose trace row");
    }
  }

  throw std::runtime_error("Unable to load required frame index " + std::to_string(frameIndex));
}
} // namespace

auto loadViewportMetadata(const Common::Json &config, const Placeholders &placeholders,
                          std::int32_t frameIndex, const std::string &cameraName, bool isPoseTrace)
    -> MivBitstream::ViewParams {
  const auto viewportParamsPath =
      config.require(configDirectory).as<std::filesystem::path>() /
      fmt::format(config.require(inputViewportParamsPathFmt).as<std::string>(),
                  placeholders.numberOfInputFrames, placeholders.contentId, placeholders.testId);
  std::ifstream stream{viewportParamsPath};
  if (!stream.good()) {
    throw std::runtime_error(
        fmt::format("Failed to load viewport parameters from {}", viewportParamsPath));
  }

  const auto sequenceConfig = MivBitstream::SequenceConfig{stream};
  auto result = sequenceConfig.cameraByName(isPoseTrace ? "viewport"s : cameraName).viewParams;

  // The result may have invalid depth values
  result.hasOccupancy = true;

  if (isPoseTrace) {
    const auto poseTracePath = config.require(configDirectory).as<std::filesystem::path>() /
                               fmt::format(config.require(inputPoseTracePathFmt).as<std::string>(),
                                           placeholders.numberOfInputFrames, placeholders.contentId,
                                           placeholders.testId, cameraName);
    std::ifstream stream{poseTracePath};
    if (!stream.good()) {
      throw std::runtime_error(
          fmt::format("Failed to load pose trace file from {}", poseTracePath));
    }

    const auto pose = loadPoseFromCSV(stream, frameIndex);

    result.ce.position(result.ce.position() + pose.position);
    result.ce.rotation(euler2quat(Common::radperdeg * pose.rotation));
  }

  return result;
} // namespace TMIV::IO

namespace {
template <typename FORMAT>
auto loadVideoFrame(const std::string &key, const Common::Json &config,
                    const Placeholders &placeholders, MivBitstream::AtlasId atlasId,
                    uint32_t frameId, Common::Vec2i frameSize) -> Common::Frame<FORMAT> {
  return IO::loadFrame<FORMAT>(config.require(IO::outputDirectory).as<std::filesystem::path>() /
                                   fmt::format(config.require(key).as<std::string>(),
                                               placeholders.numberOfInputFrames,
                                               placeholders.contentId, placeholders.testId, atlasId,
                                               frameSize.x(), frameSize.y()),
                               frameId, frameSize);
}
} // namespace

auto loadOccupancyVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                             MivBitstream::AtlasId atlasId, uint32_t frameId,
                             Common::Vec2i frameSize) -> Common::Occupancy10Frame {
  return loadVideoFrame<Common::YUV400P10>(IO::inputOccupancyVideoFramePathFmt, config,
                                           placeholders, atlasId, frameId, frameSize);
}

auto loadGeometryVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                            MivBitstream::AtlasId atlasId, uint32_t frameId,
                            Common::Vec2i frameSize) -> Common::Depth10Frame {
  return loadVideoFrame<Common::YUV400P10>(IO::inputGeometryVideoFramePathFmt, config, placeholders,
                                           atlasId, frameId, frameSize);
}

auto loadTextureVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                           MivBitstream::AtlasId atlasId, uint32_t frameId, Common::Vec2i frameSize)
    -> Common::Texture444Frame {
  return Common::yuv444p(loadVideoFrame<Common::YUV420P10>(
      IO::inputTextureVideoFramePathFmt, config, placeholders, atlasId, frameId, frameSize));
}

auto loadTransparencyVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                                MivBitstream::AtlasId atlasId, uint32_t frameId,
                                Common::Vec2i frameSize) -> Common::Transparency10Frame {
  return loadVideoFrame<Common::YUV400P10>(IO::inputTransparencyVideoFramePathFmt, config,
                                           placeholders, atlasId, frameId, frameSize);
}

namespace detail {
template <bool allowNullopt>
auto tryLoadSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                           std::int32_t frameIndex)
    -> std::conditional_t<allowNullopt, std::optional<MivBitstream::SequenceConfig>,
                          MivBitstream::SequenceConfig> {
  const auto relPath = fmt::format(config.require(IO::inputSequenceConfigPathFmt).as<std::string>(),
                                   placeholders.numberOfInputFrames, placeholders.contentId,
                                   placeholders.testId, frameIndex);
  const auto path1 = config.require(IO::inputDirectory).as<std::filesystem::path>() / relPath;
  const auto path2 = config.require(IO::configDirectory).as<std::filesystem::path>() / relPath;
  const auto path = exists(path1) ? path1 : path2;

  if constexpr (allowNullopt) {
    if (!exists(path)) {
      return std::nullopt;
    }
  }
  std::ifstream stream{path};
  if (!stream.good()) {
    throw std::runtime_error(
        fmt::format("Failed to load source camera parameters from {} or {} (with current path {})",
                    path1, path2, std::filesystem::current_path()));
  }
  return MivBitstream::SequenceConfig{stream};
}
} // namespace detail

auto loadSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                        std::int32_t frameIndex) -> MivBitstream::SequenceConfig {
  return detail::tryLoadSequenceConfig<false>(config, placeholders, frameIndex);
}

auto tryLoadSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                           std::int32_t frameIndex) -> std::optional<MivBitstream::SequenceConfig> {
  return detail::tryLoadSequenceConfig<true>(config, placeholders, frameIndex);
}

auto inputBitstreamPath(const Common::Json &config, const Placeholders &placeholders)
    -> std::filesystem::path {
  return config.require(IO::inputDirectory).as<std::filesystem::path>() /
         fmt::format(config.require(IO::inputBitstreamPathFmt).as<std::string>(),
                     placeholders.numberOfInputFrames, placeholders.contentId, placeholders.testId);
}

auto inputSubBitstreamPath(const std::string &key, const Common::Json &config,
                           const Placeholders &placeholders, MivBitstream::AtlasId atlasId,
                           int attributeIdx) -> std::filesystem::path {
  return config.require(IO::inputDirectory).as<std::filesystem::path>() /
         fmt::format(config.require(key).as<std::string>(), placeholders.numberOfInputFrames,
                     placeholders.contentId, placeholders.testId, atlasId, attributeIdx);
}
} // namespace TMIV::IO
