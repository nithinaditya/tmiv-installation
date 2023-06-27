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

using namespace std::string_literals;

namespace TMIV::IO {
const std::string outputBitstreamPathFmt = "outputBitstreamPathFmt";
const std::string outputBlockToPatchMapPathFmt = "outputBlockToPatchMapPathFmt";
const std::string outputDirectory = "outputDirectory";
const std::string outputGeometryVideoDataPathFmt = "outputGeometryVideoDataPathFmt";
const std::string outputMultiviewGeometryPathFmt = "outputMultiviewGeometryPathFmt";
const std::string outputMultiviewOccupancyPathFmt = "outputMultiviewOccupancyPathFmt";
const std::string outputMultiviewTexturePathFmt = "outputMultiviewTexturePathFmt";
const std::string outputMultiviewTransparencyPathFmt = "outputMultiviewTransparencyPathFmt";
const std::string outputOccupancyVideoDataPathFmt = "outputOccupancyVideoDataPathFmt";
const std::string outputSequenceConfigPathFmt = "outputSequenceConfigPathFmt";
const std::string outputTextureVideoDataPathFmt = "outputTextureVideoDataPathFmt";
const std::string outputTransparencyVideoDataPathFmt = "outputTransparencyVideoDataPathFmt";
const std::string outputViewportGeometryPathFmt = "outputViewportGeometryPathFmt";
const std::string outputViewportTexturePathFmt = "outputViewportTexturePathFmt";

namespace {
template <typename FORMAT> void padChroma(std::ostream &stream, int bytes) {
  constexpr auto fillValue = Common::neutralColor<FORMAT>();
  const auto padding = std::vector(bytes / sizeof(fillValue), fillValue);
  auto buffer = std::vector<char>(bytes);
  std::memcpy(buffer.data(), padding.data(), buffer.size());
  stream.write(buffer.data(), buffer.size());
}
} // namespace

template <typename FORMAT>
void saveFrame(const std::filesystem::path &path, const Common::Frame<FORMAT> &frame,
               std::int32_t frameIndex) {
  create_directories(path.parent_path());

  std::fstream stream(path, frameIndex == 0 ? std::ios::out | std::ios::binary
                                            : std::ios::in | std::ios::out | std::ios::binary);
  if (!stream.good()) {
    throw std::runtime_error(fmt::format("Failed to open {} for writing", path));
  }

  stream.seekp(int64_t{frameIndex} * frame.getDiskSize());
  if (!stream.good()) {
    throw std::runtime_error(
        fmt::format("Failed to seek for writing to frame {} of {}", frameIndex, path));
  }

  frame.dump(stream);
  padChroma<FORMAT>(stream, frame.getDiskSize() - frame.getMemorySize());
  if (!stream.good()) {
    throw std::runtime_error(fmt::format("Failed to write to {}", path));
  }
}

void saveAtlasFrame(const Common::Json &config, const Placeholders &placeholders,
                    std::int32_t frameIndex, const Common::MVD10Frame &frame) {
  const auto outputDir = config.require(outputDirectory).as<std::filesystem::path>();

  for (size_t k = 0; k < frame.size(); ++k) {
    if (const auto &node = config.optional(outputTextureVideoDataPathFmt)) {
      saveFrame(outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                        placeholders.contentId, placeholders.testId, k,
                                        frame[k].texture.getWidth(), frame[k].texture.getHeight()),
                frame[k].texture, frameIndex);
    }
    if (const auto &node = config.optional(outputTransparencyVideoDataPathFmt)) {
      saveFrame(outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                        placeholders.contentId, placeholders.testId, k,
                                        frame[k].transparency.getWidth(),
                                        frame[k].transparency.getHeight()),
                frame[k].transparency, frameIndex);
    }
    if (const auto &node = config.optional(outputGeometryVideoDataPathFmt)) {
      saveFrame(outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                        placeholders.contentId, placeholders.testId, k,
                                        frame[k].depth.getWidth(), frame[k].depth.getHeight()),
                frame[k].depth, frameIndex);
    }
    if (const auto &node = config.optional(outputOccupancyVideoDataPathFmt)) {
      saveFrame(outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                        placeholders.contentId, placeholders.testId, k,
                                        frame[k].occupancy.getWidth(),
                                        frame[k].occupancy.getHeight()),
                frame[k].occupancy, frameIndex);
    }
  }
}

void saveViewport(const Common::Json &config, const Placeholders &placeholders,
                  std::int32_t frameIndex, const std::string &name,
                  const Common::TextureDepth16Frame &frame) {
  const auto outputDir = config.require(outputDirectory).as<std::filesystem::path>();
  auto saved = false;

  if (const auto &node = config.optional(outputViewportTexturePathFmt)) {
    saveFrame(outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                      placeholders.contentId, placeholders.testId,
                                      placeholders.numberOfOutputFrames, name,
                                      frame.texture.getWidth(), frame.texture.getHeight(),
                                      "yuv420p10le"),
              frame.texture, frameIndex);
    saved = true;
  }
  if (const auto &node = config.optional(outputViewportGeometryPathFmt)) {
    saveFrame(outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                      placeholders.contentId, placeholders.testId,
                                      placeholders.numberOfOutputFrames, name,
                                      frame.depth.getWidth(), frame.depth.getHeight(),
                                      "yuv420p16le"),
              frame.depth, frameIndex);
    saved = true;
  }

  if (!saved) {
    fmt::print("WARNING: Calculated viewport but not saving texture or geometry. Add {} or {} to "
               "the configuration file.\n",
               outputViewportTexturePathFmt, outputViewportGeometryPathFmt);
  }
}

void saveBlockToPatchMaps(const Common::Json &config, const Placeholders &placeholders,
                          std::int32_t frameIndex, const MivBitstream::AccessUnit &frame) {
  const auto outputDir = config.require(outputDirectory).as<std::filesystem::path>();

  if (const auto &node = config.optional(outputBlockToPatchMapPathFmt)) {
    for (size_t k = 0; k < frame.atlas.size(); ++k) {
      const auto &btpm = frame.atlas[k].blockToPatchMap;
      saveFrame(outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                        placeholders.contentId, placeholders.testId, k,
                                        btpm.getWidth(), btpm.getHeight()),
                btpm, frameIndex);
    }
  }
}

void savePrunedFrame(const Common::Json &config, const Placeholders &placeholders,
                     std::int32_t frameIndex,
                     const std::pair<std::vector<Common::Texture444Depth10Frame>, Common::MaskList>
                         &prunedViewsAndMasks) {
  const auto outputDir = config.require(outputDirectory).as<std::filesystem::path>();

  for (size_t v = 0; v < prunedViewsAndMasks.first.size(); ++v) {
    if (const auto &node = config.optional(outputMultiviewTexturePathFmt)) {
      const auto &texture = prunedViewsAndMasks.first[v].first;
      saveFrame(outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                        placeholders.contentId, placeholders.testId, v,
                                        texture.getWidth(), texture.getHeight()),
                yuv420p(texture), frameIndex);
    }
    if (config.optional(outputMultiviewTransparencyPathFmt)) {
      // TODO(BK): The pruned view reconstruction needs to be extended to support transparency
      throw std::logic_error("Saving multiview (pruned) transparency maps is not yet implemented.");
    }
    if (const auto &node = config.optional(outputMultiviewGeometryPathFmt)) {
      const auto &depth = prunedViewsAndMasks.first[v].second;
      saveFrame(outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                        placeholders.contentId, placeholders.testId, v,
                                        depth.getWidth(), depth.getHeight()),
                depth, frameIndex);
    }
    if (const auto &node = config.optional(outputMultiviewOccupancyPathFmt)) {
      const auto &mask = prunedViewsAndMasks.second[v];
      saveFrame(outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                        placeholders.contentId, placeholders.testId, v,
                                        mask.getWidth(), mask.getHeight()),
                mask, frameIndex);
    }
  }
}

void saveSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                        std::int32_t foc, const MivBitstream::SequenceConfig &seqConfig) {
  if (const auto &node = config.optional(IO::outputSequenceConfigPathFmt)) {
    const auto path = config.require(IO::outputDirectory).as<std::filesystem::path>() /
                      fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                  placeholders.contentId, placeholders.testId, foc);
    std::ofstream stream{path};
    const auto json = Common::Json{seqConfig};
    json.saveTo(stream);
  }
}

auto outputBitstreamPath(const Common::Json &config, const Placeholders &placeholders)
    -> std::filesystem::path {
  auto path =
      config.require(IO::outputDirectory).as<std::filesystem::path>() /
      fmt::format(config.require(IO::outputBitstreamPathFmt).as<std::string>(),
                  placeholders.numberOfInputFrames, placeholders.contentId, placeholders.testId);
  create_directories(path.parent_path());
  return path;
}
} // namespace TMIV::IO
