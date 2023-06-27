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

#ifndef _TMIV_IO_IO_H_
#define _TMIV_IO_IO_H_

#include <TMIV/Common/Frame.h>
#include <TMIV/Common/Json.h>
#include <TMIV/MivBitstream/AccessUnit.h>

using namespace std::string_literals;

namespace TMIV::IO {
extern const std::string configDirectory;
extern const std::string inputBitstreamPathFmt;
extern const std::string inputCameraNames;
extern const std::string inputDirectory;
extern const std::string inputEntityPathFmt;
extern const std::string inputGeometryPathFmt;
extern const std::string inputGeometryVideoFramePathFmt;
extern const std::string inputGeometryVsbPathFmt;
extern const std::string inputMaterialIdVsbPathFmt;
extern const std::string inputNormalVsbPathFmt;
extern const std::string inputOccupancyVideoFramePathFmt;
extern const std::string inputOccupancyVsbPathFmt;
extern const std::string inputPoseTracePathFmt;
extern const std::string inputReflectanceVsbPathFmt;
extern const std::string inputSequenceConfigPathFmt;
extern const std::string inputTexturePathFmt;
extern const std::string inputTextureVideoFramePathFmt;
extern const std::string inputTextureVsbPathFmt;
extern const std::string inputTransparencyPathFmt;
extern const std::string inputTransparencyVideoFramePathFmt;
extern const std::string inputTransparencyVsbPathFmt;
extern const std::string inputViewportParamsPathFmt;

extern const std::string outputBitstreamPathFmt;
extern const std::string outputBlockToPatchMapPathFmt;
extern const std::string outputDirectory;
extern const std::string outputGeometryVideoDataPathFmt;
extern const std::string outputMultiviewGeometryPathFmt;
extern const std::string outputMultiviewOccupancyPathFmt;
extern const std::string outputMultiviewTexturePathFmt;
extern const std::string outputMultiviewTransparencyPathFmt;
extern const std::string outputOccupancyVideoDataPathFmt;
extern const std::string outputSequenceConfigPathFmt;
extern const std::string outputTextureVideoDataPathFmt;
extern const std::string outputTransparencyVideoDataPathFmt;
extern const std::string outputViewportGeometryPathFmt;
extern const std::string outputViewportTexturePathFmt;

struct Placeholders {
  std::string contentId{};    // e.g. A
  std::string testId{"R0"};   // e.g. QP3 or R0
  int numberOfInputFrames{};  // e.g. 97
  int numberOfOutputFrames{}; // e.g. 300
};

template <typename FORMAT>
auto loadFrame(const std::filesystem::path &path, std::int32_t frameIndex, Common::Vec2i frameSize)
    -> Common::Frame<FORMAT>;
auto loadMultiviewFrame(const Common::Json &config, const Placeholders &placeholders,
                        const MivBitstream::SequenceConfig &sc, std::int32_t frameIndex)
    -> Common::MVD16Frame;
auto loadViewportMetadata(const Common::Json &config, const Placeholders &placeholders,
                          std::int32_t frameIndex, const std::string &cameraName, bool isPoseTrace)
    -> MivBitstream::ViewParams;
auto loadOccupancyVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                             MivBitstream::AtlasId atlasId, uint32_t frameId,
                             Common::Vec2i frameSize) -> Common::Occupancy10Frame;
auto loadGeometryVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                            MivBitstream::AtlasId atlasId, uint32_t frameId,
                            Common::Vec2i frameSize) -> Common::Depth10Frame;
auto loadTextureVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                           MivBitstream::AtlasId atlasId, uint32_t frameId, Common::Vec2i frameSize)
    -> Common::Texture444Frame;
auto loadTransparencyVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                                MivBitstream::AtlasId atlasId, uint32_t frameId,
                                Common::Vec2i frameSize) -> Common::Transparency10Frame;
auto loadSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                        std::int32_t frameIndex) -> MivBitstream::SequenceConfig;
auto tryLoadSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                           std::int32_t frameIndex) -> std::optional<MivBitstream::SequenceConfig>;

auto loadMpiTextureMpiLayer(const Common::Json &config, const Placeholders &placeholders,
                            const MivBitstream::SequenceConfig &sc, std::int32_t frameIndex,
                            int mpiLayerIndex, int nbMpiLayers) -> Common::TextureFrame;

auto loadMpiTransparencyMpiLayer(const Common::Json &config, const Placeholders &placeholders,
                                 const MivBitstream::SequenceConfig &sc, std::int32_t frameIndex,
                                 int mpiLayerIndex, int nbMpiLayers) -> Common::Transparency10Frame;

auto inputBitstreamPath(const Common::Json &config, const Placeholders &placeholders)
    -> std::filesystem::path;
auto inputSubBitstreamPath(const std::string &key, const Common::Json &config,
                           const Placeholders &placeholders, MivBitstream::AtlasId atlasId,
                           int attributeIdx) -> std::filesystem::path;

template <typename FORMAT>
void saveFrame(const std::filesystem::path &path, const Common::Frame<FORMAT> &frame,
               std::int32_t frameIndex);
void saveAtlasFrame(const Common::Json &config, const Placeholders &placeholders,
                    std::int32_t frameIndex, const Common::MVD10Frame &frame);
void saveViewport(const Common::Json &config, const Placeholders &placeholders,
                  std::int32_t frameIndex, const std::string &name,
                  const Common::TextureDepth16Frame &frame);
void saveBlockToPatchMaps(const Common::Json &config, const Placeholders &placeholders,
                          std::int32_t frameIndex, const MivBitstream::AccessUnit &frame);
void savePrunedFrame(const Common::Json &config, const Placeholders &placeholders,
                     std::int32_t frameIndex,
                     const std::pair<std::vector<Common::Texture444Depth10Frame>, Common::MaskList>
                         &prunedViewsAndMasks);
void saveSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                        std::int32_t foc, const MivBitstream::SequenceConfig &seqConfig);

// Construct the output bitstream path and create the parent directories
auto outputBitstreamPath(const Common::Json &config, const Placeholders &placeholders)
    -> std::filesystem::path;
} // namespace TMIV::IO

#endif
