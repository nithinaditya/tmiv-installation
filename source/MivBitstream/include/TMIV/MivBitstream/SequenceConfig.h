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

#ifndef _TMIV_MIVBITSTREAM_SEQUENCECONFIG_H_
#define _TMIV_MIVBITSTREAM_SEQUENCECONFIG_H_

#include <TMIV/MivBitstream/ViewParamsList.h>

#include <TMIV/Common/Frame.h>

#include <variant>

namespace TMIV::MivBitstream {
struct CameraConfig {
  enum class Colorspace { yuv420 };

  ViewParams viewParams;
  int bitDepthColor{};
  int bitDepthTransparency{};
  int bitDepthDepth{};
  int bitDepthEntities{};
  Colorspace colorspace{Colorspace::yuv420};
  Colorspace transparencyColorspace{Colorspace::yuv420};
  Colorspace depthColorspace{Colorspace::yuv420};
  Colorspace entitiesColorspace{Colorspace::yuv420};

  auto textureVideoFormat() const -> std::string;
  auto transparencyVideoFormat() const -> std::string;
  auto geometryVideoFormat() const -> std::string;
  auto entitiesVideoFormat() const -> std::string;

  CameraConfig() = default;
  explicit CameraConfig(const Common::Json &config);

  explicit operator Common::Json() const;

  auto operator==(const CameraConfig &other) const noexcept -> bool;
  auto operator!=(const CameraConfig &other) const noexcept -> bool;
};

struct SequenceConfig {
  struct FrameRange {
    std::int32_t maxNumberOfFrames{};
    std::int32_t startFrame{};

    auto operator==(const FrameRange &other) const noexcept -> bool;
    auto operator!=(const FrameRange &other) const noexcept -> bool;
  };

  Common::Vec3d boundingBoxCenter;
  std::string contentName;
  double frameRate{};
  int numberOfFrames{};
  std::vector<CameraConfig> cameras;
  std::vector<std::string> sourceCameraNames;
  std::vector<FrameRange> frameRanges;
  bool lengthsInMeters{true};

  SequenceConfig() = default;
  explicit SequenceConfig(const Common::Json &config);
  explicit SequenceConfig(std::istream &stream);

  explicit operator Common::Json() const;

  [[nodiscard]] auto cameraByName(const std::string &name) const -> CameraConfig;
  [[nodiscard]] auto sourceViewParams() const -> ViewParamsList;
  [[nodiscard]] auto startFrameGiven(std::int32_t numberOfInputFrames) const -> std::int32_t;

  auto operator==(const SequenceConfig &other) const noexcept -> bool;
  auto operator!=(const SequenceConfig &other) const noexcept -> bool;
};
} // namespace TMIV::MivBitstream

#endif
