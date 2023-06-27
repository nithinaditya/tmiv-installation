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

#include <TMIV/Renderer/Front/mapInputToOutputFrames.h>

#include <stdexcept>

namespace TMIV::Renderer::Front {
namespace {
// Returns a frame index. If frameIndex is strictly less than the actual number of frames in the
// encoded stream, then regular values are returned else mirrored indices are computed.
auto getExtendedIndex(std::int32_t outputFrameIndex, std::int32_t numberOfInputFrames)
    -> std::int32_t {
  if (numberOfInputFrames <= 0) {
    throw std::runtime_error("Cannot extend frame index with zero input frames");
  }
  const auto frameGroupIndex = outputFrameIndex / numberOfInputFrames;
  const auto frameRelativeIndex = outputFrameIndex % numberOfInputFrames;
  return frameGroupIndex % 2 != 0 ? numberOfInputFrames - frameRelativeIndex - 1
                                  : frameRelativeIndex;
}
} // namespace

// TODO(BK): Lock behavior with unit tests
auto mapInputToOutputFrames(std::int32_t numberOfInputFrames, std::int32_t numberOfOutputFrames)
    -> FrameMapping {
  auto x = FrameMapping{};

  if (numberOfInputFrames < 0) {
    throw std::runtime_error("Negative number of input frames");
  }
  if (numberOfOutputFrames < 0) {
    throw std::runtime_error("Negative number of output frames");
  }

  for (std::int32_t outputFrame = 0; outputFrame < numberOfOutputFrames; ++outputFrame) {
    const auto inputFrame = getExtendedIndex(outputFrame, numberOfInputFrames);
    x.emplace(inputFrame, outputFrame);
  }

  return x;
}
} // namespace TMIV::Renderer::Front
