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

#ifndef _TMIV_RENDERER_FRONT_RENDERMULTIPLEFRAMES_H_
#define _TMIV_RENDERER_FRONT_RENDERMULTIPLEFRAMES_H_

#include <TMIV/IO/IO.h>
#include <TMIV/MivBitstream/AccessUnit.h>
#include <TMIV/Renderer/Front/mapInputToOutputFrames.h>
#include <TMIV/Renderer/ICuller.h>
#include <TMIV/Renderer/IRenderer.h>

namespace TMIV::Renderer::Front {
class MultipleFrameRenderer {
public:
  MultipleFrameRenderer(const Common::Json &rootNode,
                        const std::vector<std::string> &outputCameraNames,
                        const std::vector<std::string> &outputPoseTraceNames,
                        IO::Placeholders placeholders);

  void renderMultipleFrames(const MivBitstream::AccessUnit &frame,
                            FrameMapping::const_iterator first,
                            FrameMapping::const_iterator last) const;

  [[nodiscard]] auto isOptimizedForRestrictedGeometry() const -> bool {
    // NOTe(FT): added to handle the absence of renderer in the G3 anchor type
    if (m_renderer) {
      return m_renderer->isOptimizedForRestrictedGeometry();
    }
    return false;
  }

private:
  void renderFrame(MivBitstream::AccessUnit frame, std::int32_t outputFrameIndex,
                   const std::string &cameraName, bool isPoseTrace) const;

  const Common::Json &m_config;
  const std::vector<std::string> &m_outputCameraNames;
  const std::vector<std::string> &m_outputPoseTraceNames;
  IO::Placeholders m_placeholders;
  std::unique_ptr<Renderer::ICuller> m_culler;
  std::unique_ptr<Renderer::IRenderer> m_renderer;
};
} // namespace TMIV::Renderer::Front

#endif