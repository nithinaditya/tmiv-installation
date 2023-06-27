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

#ifndef _TMIV_RENDERER_ENGINE_H_
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/Common.h>

namespace TMIV::Renderer {
template <> struct Engine<MivBitstream::CiCamType::perspective> {
  const float f_x;
  const float f_y;
  const float c_x;
  const float c_y;

  explicit Engine(const MivBitstream::CameraIntrinsics &ci)
      : f_x{ci.ci_perspective_focal_hor()}
      , f_y{ci.ci_perspective_focal_ver()}
      , c_x{ci.ci_perspective_center_hor()}
      , c_y{ci.ci_perspective_center_ver()} {}

  // Unprojection equation
  [[nodiscard]] auto unprojectVertex(Common::Vec2f uv, float depth) const -> Common::Vec3f {
    if (depth > 0.F) {
      return {depth, -(depth / f_x) * (uv.x() - c_x), -(depth / f_y) * (uv.y() - c_y)};
    }
    return {NAN, NAN, NAN};
  }

  // Projection equation
  [[nodiscard]] auto projectVertex(const SceneVertexDescriptor &v) const
      -> ImageVertexDescriptor const {
    if (v.position.x() > 0.F) {
      auto uv = Common::Vec2f{-f_x * v.position.y() / v.position.x() + c_x,
                              -f_y * v.position.z() / v.position.x() + c_y};
      return {uv, v.position.x(), v.rayAngle};
    }
    return {{NAN, NAN}, NAN, NAN};
  }

  // Project mesh to target view
  template <typename... T>
  auto project(const SceneVertexDescriptorList &sceneVertices,
               const TriangleDescriptorList &triangles, std::tuple<std::vector<T>...> attributes) {
    ImageVertexDescriptorList imageVertices;
    imageVertices.reserve(sceneVertices.size());
    for (const SceneVertexDescriptor &v : sceneVertices) {
      imageVertices.push_back(projectVertex(v));
    }
    return std::tuple{std::move(imageVertices), triangles, attributes};
  }
};
} // namespace TMIV::Renderer
