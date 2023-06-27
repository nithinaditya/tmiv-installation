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

#ifndef _TMIV_RENDERER_REPROJECTPOINTS_H_
#define _TMIV_RENDERER_REPROJECTPOINTS_H_

#include <TMIV/Common/Quaternion.h>
#include <TMIV/MivBitstream/EncoderParams.h>
#include <TMIV/Renderer/Engine.h>

namespace TMIV::Renderer {
// Change the reference frame from a source camera to a target camera
//
// This corresponds to the affine transformation: x -> Rx + t with rotation matrix R and translation
// vector t.
class AffineTransform {
public:
  AffineTransform(const MivBitstream::CameraExtrinsics &source,
                  const MivBitstream::CameraExtrinsics &target);

  [[nodiscard]] auto translation() const -> auto & { return m_t; }
  auto operator()(Common::Vec3f x) const -> Common::Vec3f;

private:
  Common::Mat3x3f m_R;
  Common::Vec3f m_t;
};

// Unproject a pixel from a source frame to scene coordinates in the reference
// frame of the target camera.
//
// This method is less efficient because of the switch on projection type, but
// suitable for rendering directly from an atlas.
auto unprojectVertex(Common::Vec2f position, float depth, const MivBitstream::CameraIntrinsics &ci)
    -> Common::Vec3f;

// Project point: From world position (with the camera as reference frame)
// to image position
//
// This method is less efficient because of the switch on projection type, but
// suitable for rendering directly from an atlas.
auto projectVertex(const Common::Vec3f &position, const MivBitstream::CameraIntrinsics &ci)
    -> std::pair<Common::Vec2f, float>;

inline auto isValidDepth(float d) -> bool { return (0.F < d); }

using PointCloud = std::vector<Common::Vec3f>;
using PointCloudList = std::vector<PointCloud>;

namespace MetaEngine {
class Base {
public:
  Base() = default;
  virtual ~Base() = default;
  Base(const Base &) = default;
  Base(Base &&) = default;
  auto operator=(Base &&) -> Base & = default;
  auto operator=(const Base &) -> Base & = default;
  [[nodiscard]] virtual auto unprojectVertex(Common::Vec2f uv, float depth) const
      -> Common::Vec3f = 0;
  [[nodiscard]] virtual auto projectVertex(const SceneVertexDescriptor &v) const
      -> ImageVertexDescriptor const = 0;
};

template <MivBitstream::CiCamType camType> class Variant : public Base, public Engine<camType> {
public:
  using engine_type = Engine<camType>;

public:
  using engine_type::Engine;
  using engine_type::operator=;
  [[nodiscard]] auto unprojectVertex(Common::Vec2f uv, float depth) const
      -> Common::Vec3f override {
    return engine_type::unprojectVertex(uv, depth);
  }
  [[nodiscard]] auto projectVertex(const SceneVertexDescriptor &v) const
      -> ImageVertexDescriptor const override {
    return engine_type::projectVertex(v);
  }
};

using Perspective = Variant<MivBitstream::CiCamType::perspective>;
using Equirectangular = Variant<MivBitstream::CiCamType::equirectangular>;
using Orthographic = Variant<MivBitstream::CiCamType::orthographic>;

} // namespace MetaEngine

class ProjectionHelper {
public:
  class List : public std::vector<ProjectionHelper> {
  public:
    List(const MivBitstream::ViewParamsList &viewParamsList);
    List(const List &) = default;
    List(List &&) = default;
    auto operator=(const List &) -> List & = default;
    auto operator=(List &&) -> List & = default;
  };

private:
  std::reference_wrapper<const MivBitstream::ViewParams> m_viewParams;
  std::unique_ptr<MetaEngine::Base> m_engine;
  Common::QuatF m_rotation;

public:
  ProjectionHelper(const MivBitstream::ViewParams &viewParams);
  ProjectionHelper(const ProjectionHelper &) = delete;
  ProjectionHelper(ProjectionHelper &&) = default;
  auto operator=(const ProjectionHelper &) -> ProjectionHelper & = delete;
  auto operator=(ProjectionHelper &&) -> ProjectionHelper & = delete;
  [[nodiscard]] auto getViewParams() const -> const MivBitstream::ViewParams & {
    return m_viewParams.get();
  }
  [[nodiscard]] auto getViewingPosition() const -> Common::Vec3f {
    return m_viewParams.get().ce.position();
  }
  [[nodiscard]] auto getViewingDirection() const -> Common::Vec3f;
  [[nodiscard]] auto changeFrame(const Common::Vec3f &P) const -> Common::Vec3f;
  [[nodiscard]] auto doProjection(const Common::Vec3f &P) const -> std::pair<Common::Vec2f, float>;
  [[nodiscard]] auto doUnprojection(const Common::Vec2f &p, float d) const -> Common::Vec3f;
  [[nodiscard]] auto isStrictlyInsideViewport(const Common::Vec2f &p) const -> bool;
  [[nodiscard]] auto isInsideViewport(const Common::Vec2f &p) const -> bool;
  [[nodiscard]] auto isValidDepth(float d) const -> bool;
  [[nodiscard]] auto getAngularResolution() const -> float;
  [[nodiscard]] auto getDepthRange() const -> Common::Vec2f;
  [[nodiscard]] auto getRadialRange() const -> Common::Vec2f;
  [[nodiscard]] auto getPointCloud(unsigned N = 8) const -> PointCloud;
};

using ProjectionHelperList = typename ProjectionHelper::List;

auto getPointCloudList(const ProjectionHelperList &sourceHelperList, unsigned N = 16)
    -> PointCloudList;

auto getOverlapping(const ProjectionHelperList &sourceHelperList,
                    const PointCloudList &pointCloudList, std::size_t firstId, std::size_t secondId)
    -> float;

auto computeOverlappingMatrix(const ProjectionHelperList &sourceHelperList) -> Common::Mat<float>;

} // namespace TMIV::Renderer

#endif
