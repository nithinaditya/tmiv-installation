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

#include <TMIV/Renderer/reprojectPoints.h>

#include <TMIV/Common/Thread.h>

namespace TMIV::Renderer {
AffineTransform::AffineTransform(const MivBitstream::CameraExtrinsics &source,
                                 const MivBitstream::CameraExtrinsics &target) {
  const auto r1 = Common::QuatD(source.rotation());
  const auto r2 = Common::QuatD(target.rotation());
  const auto t1 = Common::Vec3d(source.position());
  const auto t2 = Common::Vec3d(target.position());

  const auto r = conj(r2) * r1;
  const auto t = rotate(t1 - t2, conj(r2));

  m_R = Common::Mat3x3f(rotationMatrix(r));
  m_t = Common::Vec3f(t);
}

auto AffineTransform::operator()(Common::Vec3f x) const -> Common::Vec3f { return m_R * x + m_t; }

auto unprojectVertex(Common::Vec2f position, float depth, const MivBitstream::CameraIntrinsics &ci)
    -> Common::Vec3f {
  return ci.dispatch([&](auto camType) {
    Engine<camType> engine{ci};
    return engine.unprojectVertex(position, depth);
  });
}

auto projectVertex(const Common::Vec3f &position, const MivBitstream::CameraIntrinsics &ci)
    -> std::pair<Common::Vec2f, float> {
  return ci.dispatch([&](auto camType) {
    Engine<camType> engine{ci};
    auto imageVertexDescriptor = engine.projectVertex(SceneVertexDescriptor{position, 0.F});
    return std::pair{imageVertexDescriptor.position, imageVertexDescriptor.depth};
  });
}

ProjectionHelper::List::List(const MivBitstream::ViewParamsList &viewParamsList) {
  for (const auto &viewParams : viewParamsList) {
    this->emplace_back(viewParams);
  }
}

ProjectionHelper::ProjectionHelper(const MivBitstream::ViewParams &viewParams)
    : m_viewParams{viewParams}, m_rotation{viewParams.ce.rotation()} {
  switch (viewParams.ci.ci_cam_type()) {
  case MivBitstream::CiCamType::equirectangular:
    m_engine = std::make_unique<MetaEngine::Equirectangular>(viewParams.ci);
    break;
  case MivBitstream::CiCamType::perspective:
    m_engine = std::make_unique<MetaEngine::Perspective>(viewParams.ci);
    break;
  case MivBitstream::CiCamType::orthographic:
    m_engine = std::make_unique<MetaEngine::Orthographic>(viewParams.ci);
    break;
  default:
    abort();
  }
}

auto ProjectionHelper::getViewingDirection() const -> Common::Vec3f {
  return rotate(Common::Vec3f{1.F, 0.F, 0.F}, m_rotation);
}

auto ProjectionHelper::changeFrame(const Common::Vec3f &P) const -> Common::Vec3f {
  return rotate(P - m_viewParams.get().ce.position(), conj(m_rotation));
}

auto ProjectionHelper::doProjection(const Common::Vec3f &P) const
    -> std::pair<Common::Vec2f, float> {
  Common::Vec3f Q = changeFrame(P);
  auto imageVertexDescriptor = m_engine->projectVertex(SceneVertexDescriptor{Q, 0.F});
  return std::make_pair(imageVertexDescriptor.position, imageVertexDescriptor.depth);
}

auto ProjectionHelper::doUnprojection(const Common::Vec2f &p, float d) const -> Common::Vec3f {
  auto P = m_engine->unprojectVertex(p, d);
  return rotate(P, m_rotation) + m_viewParams.get().ce.position();
}

auto ProjectionHelper::isStrictlyInsideViewport(const Common::Vec2f &p) const -> bool {
  return ((0.5F <= p.x()) && (p.x() <= (m_viewParams.get().ci.projectionPlaneSize().x() - 0.5F))) &&
         ((0.5F <= p.y()) && (p.y() <= (m_viewParams.get().ci.projectionPlaneSize().y() - 0.5F)));
}

auto ProjectionHelper::isInsideViewport(const Common::Vec2f &p) const -> bool {
  return ((-0.5F <= p.x()) &&
          (p.x() <= (m_viewParams.get().ci.projectionPlaneSize().x() + 0.5F))) &&
         ((-0.5F <= p.y()) && (p.y() <= (m_viewParams.get().ci.projectionPlaneSize().y() + 0.5F)));
}

auto ProjectionHelper::isValidDepth(float d) const -> bool {
  static constexpr auto far = 999.999F;
  return (TMIV::Renderer::isValidDepth(d) &&
          (m_viewParams.get().dq.dq_norm_disp_low() <= (1.F / d)) && (d < far));
}

auto ProjectionHelper::getAngularResolution() const -> float {
  const auto &ci = m_viewParams.get().ci;

  switch (ci.ci_cam_type()) {
  case MivBitstream::CiCamType::equirectangular: {
    auto nbPixel = static_cast<float>(ci.projectionPlaneSize().x() * ci.projectionPlaneSize().y());
    float DT = ci.ci_erp_phi_max() - ci.ci_erp_phi_min();
    float DS = std::sin(ci.ci_erp_theta_max()) - std::sin(ci.ci_erp_theta_min());

    return nbPixel / (DS * DT);
  }
  case MivBitstream::CiCamType::perspective: {
    auto nbPixel = static_cast<float>(ci.projectionPlaneSize().x() * ci.projectionPlaneSize().y());
    const auto projectionFocalLength =
        (ci.ci_perspective_focal_hor() + ci.ci_perspective_focal_ver()) / 2.F;
    auto w = static_cast<float>(ci.projectionPlaneSize().x());
    auto h = static_cast<float>(ci.projectionPlaneSize().y());
    float omega =
        4.F * std::atan(nbPixel /
                        (2.F * projectionFocalLength *
                         std::sqrt(4.F * Common::sqr(projectionFocalLength) + (w * w + h * h))));

    return nbPixel / omega;
  }
  case MivBitstream::CiCamType::orthographic: {
    auto nbPixel = static_cast<float>(ci.projectionPlaneSize().x() * ci.projectionPlaneSize().y());
    return nbPixel / Common::hemiSphere;
  }
  default:
    abort();
  }
}

auto ProjectionHelper::getDepthRange() const -> Common::Vec2f {
  return {1.F / m_viewParams.get().dq.dq_norm_disp_high(),
          1.F / m_viewParams.get().dq.dq_norm_disp_low()};
}

auto ProjectionHelper::getRadialRange() const -> Common::Vec2f {
  switch (m_viewParams.get().ci.ci_cam_type()) {
  case MivBitstream::CiCamType::equirectangular: {
    return {1.F / m_viewParams.get().dq.dq_norm_disp_high(),
            1.F / m_viewParams.get().dq.dq_norm_disp_low()};
  }
  case MivBitstream::CiCamType::perspective: {
    const auto &ci = m_viewParams.get().ci;
    float x = (static_cast<float>(m_viewParams.get().ci.projectionPlaneSize().x()) -
               ci.ci_perspective_center_hor()) /
              ci.ci_perspective_focal_hor();
    float y = (static_cast<float>(m_viewParams.get().ci.projectionPlaneSize().y()) -
               ci.ci_perspective_center_ver()) /
              ci.ci_perspective_focal_ver();

    return {1.F / m_viewParams.get().dq.dq_norm_disp_high(),
            norm(Common::Vec3f{x, y, 1.F}) / m_viewParams.get().dq.dq_norm_disp_low()};
  }
  case MivBitstream::CiCamType::orthographic: {
    return {1.F / m_viewParams.get().dq.dq_norm_disp_high(),
            1.F / m_viewParams.get().dq.dq_norm_disp_low()};
  }
  default:
    abort();
  }
}

auto ProjectionHelper::getPointCloud(unsigned N) const -> PointCloud {
  PointCloud pointCloud;

  float step = 1.F / static_cast<float>(N - 1U);
  auto depthRange = getDepthRange();

  float x = 0.F;

  for (unsigned i = 0U; i < N; i++) {
    float y = 0.F;

    float px = x * static_cast<float>(m_viewParams.get().ci.projectionPlaneSize().x());

    for (unsigned j = 0U; j < N; j++) {
      float d = depthRange.x();

      float py = y * static_cast<float>(m_viewParams.get().ci.projectionPlaneSize().y());

      for (unsigned k = 0U; k < N; k++) {
        pointCloud.emplace_back(doUnprojection({px, py}, d));

        d += step * (depthRange.y() - depthRange.x());
      }

      y += step;
    }

    x += step;
  }

  return pointCloud;
}

auto getPointCloudList(const ProjectionHelperList &sourceHelperList, unsigned N) -> PointCloudList {
  PointCloudList pointCloudList;

  for (const auto &helper : sourceHelperList) {
    pointCloudList.emplace_back(helper.getPointCloud(N));
  }

  return pointCloudList;
}

auto getOverlapping(const ProjectionHelperList &sourceHelperList,
                    const PointCloudList &pointCloudList, std::size_t firstId, std::size_t secondId)
    -> float {
  std::size_t N = 0;

  const ProjectionHelper &secondHelper = sourceHelperList[secondId];
  const PointCloud &firstPointCloud = pointCloudList[firstId];

  for (const auto &P : firstPointCloud) {
    auto p = secondHelper.doProjection(P);

    if (isValidDepth(p.second) && secondHelper.isInsideViewport(p.first)) {
      N++;
    }
  }

  return static_cast<float>(N) / static_cast<float>(firstPointCloud.size());
}

auto computeOverlappingMatrix(const ProjectionHelperList &sourceHelperList) -> Common::Mat<float> {
  auto pointCloudList = getPointCloudList(sourceHelperList, 16);
  std::size_t K = sourceHelperList.size();
  Common::Mat<float> overlappingMatrix({K, K});

  for (std::size_t i = 0; i < K; i++) {
    for (std::size_t j = 0; j < K; j++) {
      if (i != j) {
        overlappingMatrix(i, j) = getOverlapping(sourceHelperList, pointCloudList, i, j);
      } else {
        overlappingMatrix(i, j) = 1.F;
      }
    }
  }

  return overlappingMatrix;
}

} // namespace TMIV::Renderer
