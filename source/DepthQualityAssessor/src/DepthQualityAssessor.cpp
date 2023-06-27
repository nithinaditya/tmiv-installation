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

#include <TMIV/Common/Thread.h>
#include <TMIV/DepthQualityAssessor/DepthQualityAssessor.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/MivBitstream/EncoderParams.h>
#include <TMIV/Renderer/reprojectPoints.h>
#include <iostream>

namespace TMIV::DepthQualityAssessor {
namespace {
template <typename MAT>
auto textureNeighbourhood(const MAT &m, const Common::Vec2f &p)
    -> std::vector<typename MAT::value_type> {
  std::vector<typename MAT::value_type> fetchedValues;

  const int N = 1;

  int w_last = static_cast<int>(m.width()) - 1;
  int h_last = static_cast<int>(m.height()) - 1;

  int xc = std::clamp(static_cast<int>(std::floor(p.x() + 0.5F)), 0, w_last);
  int yc = std::clamp(static_cast<int>(std::floor(p.y() + 0.5F)), 0, h_last);

  int x0 = std::max(0, xc - N);
  int y0 = std::max(0, yc - N);

  int x1 = std::min(xc + N, w_last);
  int y1 = std::min(yc + N, h_last);

  fetchedValues.reserve((x1 - x0 + 1) * (y1 - y0 + 1));

  for (int y = y0; y <= y1; y++) {
    for (int x = x0; x <= x1; x++) {
      fetchedValues.emplace_back(m(y, x));
    }
  }

  return fetchedValues;
}

auto isLowQuality(float blendingFactor, float maxOutlierRatio,
                  const std::vector<Common::Mat<Common::Vec3f>> &sourceUnprojectionList,
                  const Renderer::ProjectionHelper &firstHelper,
                  const Common::Mat<float> &firstDepth, std::size_t secondId) -> bool {
  const auto &secondUnprojection = sourceUnprojectionList[secondId];
  std::atomic<size_t> outliers = 0U;

  Common::parallel_for(
      secondUnprojection.width(), secondUnprojection.height(), [&](size_t y, size_t x) {
        const auto &P = secondUnprojection(y, x);

        if (!std::isnan(P.x())) {
          auto p = firstHelper.doProjection(P);

          if (firstHelper.isValidDepth(p.second) && firstHelper.isStrictlyInsideViewport(p.first)) {
            auto zOnFirst = textureNeighbourhood(firstDepth, p.first);

            if (std::all_of(zOnFirst.begin(), zOnFirst.end(), [&](float z) {
                  return (!std::isnan(z) && (p.second < z * (1.F - blendingFactor)));
                })) {
              outliers++;
            }
          }
        }
      });

  float outlierRatio = static_cast<float>(outliers) /
                       static_cast<float>(secondUnprojection.width() * secondUnprojection.height());

  if (maxOutlierRatio < outlierRatio) {
    std::cout << "DepthQualityAssessor -> Threshold exceeded (" << outlierRatio * 100.F << "%)"
              << std::endl;
    return true;
  }

  return false;
}

auto isLowDepthQuality(const MivBitstream::ViewParamsList &vpl,
                       const Common::MVD16Frame &sourceViews, float blendingFactor,
                       float maxOutlierRatio) -> bool {
  const auto sourceHelperList = Renderer::ProjectionHelperList{vpl};

  // Expand depth
  std::vector<Common::Mat<float>> sourceDepthExpandedList;
  std::vector<Common::Mat<Common::Vec3f>> sourceUnprojectionList;

  sourceDepthExpandedList.reserve(sourceHelperList.size());
  sourceUnprojectionList.reserve(sourceHelperList.size());

  for (size_t viewId = 0; viewId < sourceHelperList.size(); viewId++) {
    const auto &sourceHelper = sourceHelperList[viewId];
    const auto occupancyTransform = MivBitstream::OccupancyTransform{sourceHelper.getViewParams()};

    auto sourceDepthExpanded =
        MivBitstream::DepthTransform{sourceHelper.getViewParams().dq, 16}.expandDepth(
            sourceViews[viewId].depth);

    std::transform(sourceViews[viewId].depth.getPlane(0).begin(),
                   sourceViews[viewId].depth.getPlane(0).end(), sourceDepthExpanded.begin(),
                   sourceDepthExpanded.begin(), [&](uint16_t normDisp, float depthValue) {
                     return occupancyTransform.occupant(normDisp) ? depthValue : NAN;
                   });

    Common::Mat<Common::Vec3f> sourceUnprojection(
        {sourceDepthExpanded.height(), sourceDepthExpanded.width()});

    Common::parallel_for(
        sourceUnprojection.width(), sourceUnprojection.height(), [&](size_t y, size_t x) {
          float z = sourceDepthExpanded(y, x);
          if (sourceHelper.isValidDepth(z)) {
            sourceUnprojection(y, x) = sourceHelper.doUnprojection(
                Common::Vec2f({static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F}), z);
          } else {
            sourceUnprojection(y, x) = Common::Vec3f{NAN, NAN, NAN};
          }
        });

    sourceDepthExpandedList.emplace_back(std::move(sourceDepthExpanded));
    sourceUnprojectionList.emplace_back(std::move(sourceUnprojection));
  }

  // Reprojection for outlier detection
  for (size_t firstId = 0; firstId < sourceHelperList.size(); firstId++) {
    const auto &firstHelper = sourceHelperList[firstId];
    const auto &firstDepth = sourceDepthExpandedList[firstId];

    for (size_t secondId = 0; secondId < sourceHelperList.size(); secondId++) {
      if (firstId != secondId) {
        if (isLowQuality(blendingFactor, maxOutlierRatio, sourceUnprojectionList, firstHelper,
                         firstDepth, secondId)) {
          return true;
        }
      }
    }

    std::cout << "DepthQualityAssessor -> View #" << firstId << " done !" << std::endl;
  }

  std::cout << "DepthQualityAssessor -> OK" << std::endl;

  return false;
}
} // namespace

DepthQualityAssessor::DepthQualityAssessor(const Common::Json & /*unused*/,
                                           const Common::Json &componentNode) {
  m_blendingFactor = componentNode.require("blendingFactor").as<float>();
  m_maxOutlierRatio = componentNode.require("maxOutlierRatio").as<float>();
}

auto DepthQualityAssessor::isLowDepthQuality(const MivBitstream::ViewParamsList &vpl,
                                             const Common::MVD16Frame &sourceViews) -> bool {
  return TMIV::DepthQualityAssessor::isLowDepthQuality(vpl, sourceViews, m_blendingFactor,
                                                       m_maxOutlierRatio);
}
} // namespace TMIV::DepthQualityAssessor
