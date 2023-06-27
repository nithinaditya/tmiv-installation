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

#include <TMIV/ViewOptimizer/BasicViewAllocator.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/LinAlg.h>

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include <numeric>

namespace TMIV::ViewOptimizer {
namespace {
[[noreturn]] void reportError(const char *what, int line) noexcept {
  std::cerr << "Error in BasicViewAllocator at line " << line << ": " << what << '\n';
  abort();

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define VERIFY(condition)                                                                          \
  static_cast<void>((!!(condition) || (reportError(#condition, __LINE__), false)))
}
} // namespace

BasicViewAllocator::BasicViewAllocator(const Common::Json &rootNode,
                                       const Common::Json &componentNode)
    : AbstractViewSelector{rootNode, componentNode}
    , m_numGroups{rootNode.require("numGroups").as<int>()}
    , m_maxLumaPictureSize{rootNode.require("maxLumaPictureSize").as<int>()}
    , m_maxAtlases{rootNode.require("maxAtlases").as<int>()}
    , m_minNonCodedViews{componentNode.require("minNonCodedViews").as<int>()}
    , m_maxBasicViewFraction{componentNode.require("maxBasicViewFraction").as<double>()} {
  VERIFY(m_numGroups <= m_maxAtlases);
  VERIFY(0 < m_maxLumaPictureSize);
  VERIFY(0 <= m_minNonCodedViews);
  VERIFY(0. < m_maxBasicViewFraction && m_maxBasicViewFraction <= 1.);
}

auto BasicViewAllocator::isBasicView() const -> std::vector<bool> {
  const auto viewCount = params().viewParamsList.size();
  const auto count = basicViewCount();
  VERIFY(0 < count && count <= viewCount);

  const auto positions = viewPositions();
  const auto cost = KMedoidsCost{sqDistanceMatrix(positions)};
  const auto first = forwardView(positions);

  auto centroids = selectInitialCentroids(cost, first, count);
  std::cout << "Initial centroids:";
  for (auto i : centroids) {
    std::cout << ' ' << params().viewParamsList[i].name;
  }
  std::cout << " (cost: " << cost(centroids) << " m^-2)\n";

  while (auto update = updateCentroids(cost, centroids)) {
    std::swap(*update, centroids);
    std::cout << "Updated centroids:";
    for (auto i : centroids) {
      std::cout << ' ' << params().viewParamsList[i].name;
    }
    std::cout << " (cost: " << cost(centroids) << " m^-2)\n";
  }

  auto result = std::vector<bool>(cost.N(), false);
  for (auto i : centroids) {
    result[i] = true;
  }
  return result;
}

auto BasicViewAllocator::basicViewCount() const -> size_t {
  const auto numAtlases = m_maxAtlases / std::max(1, m_numGroups);
  const auto maxSamples =
      static_cast<size_t>(m_maxBasicViewFraction * numAtlases * m_maxLumaPictureSize);

  size_t count = 0;
  size_t samplesInTotal = 0;
  size_t samplesInAtlas0 = 0;

  for (auto samplesInView : lumaSamplesPerSourceViewSortedDesc()) {
    samplesInTotal += samplesInView;
    if (samplesInTotal > maxSamples) {
      std::cout << "Basic view count is limited by maximum basic view fraction.\n";
      break;
    }

    if (count % numAtlases == 0) {
      samplesInAtlas0 += samplesInView;
      if (samplesInAtlas0 > static_cast<size_t>(m_maxLumaPictureSize)) {
        std::cout << "Basic view count is limited by maximum luma picture size.\n";
        break;
      }
    }

    if (++count + m_minNonCodedViews >= params().viewParamsList.size()) {
      std::cout << "Basic view count is limited by minimum non-coded view count.\n";
      break;
    }
  }

  VERIFY(0 < count && count <= params().viewParamsList.size());
  return count;
}

auto BasicViewAllocator::lumaSamplesPerSourceViewSortedDesc() const -> std::vector<std::size_t> {
  auto result = std::vector<std::size_t>{};
  result.reserve(params().viewParamsList.size());
  std::transform(params().viewParamsList.cbegin(), params().viewParamsList.cend(),
                 std::back_inserter(result), [](const MivBitstream::ViewParams &vp) {
                   return (vp.ci.ci_projection_plane_width_minus1() + 1) *
                          (vp.ci.ci_projection_plane_height_minus1() + 1);
                 });
  std::sort(result.begin(), result.end(), std::greater<>());
  return result;
}

auto BasicViewAllocator::viewPositions() const -> std::vector<Common::Vec3d> {
  auto result = std::vector<Common::Vec3d>(params().viewParamsList.size());
  std::transform(params().viewParamsList.cbegin(), params().viewParamsList.cend(), result.begin(),
                 [](const MivBitstream::ViewParams &vp) { return vp.ce.position(); });
  return result;
}

auto BasicViewAllocator::forwardView(const Positions &pos) const -> std::size_t {
  const auto N = pos.size();
  auto result = std::vector<double>(N, 1.);

  const auto lessX = [](const auto &p1, const auto &p2) { return p1.x() < p2.x(); };
  const auto maxX = std::max_element(pos.cbegin(), pos.cend(), lessX)->x();
  const auto sumPos = std::accumulate(pos.cbegin(), pos.cend(), Common::Vec3d{},
                                      [](const auto &p1, const auto &p2) { return p1 + p2; });
  const auto target =
      Common::Vec3d{maxX, sumPos.y() / static_cast<double>(N), sumPos.z() / static_cast<double>(N)};

  auto dist2 = std::vector<double>(N);
  std::transform(pos.cbegin(), pos.cend(), dist2.begin(),
                 [&target](const auto &p) { return Common::norm2(p - target); });
  const auto nearest = std::min_element(dist2.cbegin(), dist2.cend());
  const auto index = static_cast<size_t>(nearest - dist2.cbegin());
  std::cout << "Forward central view is " << params().viewParamsList[index].name << ".\n";

  return index;
}

auto BasicViewAllocator::sqDistanceMatrix(const Positions &pos) -> Common::Mat<double> {
  const auto N = pos.size();
  auto result = Common::Mat<double>({N, N});
  for (size_t i = 0; i < N; ++i) {
    for (size_t j = 0; j < N; ++j) {
      result(i, j) = Common::norm2(pos[i] - pos[j]);
    }
  }
  return result;
}

auto BasicViewAllocator::selectInitialCentroids(const KMedoidsCost &cost, std::size_t first,
                                                std::size_t k) -> Centroids {
  auto result = Centroids{};
  result.reserve(cost.N());
  VERIFY(k <= cost.N());

  result.push_back(first);

  while (result.size() < k) {
    auto lowestCost = std::numeric_limits<double>::infinity();
    auto newCentroid = std::optional<std::size_t>{};

    for (size_t i = 0; i < cost.N(); ++i) {
      if (!Common::contains(result, i)) {
        result.push_back(i);
        auto value = cost(result);
        result.pop_back();

        if (value < lowestCost) {
          lowestCost = value;
          newCentroid = i;
        }
      }
    }

    VERIFY(newCentroid);
    result.push_back(*newCentroid);
  }

  VERIFY(result.size() == k);
  return result;
}

auto BasicViewAllocator::updateCentroids(const KMedoidsCost &cost, Centroids centroids)
    -> std::optional<Centroids> {
  auto lowestCost = cost(centroids);
  auto update = std::optional<Centroids>{};

  // For each (centroid, non-centroid) pair
  for (auto &i : centroids) {
    for (size_t j = 0; j < cost.N(); ++j) {
      if (!Common::contains(centroids, j)) {
        std::swap(i, j);
        const auto value = cost(centroids);
        if (value < lowestCost) {
          lowestCost = value;
          update = centroids;
        }
        std::swap(i, j);
      }
    }
  }

  return update;
}
} // namespace TMIV::ViewOptimizer
