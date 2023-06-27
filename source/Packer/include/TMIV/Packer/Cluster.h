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

#ifndef _TMIV_ATLASCONSTRUCTOR_CLUSTER_H_
#define _TMIV_ATLASCONSTRUCTOR_CLUSTER_H_

#include <TMIV/Common/Frame.h>

#include <queue>
#include <tuple>

namespace TMIV::Packer {
using ClusteringMap = Common::Frame<Common::YUV400P16>;
using ClusteringMapList = std::vector<ClusteringMap>;

class Cluster;
using ClusterList = std::vector<Cluster>;

class Cluster {
public:
  Cluster() = default;
  Cluster(int viewId, bool isBasicView, int clusterId, int entityId);
  Cluster(const Cluster &) = default;
  Cluster(Cluster &&) = default;
  auto operator=(const Cluster &) -> Cluster & = default;
  auto operator=(Cluster &&) -> Cluster & = default;
  ~Cluster() = default;

  void push(int i, int j);
  [[nodiscard]] auto getViewId() const -> int { return viewId_; }
  [[nodiscard]] auto getClusterId() const -> int { return clusterId_; }
  [[nodiscard]] auto getEntityId() const -> int { return entityId_; }
  [[nodiscard]] auto getNumActivePixels() const -> int { return numActivePixels_; }
  [[nodiscard]] auto imin() const -> int { return imin_; }
  [[nodiscard]] auto jmin() const -> int { return jmin_; }
  [[nodiscard]] auto imax() const -> int { return imax_; }
  [[nodiscard]] auto jmax() const -> int { return jmax_; }
  [[nodiscard]] auto getFilling() const -> int { return filling_; }
  [[nodiscard]] auto width() const -> int { return (jmax_ - jmin_ + 1); }
  [[nodiscard]] auto height() const -> int { return (imax_ - imin_ + 1); }
  [[nodiscard]] auto getArea() const -> int { return width() * height(); }
  [[nodiscard]] auto getMinSize() const -> int { return std::min(width(), height()); }
  [[nodiscard]] auto split(const ClusteringMap &clusteringMap, int overlap) const
      -> std::pair<Cluster, Cluster>;
  [[nodiscard]] constexpr auto isBasicView() const noexcept { return m_isBasicView; }

  void recursiveSplit(const ClusteringMap &clusteringMap, std::vector<Cluster> &out, int alignment,
                      int minPatchSize) const;

  static auto Empty() -> Cluster {
    Cluster out;
    out.imin_ = 0;
    out.imax_ = 0;
    out.jmin_ = 0;
    out.jmax_ = 0;
    return out;
  }
  static auto setEntityId(Cluster &c, int entityId) -> Cluster;
  static auto align(const Cluster &c, int alignment) -> Cluster;
  static auto merge(const Cluster &c1, const Cluster &c2) -> Cluster;
  static auto retrieve(int viewId, const Common::Mask &maskMap, int firstClusterId,
                       bool isBasicView, bool enableMerging)
      -> std::pair<ClusterList, ClusteringMap>;

private:
  auto splitLPatchVertically(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                             int alignment, int minPatchSize,
                             const std::array<std::deque<int>, 2> &min_h_agg,
                             const std::array<std::deque<int>, 2> &max_h_agg) const -> bool;
  auto splitLPatchHorizontally(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                               int alignment, int minPatchSize,
                               const std::array<std::deque<int>, 2> &min_w_agg,
                               const std::array<std::deque<int>, 2> &max_w_agg) const -> bool;
  auto splitCPatchVertically(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                             int alignment, int minPatchSize) const -> bool;
  auto splitCPatchHorizontally(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                               int alignment, int minPatchSize) const -> bool;
  [[nodiscard]] auto createAggregatedQueues(const ClusteringMap &clusteringMap,
                                            bool aggregateHorizontally) const
      -> std::tuple<std::array<std::deque<int>, 2>, std::array<std::deque<int>, 2>>;
  [[nodiscard]] auto computeMinAndMaxVectors(const ClusteringMap &clusteringMap,
                                             bool aggregateHorizontally) const
      -> std::tuple<std::vector<int>, std::vector<int>>;

  int viewId_ = 0;
  bool m_isBasicView{};
  int clusterId_ = 0;
  int entityId_ = 0;
  int numActivePixels_ = 0;
  int imin_ = std::numeric_limits<int>::max();
  int jmin_ = std::numeric_limits<int>::max();
  int imax_ = std::numeric_limits<int>::min();
  int jmax_ = std::numeric_limits<int>::min();
  int filling_ = 0;
};
} // namespace TMIV::Packer

#endif
