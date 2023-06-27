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

#include <TMIV/Packer/Cluster.h>

auto roundToAlignment(int val, int alignment) -> int {
  return ((int(val - 1) / alignment + 1) * alignment);
}

namespace TMIV::Packer {
static const uint16_t ACTIVE = 65534;
static const uint16_t INVALID = 65535;

Cluster::Cluster(int viewId, bool isBasicView, int clusterId, int entityId)
    : viewId_(viewId), m_isBasicView{isBasicView}, clusterId_(clusterId), entityId_(entityId) {}

void Cluster::push(int i, int j) {
  if (i < imin_) {
    imin_ = i;
  }
  if (imax_ < i) {
    imax_ = i;
  }
  if (j < jmin_) {
    jmin_ = j;
  }
  if (jmax_ < j) {
    jmax_ = j;
  }

  filling_++;
}

auto Cluster::setEntityId(Cluster &c, int entityId) -> Cluster {
  Cluster d(c.viewId_, c.isBasicView(), c.clusterId_, entityId);
  d.imin_ = c.imin_;
  d.imax_ = c.imax_;
  d.jmin_ = c.jmin_;
  d.jmax_ = c.jmax_;
  d.numActivePixels_ = c.numActivePixels_;
  d.filling_ = c.filling_;
  return d;
}

auto Cluster::align(const Cluster &c, int alignment) -> Cluster {
  Cluster d(c.viewId_, c.isBasicView(), c.clusterId_, c.entityId_);

  d.imin_ = c.imin_ - (c.imin_ % alignment);
  d.imax_ = c.imax_; // modification to align the imin,jmin to even values to
                     // help renderer

  d.jmin_ = c.jmin_ - (c.jmin_ % alignment);
  d.jmax_ = c.jmax_; // modification to align the imin,jmin to even values to
                     // help renderer

  d.numActivePixels_ = c.numActivePixels_;
  d.filling_ = c.filling_;

  return d;
}

auto Cluster::merge(const Cluster &c1, const Cluster &c2) -> Cluster {
  assert(!c1.isBasicView() && !c2.isBasicView());
  Cluster c(c1.viewId_, false, c1.clusterId_, c1.entityId_);

  c.imin_ = std::min(c1.imin_, c2.imin_);
  c.imax_ = std::max(c1.imax_, c2.imax_);

  c.jmin_ = std::min(c1.jmin_, c2.jmin_);
  c.jmax_ = std::max(c1.jmax_, c2.jmax_);

  c.numActivePixels_ = c1.numActivePixels_ + c2.numActivePixels_;
  c.filling_ = (c1.filling_ + c2.filling_);

  return c;
}

auto Cluster::splitLPatchHorizontally(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                                      int alignment, int minPatchSize,
                                      const std::array<std::deque<int>, 2> &min_w_agg,
                                      const std::array<std::deque<int>, 2> &max_w_agg) const
    -> bool {
  double splitThresholdL = 0.9;

  const Cluster &c = (*this);
  const auto &clusteringBuffer = clusteringMap.getPlane(0);
  int H = c.height();
  int W = c.width();

  int alignedImsize = roundToAlignment(W, alignment) * roundToAlignment(H, alignment);

  int minArea = alignedImsize;
  int bestSplitPos = 0;
  for (int h = minPatchSize; h < H - minPatchSize; h++) {
    int currArea = roundToAlignment(h + 1, alignment) *
                       roundToAlignment(max_w_agg[0][h] - min_w_agg[0][h], alignment) +
                   roundToAlignment(H - h - 1, alignment) *
                       roundToAlignment(max_w_agg[1][h] - min_w_agg[1][h], alignment);

    if (minArea > currArea) {
      minArea = currArea;
      bestSplitPos = h;
    }
  }

  if ((bestSplitPos != 0) && static_cast<double>(minArea) / alignedImsize < splitThresholdL) {
    Cluster c1(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());
    Cluster c2(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());

    for (int i = c.imin(); i < c.imin() + bestSplitPos + 1; i++) {
      for (int j = c.jmin(); j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c1.push(i, j);
        }
      }
    }
    for (int i = c.imin() + bestSplitPos - 1; i <= c.imax(); i++) {
      for (int j = c.jmin(); j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c2.push(i, j);
        }
      }
    }

    c1.recursiveSplit(clusteringMap, out, alignment, minPatchSize);
    c2.recursiveSplit(clusteringMap, out, alignment, minPatchSize);

    return true;
  }
  return false;
}

auto Cluster::splitCPatchVertically(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                                    int alignment, int minPatchSize) const -> bool {
  double splitThresholdC = 0.3;

  const Cluster &c = (*this);
  const auto &clusteringBuffer = clusteringMap.getPlane(0);
  int H = c.height();
  int W = c.width();

  int numOfEmptyBlocks = 0;
  int numOfNonEmptyBlocks = 0;

  for (int h = 0; h < H; h += alignment) {
    for (int w = 0; w < W; w += alignment) {
      bool isEmpty = true;

      for (int hh = h; hh < std::min(h + alignment, H); hh++) {
        int i = hh + c.imin();
        for (int ww = w; ww < std::min(w + alignment, W); ww++) {
          int j = ww + c.jmin();

          if (clusteringBuffer(i, j) == c.getClusterId()) {
            isEmpty = false;
            break;
          }
        } // ww
        if (!isEmpty) {
          break;
        }
      } // hh
      if (isEmpty) {
        numOfEmptyBlocks++;
      } else {
        numOfNonEmptyBlocks++;
      }
    } // w
  }   // h

  if (static_cast<double>(numOfNonEmptyBlocks) / (numOfEmptyBlocks + numOfNonEmptyBlocks) <
      splitThresholdC) {
    int bestSplitPos = roundToAlignment(W / 2, alignment);

    Cluster c1(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());
    Cluster c2(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());

    for (int i = c.imin(); i <= c.imax(); i++) {
      for (int j = c.jmin(); j < c.jmin() + bestSplitPos + 1; j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c1.push(i, j);
        }
      }
    }
    for (int i = c.imin(); i <= c.imax(); i++) {
      for (int j = c.jmin() + bestSplitPos - 1; j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c2.push(i, j);
        }
      }
    }

    c1.recursiveSplit(clusteringMap, out, alignment, minPatchSize);
    c2.recursiveSplit(clusteringMap, out, alignment, minPatchSize);

    return true;
  }
  return false;
}

auto Cluster::splitCPatchHorizontally(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                                      int alignment, int minPatchSize) const -> bool {
  double splitThresholdC = 0.3;

  const Cluster &c = (*this);
  const auto &clusteringBuffer = clusteringMap.getPlane(0);
  int H = c.height();
  int W = c.width();

  int numOfEmptyBlocks = 0;
  int numOfNonEmptyBlocks = 0;

  for (int h = 0; h < H; h += alignment) {
    for (int w = 0; w < W; w += alignment) {
      bool isEmpty = true;

      for (int hh = h; hh < std::min(h + alignment, H); hh++) {
        int i = hh + c.imin();
        for (int ww = w; ww < std::min(w + alignment, W); ww++) {
          int j = ww + c.jmin();

          if (clusteringBuffer(i, j) == c.getClusterId()) {
            isEmpty = false;
            break;
          }
        } // ww
        if (!isEmpty) {
          break;
        }
      } // hh
      if (isEmpty) {
        numOfEmptyBlocks++;
      } else {
        numOfNonEmptyBlocks++;
      }
    } // w
  }   // h

  if (static_cast<double>(numOfNonEmptyBlocks) / (numOfEmptyBlocks + numOfNonEmptyBlocks) <
      splitThresholdC) {
    int bestSplitPos = roundToAlignment(H / 2, alignment);

    Cluster c1(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());
    Cluster c2(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());

    for (int i = c.imin(); i < c.imin() + bestSplitPos + 1; i++) {
      for (int j = c.jmin(); j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c1.push(i, j);
        }
      }
    }
    for (int i = c.imin() + bestSplitPos - 1; i <= c.imax(); i++) {
      for (int j = c.jmin(); j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c2.push(i, j);
        }
      }
    }

    c1.recursiveSplit(clusteringMap, out, alignment, minPatchSize);
    c2.recursiveSplit(clusteringMap, out, alignment, minPatchSize);

    return true;
  }
  return false;
}

auto Cluster::splitLPatchVertically(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                                    int alignment, int minPatchSize,
                                    const std::array<std::deque<int>, 2> &min_h_agg,
                                    const std::array<std::deque<int>, 2> &max_h_agg) const -> bool {
  double splitThresholdL = 0.9;

  const Cluster &c = (*this);
  const auto &clusteringBuffer = clusteringMap.getPlane(0);
  int H = c.height();
  int W = c.width();

  int alignedImsize = roundToAlignment(W, alignment) * roundToAlignment(H, alignment);

  int minArea = alignedImsize;
  int bestSplitPos = 0;
  for (int w = minPatchSize; w < W - minPatchSize; w++) {
    int currArea = roundToAlignment(w + 1, alignment) *
                       roundToAlignment(max_h_agg[0][w] - min_h_agg[0][w], alignment) +
                   roundToAlignment(W - w - 1, alignment) *
                       roundToAlignment(max_h_agg[1][w] - min_h_agg[1][w], alignment);

    if (minArea > currArea) {
      minArea = currArea;
      bestSplitPos = w;
    }
  }

  if ((bestSplitPos != 0) && static_cast<double>(minArea) / alignedImsize < splitThresholdL) {
    Cluster c1(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());
    Cluster c2(c.getViewId(), c.isBasicView(), c.getClusterId(), c.getEntityId());

    for (int i = c.imin(); i <= c.imax(); i++) {
      for (int j = c.jmin(); j < c.jmin() + bestSplitPos + 1; j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c1.push(i, j);
        }
      }
    }
    for (int i = c.imin(); i <= c.imax(); i++) {
      for (int j = c.jmin() + bestSplitPos - 1; j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c2.push(i, j);
        }
      }
    }

    c1.recursiveSplit(clusteringMap, out, alignment, minPatchSize);
    c2.recursiveSplit(clusteringMap, out, alignment, minPatchSize);

    return true;
  }
  return false;
}

void Cluster::recursiveSplit(const ClusteringMap &clusteringMap, std::vector<Cluster> &out,
                             int alignment, int minPatchSize) const {
  bool splitted = false;
  const int maxNonSplitTableSize = 64;

  if (width() > height()) { // split vertically
    if (width() > maxNonSplitTableSize) {
      const auto [min_h_agg, max_h_agg] = createAggregatedQueues(clusteringMap, false);
      splitted =
          splitLPatchVertically(clusteringMap, out, alignment, minPatchSize, min_h_agg, max_h_agg);
      if (!splitted) {
        splitted = splitCPatchVertically(clusteringMap, out, alignment, minPatchSize);
      }
    }
  } else { // split horizontally
    if (height() > maxNonSplitTableSize) {
      const auto [min_w_agg, max_w_agg] = createAggregatedQueues(clusteringMap, true);
      splitted = splitLPatchHorizontally(clusteringMap, out, alignment, minPatchSize, min_w_agg,
                                         max_w_agg);
      if (!splitted) {
        splitted = splitCPatchHorizontally(clusteringMap, out, alignment, minPatchSize);
      }
    }
  }

  if (!splitted) {
    out.push_back(*this);
  }
}

auto Cluster::createAggregatedQueues(const ClusteringMap &clusteringMap,
                                     const bool aggregateHorizontally) const
    -> std::tuple<std::array<std::deque<int>, 2>, std::array<std::deque<int>, 2>> {

  const auto aggregationDimensionSize = aggregateHorizontally ? height() : width();

  std::array<std::deque<int>, 2> min_agg{};
  std::array<std::deque<int>, 2> max_agg{};
  const auto [minima, maxima] = computeMinAndMaxVectors(clusteringMap, aggregateHorizontally);

  min_agg[0].push_back(minima[0]);
  max_agg[0].push_back(maxima[0]);
  for (int i = 1; i < aggregationDimensionSize; i++) {
    min_agg[0].push_back(std::min(min_agg[0][i - 1], minima[i]));
    max_agg[0].push_back(std::max(max_agg[0][i - 1], maxima[i]));
  }
  min_agg[1].push_front(minima[aggregationDimensionSize - 1]);
  max_agg[1].push_front(maxima[aggregationDimensionSize - 1]);
  for (int i = aggregationDimensionSize - 2; i >= 0; i--) {
    min_agg[1].push_front(std::min(min_agg[1][0], minima[i]));
    max_agg[1].push_front(std::max(max_agg[1][0], maxima[i]));
  }
  return {min_agg, max_agg};
}

auto Cluster::computeMinAndMaxVectors(const ClusteringMap &clusteringMap,
                                      bool aggregateHorizontally) const
    -> std::tuple<std::vector<int>, std::vector<int>> {
  const auto dim1 = aggregateHorizontally ? height() : width();
  const auto dim2 = aggregateHorizontally ? width() : height();
  const auto minDim1 = aggregateHorizontally ? imin() : jmin();
  const auto minDim2 = aggregateHorizontally ? jmin() : imin();

  const auto &clusteringBuffer = clusteringMap.getPlane(0);
  std::vector<int> minima(dim1, dim2 - 1);
  std::vector<int> maxima(dim1, 0);

  for (int k = 0; k < dim1; k++) {
    int i = k + minDim1;

    for (int l = 0; l < dim2; l++) {
      int j = l + minDim2;
      const auto bufferValue =
          aggregateHorizontally ? clusteringBuffer(i, j) : clusteringBuffer(j, i);
      if (bufferValue == getClusterId()) {
        minima[k] = l;
        break;
      }
    }

    for (int l = dim2 - 1; l >= 0; l--) {
      int j = l + minDim2;
      const auto bufferValue =
          aggregateHorizontally ? clusteringBuffer(i, j) : clusteringBuffer(j, i);
      if (bufferValue == getClusterId()) {
        maxima[k] = l;
        break;
      }
    }
  }

  return {minima, maxima};
}

auto Cluster::split(const ClusteringMap &clusteringMap, int overlap) const
    -> std::pair<Cluster, Cluster> {
  const auto &clusteringBuffer = clusteringMap.getPlane(0);
  const Cluster &c = *this;
  assert(!c.isBasicView());
  Cluster c1(c.getViewId(), false, c.getClusterId(), c.getEntityId());
  Cluster c2(c.getViewId(), false, c.getClusterId(), c.getEntityId());

  if (c.width() < c.height()) {
    int imid = (c.imin() + c.imax()) / 2;
    int imid1 = std::min(imid + overlap, static_cast<int>(clusteringBuffer.m()) - 1);
    int imid2 = std::max(0, imid - overlap);

    for (int i = c.imin(); i < imid1; i++) {
      for (int j = c.jmin(); j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c1.push(i, j);
        }
      }
    }

    for (int i = imid2; i <= c.imax(); i++) {
      for (int j = c.jmin(); j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c2.push(i, j);
        }
      }
    }
  } else {
    int jmid = (c.jmin() + c.jmax()) / 2;
    int jmid1 = std::min(jmid + overlap, static_cast<int>(clusteringBuffer.n()) - 1);
    int jmid2 = std::max(0, jmid - overlap);

    for (int i = c.imin(); i <= c.imax(); i++) {
      for (int j = c.jmin(); j < jmid1; j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c1.push(i, j);
        }
      }
    }

    for (int i = c.imin(); i <= c.imax(); i++) {
      for (int j = jmid2; j <= c.jmax(); j++) {
        if (clusteringBuffer(i, j) == c.getClusterId()) {
          c2.push(i, j);
        }
      }
    }
  }
  c1.numActivePixels_ = (c.numActivePixels_ * c1.filling_) / c.filling_; // Approximation
  c2.numActivePixels_ = c.numActivePixels_ - c1.numActivePixels_;        // Approximation
  return std::pair<Cluster, Cluster>(c1, c2);
}

namespace {
template <typename ClusterBufferType>
auto mergePatches(ClusterList &clusterList, int A, int B, const Cluster &cluster,
                  std::queue<std::array<int, 2>> &candidates, ClusterBufferType &clusteringBuffer)
    -> int {
  int i_top = cluster.imin();
  int i_bottom = cluster.imax();
  int j_left = cluster.jmin();
  int j_right = cluster.jmax();

  auto subRegionGrowing = [&](int ID) {
    Cluster subCluster(cluster.getViewId(), cluster.isBasicView(), ID, cluster.getEntityId());
    while (!candidates.empty()) {
      const std::array<int, 2> &current = candidates.front();
      int a = current[0];
      int b = current[1];

      subCluster.push(a, b);
      clusteringBuffer(a, b) = static_cast<uint16_t>(ID);

      if (0 < a) {
        if (clusteringBuffer(a - 1, b) == ACTIVE) {
          candidates.push({a - 1, b});
          subCluster.push(a - 1, b);
          clusteringBuffer(a - 1, b) = static_cast<uint16_t>(ID);
        }
        if (0 < b) {
          if (clusteringBuffer(a - 1, b - 1) == ACTIVE) {
            candidates.push({a - 1, b - 1});
            subCluster.push(a - 1, b - 1);
            clusteringBuffer(a - 1, b - 1) = static_cast<uint16_t>(ID);
          }
        }
        if (b < B - 1) {
          if (clusteringBuffer(a - 1, b + 1) == ACTIVE) {
            candidates.push({a - 1, b + 1});
            subCluster.push(a - 1, b + 1);
            clusteringBuffer(a - 1, b + 1) = static_cast<uint16_t>(ID);
          }
        }
      }
      if (a < A - 1) {
        if (clusteringBuffer(a + 1, b) == ACTIVE) {
          candidates.push({a + 1, b});
          subCluster.push(a + 1, b);
          clusteringBuffer(a + 1, b) = static_cast<uint16_t>(ID);
        }
        if (0 < b) {
          if (clusteringBuffer(a + 1, b - 1) == ACTIVE) {
            candidates.push({a + 1, b - 1});
            subCluster.push(a + 1, b - 1);
            clusteringBuffer(a + 1, b - 1) = static_cast<uint16_t>(ID);
          }
        }
        if (b < B - 1) {
          if (clusteringBuffer(a + 1, b + 1) == ACTIVE) {
            candidates.push({a + 1, b + 1});
            subCluster.push(a + 1, b + 1);
            clusteringBuffer(a + 1, b + 1) = static_cast<uint16_t>(ID);
          }
        }
      }
      if (0 < b) {
        if (clusteringBuffer(a, b - 1) == ACTIVE) {
          candidates.push({a, b - 1});
          subCluster.push(a, b - 1);
          clusteringBuffer(a, b - 1) = static_cast<uint16_t>(ID);
        }
      }
      if (b < B - 1) {
        if (clusteringBuffer(a, b + 1) == ACTIVE) {
          candidates.push({a, b + 1});
          subCluster.push(a, b + 1);
          clusteringBuffer(a, b + 1) = static_cast<uint16_t>(ID);
        }
      }
      candidates.pop();
    }
    clusterList.push_back(subCluster);
  };

  int subClusterId = cluster.getClusterId();
  // left side
  if (j_left != 0) {
    for (int i_unit = i_top; i_unit <= i_bottom; i_unit++) {
      if (clusteringBuffer(i_unit, j_left - 1) == ACTIVE) {
        subClusterId = subClusterId + 1;
        candidates.push({i_unit, j_left - 1});
        subRegionGrowing(subClusterId);
      }
    }
  }
  // right side
  if (j_right != B - 1) {
    for (int i_unit = i_top; i_unit <= i_bottom; i_unit++) {
      if (clusteringBuffer(i_unit, j_right + 1) == ACTIVE) {
        subClusterId = subClusterId + 1;
        candidates.push({i_unit, j_right + 1});
        subRegionGrowing(subClusterId);
      }
    }
  }
  // bottom side
  if (i_bottom != A - 1) {
    for (int j_unit = j_left; j_unit <= j_right; j_unit++) {
      if (clusteringBuffer(i_bottom + 1, j_unit) == ACTIVE) {
        subClusterId = subClusterId + 1;
        candidates.push({i_bottom + 1, j_unit});
        subRegionGrowing(subClusterId);
      }
    }
  }
  if (!cluster.isBasicView()) {
    const auto clusterId = static_cast<std::uint16_t>(cluster.getClusterId());
    for (int i_inter = i_top; i_inter <= i_bottom; i_inter++) {
      for (int j_inter = j_left; j_inter <= j_right; j_inter++) {
        if (clusteringBuffer(i_inter, j_inter) == ACTIVE) {
          clusteringBuffer(i_inter, j_inter) = clusterId;
        }
      }
    }
  }

  return subClusterId;
}
} // namespace

auto Cluster::retrieve(const int viewId, const Common::Mask &maskMap, const int firstClusterId,
                       const bool isBasicView, const bool enableMerging)
    -> std::pair<ClusterList, ClusteringMap> {
  std::pair<ClusterList, ClusteringMap> out(ClusterList(),
                                            ClusteringMap(maskMap.getWidth(), maskMap.getHeight()));
  ClusterList &clusterList = out.first;
  auto &clusteringBuffer = out.second.getPlane(0);

  const auto &maskBuffer = maskMap.getPlane(0);
  int A = static_cast<int>(maskBuffer.m());
  int B = static_cast<int>(maskBuffer.n());
  int S = static_cast<int>(maskBuffer.size());

  // Build active list
  std::vector<int> activeList;

  activeList.reserve(S);
  for (int i = 0; i < S; i++) {
    if (0 < maskBuffer[i]) {
      activeList.push_back(i);
      clusteringBuffer[i] = ACTIVE;
    } else {
      clusteringBuffer[i] = INVALID;
    }
  }

  // Region growing
  int clusterId = firstClusterId;
  auto iter_seed = activeList.begin();

  while (iter_seed != activeList.end()) {
    div_t dv = div(*iter_seed, B);
    Cluster cluster(viewId, isBasicView, clusterId, 0);
    std::queue<std::array<int, 2>> candidates;

    cluster.push(dv.quot, dv.rem);
    candidates.push({dv.quot, dv.rem});
    clusteringBuffer(dv.quot, dv.rem) = static_cast<uint16_t>(clusterId);

    auto tryAddCandidate = [&](int a, int b) {
      std::uint16_t &visitedId = clusteringBuffer(a, b);

      if (visitedId == ACTIVE) {
        cluster.push(a, b);
        visitedId = static_cast<uint16_t>(clusterId);
        candidates.push({a, b});
      }
    };

    while (!candidates.empty()) {
      const std::array<int, 2> &current = candidates.front();
      int a = current[0];
      int b = current[1];

      if (0 < a) {
        tryAddCandidate(a - 1, b);

        if (0 < b) {
          tryAddCandidate(a - 1, b - 1);
        }
        if (b < B - 1) {
          tryAddCandidate(a - 1, b + 1);
        }
      }

      if (a < A - 1) {
        tryAddCandidate(a + 1, b);

        if (0 < b) {
          tryAddCandidate(a + 1, b - 1);
        }
        if (b < B - 1) {
          tryAddCandidate(a + 1, b + 1);
        }
      }

      if (0 < b) {
        tryAddCandidate(a, b - 1);
      }
      if (b < B - 1) {
        tryAddCandidate(a, b + 1);
      }

      candidates.pop();
    }

    const auto subClusterId = [&]() {
      if (enableMerging) {
        return mergePatches(clusterList, A, B, cluster, candidates, clusteringBuffer);
      }
      return clusterId;
    }();

    // Update seed & compute # Active Pixels In Patch
    auto prevIter = iter_seed;
    iter_seed = find_if(iter_seed + 1, activeList.end(),
                        [&clusteringBuffer](int i) { return (clusteringBuffer[i] == ACTIVE); });
    auto currentIter = iter_seed;
    auto counter = static_cast<int>(distance(prevIter, currentIter));
    cluster.numActivePixels_ = counter;

    // Updating output
    if (isBasicView) {
      if (!clusterList.empty()) {
        clusterList[0] = Cluster::merge(clusterList[0], cluster);
      } else {
        clusterList.push_back(cluster);
      }

    } else {
      clusterList.push_back(cluster);
      clusterId = subClusterId + 1; // Patch Merging
    }
  }

  return out;
}

} // namespace TMIV::Packer
