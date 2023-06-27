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

#include <TMIV/Packer/Packer.h>

#include "MaxRectPiP.h"

#include <iostream>
#include <queue>
#include <stdexcept>

namespace TMIV::Packer {
namespace {
void checkAtlasSize(const Common::SizeVector &atlasSizes, const int blockSize) {
  for (const auto &sz : atlasSizes) {
    if (((sz.x() % blockSize) != 0) || ((sz.y() % blockSize) != 0)) {
      throw std::runtime_error("Atlas size should be a multiple of blocksize");
    }
  }
}

void adaptPatchParamsToMask(MivBitstream::PatchParams &p, int32_t maskWidth, int32_t maskHeight) {
  if (p.atlasPatch3dOffsetU() + p.atlasPatch3dSizeU() > maskWidth) {
    if (p.atlasPatch3dSizeU() <= maskWidth) {
      p.atlasPatch3dOffsetU(maskWidth - p.atlasPatch3dSizeU());
    } else {
      p.atlasPatch3dOffsetU(0);
      p.atlasPatch3dSizeU(maskWidth);
    }
  }
  if (p.atlasPatch3dOffsetV() + p.atlasPatch3dSizeV() > maskHeight) {
    if (p.atlasPatch3dSizeV() <= maskHeight) {
      p.atlasPatch3dOffsetV(maskHeight - p.atlasPatch3dSizeV());
    } else {
      p.atlasPatch3dOffsetV(0);
      p.atlasPatch3dSizeV(maskHeight);
    }
  }
}

} // namespace
Packer::Packer(const Common::Json &rootNode, const Common::Json &componentNode) {
  m_minPatchSize = componentNode.require("MinPatchSize").as<int>();
  m_overlap = componentNode.require("Overlap").as<int>();
  m_pip = componentNode.require("PiP").as<int>() != 0;
  m_enableMerging = componentNode.require("enableMerging").as<bool>();
  switch (auto sortingMethod = componentNode.require("sortingMethod").as<int>()) {
  case 0:
    m_sortingMethod = AREA_DESCENDING;
    break;
  case 1:
    m_sortingMethod = VIEW_ID_ASCENDING;
    break;
  default:
    throw std::runtime_error(fmt::format("Sorting method {} is not available", sortingMethod));
    break;
  }
  m_enableRecursiveSplit = componentNode.require("enableRecursiveSplit").as<bool>();

  if (const auto node = rootNode.optional("maxEntityId")) {
    m_maxEntityId = node.as<int>();
  }
  if (m_maxEntityId > 0) {
    m_entityEncodeRange = rootNode.require("EntityEncodeRange").asVec<int, 2>();
  }
}

void Packer::updateAggregatedEntityMasks(const std::vector<Common::MaskList> &entityMasks) {
  for (const auto &entityMask : entityMasks) {
    m_aggregatedEntityMasks.push_back(entityMask);
  }
}

auto Packer::computeClusterToPack(const MivBitstream::ViewParamsList &viewParamsList,
                                  const int m_blockSize, ClusterList &clusterList,
                                  const ClusteringMapList &clusteringMap) const {
  auto comp = [this, &viewParamsList](const Cluster &p1, const Cluster &p2) -> bool {
    if (viewParamsList[p1.getViewId()].isBasicView != viewParamsList[p2.getViewId()].isBasicView) {
      return viewParamsList[p2.getViewId()].isBasicView;
    }
    // NOTE(FT): added for packing patches from MPI ==> reading in writePatchInAtlas is done in
    // increasing mpiLayerId order
    if (m_sortingMethod == AREA_DESCENDING) {
      if (p1.getArea() != p2.getArea()) {
        return p1.getArea() < p2.getArea();
      }
    } else if (m_sortingMethod == VIEW_ID_ASCENDING) {
      if (p1.getViewId() != p2.getViewId()) {
        return p1.getViewId() > p2.getViewId();
      }
    }
    // NOTE(BK): Stable ordering
    return p1.getClusterId() > p2.getClusterId();
  };

  std::priority_queue<Cluster, std::vector<Cluster>, decltype(comp)> clusterToPack(comp);

  std::vector<Cluster> out{};
  for (const auto &cluster : clusterList) {
    if (m_maxEntityId > 0 || cluster.isBasicView()) {
      out.push_back(cluster);
    } else {
      if (m_enableRecursiveSplit) {
        cluster.recursiveSplit(clusteringMap[cluster.getViewId()], out, m_blockSize,
                               m_minPatchSize);
      } else {
        out.push_back(cluster);
      }
    }
  }

  for (const auto &cluster : out) {
    // modification to align the imin,jmin to even values to help renderer
    Cluster c = Cluster::align(cluster, 2);
    clusterToPack.push(c);
  }
  return clusterToPack;
}

auto Packer::pack(const Common::SizeVector &atlasSizes, const Common::MaskList &masks,
                  const MivBitstream::ViewParamsList &viewParamsList, const int m_blockSize)
    -> MivBitstream::PatchParamsList {
  checkAtlasSize(atlasSizes, m_blockSize);

  auto [clusterList, clusteringMap, clusteringMapIndex] = computeClusters(masks, viewParamsList);

  auto clusterToPack =
      computeClusterToPack(viewParamsList, m_blockSize, clusterList, clusteringMap);

  // Packing
  MivBitstream::PatchParamsList atlasParamsVector{};
  std::vector<MaxRectPiP> packerList{};
  MaxRectPiP::Output packerOutput{};

  packerList.reserve(atlasSizes.size());
  for (const auto &sz : atlasSizes) {
    packerList.emplace_back(sz.x(), sz.y(), m_blockSize, m_pip);
  }

  int patchId = 0;
  int clusteringMap_viewId = 0;
  while (!clusterToPack.empty()) {
    const Cluster &cluster = clusterToPack.top();

    if (m_maxEntityId > 0) {
      clusteringMap_viewId = clusteringMapIndex[cluster.getClusterId()];
    } else {
      clusteringMap_viewId = cluster.getViewId();
    }

    if (m_minPatchSize * m_minPatchSize <= cluster.getArea()) {
      bool packed = false;

      for (size_t atlasId = 0; atlasId < packerList.size(); ++atlasId) {
        MaxRectPiP &packer = packerList[atlasId];

        if (packer.push(cluster, clusteringMap[clusteringMap_viewId], packerOutput)) {
          MivBitstream::PatchParams p;

          p.atlasId = MivBitstream::AtlasId{static_cast<uint8_t>(atlasId)};
          p.atlasPatchProjectionId(static_cast<uint16_t>(cluster.getViewId()));
          p.atlasPatch2dPosX(packerOutput.x());
          p.atlasPatch2dPosY(packerOutput.y());
          p.atlasPatch3dOffsetU(cluster.jmin());
          p.atlasPatch3dOffsetV(cluster.imin());
          p.atlasPatchOrientationIndex(packerOutput.isRotated()
                                           ? MivBitstream::FlexiblePatchOrientation::FPO_ROT270
                                           : MivBitstream::FlexiblePatchOrientation::FPO_NULL);
          p.atlasPatch3dSizeU(Common::align(cluster.width(), m_blockSize));
          p.atlasPatch3dSizeV(Common::align(cluster.height(), m_blockSize));

          adaptPatchParamsToMask(p, masks[cluster.getViewId()].getWidth(),
                                 masks[cluster.getViewId()].getHeight());

          if (m_maxEntityId > 0) {
            p.atlasPatchEntityId(cluster.getEntityId());
            std::cout << "Packing patch " << patchId << " of entity " << *p.atlasPatchEntityId()
                      << " from view " << p.atlasPatchProjectionId() << " with #active pixels "
                      << cluster.getNumActivePixels() << " in atlas " << p.atlasId << std::endl;
          }

          atlasParamsVector.push_back(p);
          patchId++;

          packed = true;
          break;
        }
      }

      if (!packed) {
        if (m_maxEntityId > 0) {
          std::cout << "Spliting cluster " << cluster.getClusterId() << std::endl;
        }
        if (cluster.isBasicView()) {
          throw std::runtime_error("Failed to pack basic view");
        }
        auto cc = cluster.split(clusteringMap[clusteringMap_viewId], m_overlap);

        if (m_minPatchSize * m_minPatchSize <= cc.first.getArea()) {
          // modification to align the imin,jmin to even values to help renderer
          Cluster c = Cluster::align(cc.first, 2);
          clusterToPack.push(c);
          clusteringMapIndex.push_back(clusteringMap_viewId);
        }

        if (m_minPatchSize * m_minPatchSize <= cc.second.getArea()) {
          // modification to align the imin,jmin to even values to help renderer
          Cluster c = Cluster::align(cc.second, 2);
          clusterToPack.push(c);
          clusteringMapIndex.push_back(clusteringMap_viewId);
        }
      }
    }

    clusterToPack.pop();
  }

  return atlasParamsVector;
}

auto Packer::computeClusters(const Common::MaskList &masks,
                             const MivBitstream::ViewParamsList &viewParamsList)
    -> std::tuple<ClusterList, ClusteringMapList, std::vector<int>> {
  ClusterList clusterList{};
  ClusteringMapList clusteringMap{};
  std::vector<int> clusteringMapIndex{};
  int index = 0;
  for (auto viewId = 0; viewId < static_cast<int>(masks.size()); viewId++) {
    if (m_maxEntityId > 0) {
      for (int entityId = m_entityEncodeRange[0]; entityId < m_entityEncodeRange[1]; entityId++) {
        // Entity clustering
        Common::Mask mask = m_aggregatedEntityMasks[entityId - m_entityEncodeRange[0]][viewId];

        auto clusteringOutput =
            Cluster::retrieve(viewId, mask, static_cast<int>(clusterList.size()),
                              viewParamsList[viewId].isBasicView, m_enableMerging);

        for (auto &cluster : clusteringOutput.first) {
          cluster = Cluster::setEntityId(cluster, entityId);
        }

        std::move(clusteringOutput.first.begin(), clusteringOutput.first.end(),
                  back_inserter(clusterList));
        clusteringMap.push_back(std::move(clusteringOutput.second));

        for (size_t i = 0; i < clusteringOutput.first.size(); i++) {
          clusteringMapIndex.push_back(index);
        }

        if (!clusteringOutput.first.empty()) {
          std::cout << "entity " << entityId << " from view " << viewId << " results in "
                    << clusteringOutput.first.size() << " patches\n";
        }
        ++index;
      }
    } else {
      auto clusteringOutput =
          Cluster::retrieve(viewId, masks[viewId], static_cast<int>(clusterList.size()),
                            viewParamsList[viewId].isBasicView, m_enableMerging);

      std::move(clusteringOutput.first.begin(), clusteringOutput.first.end(),
                back_inserter(clusterList));
      clusteringMap.push_back(std::move(clusteringOutput.second));
    }
  }
  if (m_maxEntityId > 0) {
    std::cout << "clusteringMap size = " << clusteringMap.size()
              << " with total # clusters = " << clusteringMapIndex.size() << std::endl;
  }
  return {clusterList, clusteringMap, clusteringMapIndex};
}
} // namespace TMIV::Packer
