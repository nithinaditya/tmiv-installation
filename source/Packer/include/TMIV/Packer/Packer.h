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

#ifndef _TMIV_PACKER_PACKER_H_
#define _TMIV_PACKER_PACKER_H_

#include <TMIV/Packer/IPacker.h>

#include <TMIV/Common/Json.h>
#include <TMIV/Packer/Cluster.h>

#include <tuple>

namespace TMIV::Packer {
class Packer : public IPacker {
  enum SORTING_METHOD { AREA_DESCENDING = 0, VIEW_ID_ASCENDING = 1 };

public:
  Packer(const Common::Json & /*unused*/, const Common::Json & /*componentNode*/);
  Packer(const Packer &) = delete;
  Packer(Packer &&) = default;
  auto operator=(const Packer &) -> Packer & = delete;
  auto operator=(Packer &&) -> Packer & = default;
  ~Packer() override = default;

  auto pack(const Common::SizeVector &atlasSize, const Common::MaskList &masks,
            const MivBitstream::ViewParamsList &viewParamsList, const int blockSize)
      -> MivBitstream::PatchParamsList override;
  void updateAggregatedEntityMasks(const std::vector<Common::MaskList> &entityMasks) override;

private:
  int m_minPatchSize{};
  int m_overlap{};
  bool m_pip{};
  bool m_enableMerging{};
  SORTING_METHOD m_sortingMethod{};
  bool m_enableRecursiveSplit{true};
  int m_maxEntityId{0};
  std::vector<Common::MaskList> m_aggregatedEntityMasks{};
  Common::Vec2i m_entityEncodeRange;
  auto computeClusters(const Common::MaskList &masks,
                       const MivBitstream::ViewParamsList &viewParamsList)
      -> std::tuple<ClusterList, ClusteringMapList, std::vector<int>>;
  auto computeClusterToPack(const MivBitstream::ViewParamsList &viewParamsList, int m_blockSize,
                            ClusterList &clusterList, const ClusteringMapList &clusteringMap) const;
};

} // namespace TMIV::Packer

#endif
