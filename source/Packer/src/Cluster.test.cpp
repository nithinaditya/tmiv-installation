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

#include <catch2/catch.hpp>
#include <vector>

#include <TMIV/Packer/Cluster.h>

namespace TMIV::Packer {
namespace {
auto createCluster(int width, int height) -> Cluster {
  Cluster cluster{};
  cluster.push(1, 1);
  cluster.push(height, width);
  return cluster;
}
} // namespace

SCENARIO("128x128 clustering map") {
  ClusteringMap map{128, 128}; // Has to be at least the size of the cluster
  map.fillNeutral();

  std::vector<Cluster> out_clusters{};
  const int alignment = 2;
  const int minPatchSize = 4;

  GIVEN("64x64 cluster not greater than the maxNonSplitTableSize") {
    const auto unit{createCluster(64, 64)};
    WHEN("splitting the cluster") {
      unit.recursiveSplit(map, out_clusters, alignment, minPatchSize);
      THEN("the cluster is not split") { REQUIRE(out_clusters.size() == 1); }
    }
  }
  GIVEN("65x64 cluster") {
    const auto unit{createCluster(65, 64)};
    WHEN("splitting the cluster") {
      unit.recursiveSplit(map, out_clusters, alignment, minPatchSize);
      THEN("the cluster is split vertically") {
        // TODO(CB) fix the input data such that a split occurs
        // REQUIRE(out_clusters.size() == 2);
      }
    }
  }
  GIVEN("64x65 cluster") {
    const auto unit{createCluster(64, 65)};
    WHEN("splitting the cluster") {
      unit.recursiveSplit(map, out_clusters, alignment, minPatchSize);
      THEN("the cluster is split horizontally") {
        // TODO(CB) fix the input data such that a split occurs
        // REQUIRE(out_clusters.size() == 2);
      }
    }
  }
}
} // namespace TMIV::Packer
