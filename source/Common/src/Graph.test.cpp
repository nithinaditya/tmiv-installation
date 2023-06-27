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
#include <sstream>

#include <TMIV/Common/Graph.h>

namespace TMIV::Common::Graph {
TEST_CASE("SparseDirectedAcyclicGraph graph default constructor") {
  const SparseDirectedAcyclicGraph<float> unit{};
  REQUIRE(unit.getNumberOfNodes() == 0);
}

TEST_CASE("SparseDirectedAcyclicGraph graph with two nodes") {
  SparseDirectedAcyclicGraph<float> unit{2};
  REQUIRE(unit.getNumberOfNodes() == 2);
  unit.connect(0, 1, 1.3F);
  REQUIRE(unit.getNeighbourhoodSize(0) == 1);
  REQUIRE(unit.getNeighbour(0, 0).weight() == 1.3F);
  REQUIRE(unit.getNeighbourhood(1).empty());
}

TEST_CASE("SparseDirectedAcyclicGraph graph with two nodes, inverted") {
  SparseDirectedAcyclicGraph<float> unit{1};
  REQUIRE(unit.getNumberOfNodes() == 1);
  unit.addNode();
  REQUIRE(unit.getNumberOfNodes() == 2);
  unit.connect(1, 0, 0.3F);
  REQUIRE(unit.getNeighbourhood(0).empty());
  REQUIRE(unit.getNeighbourhoodSize(1) == 1);
  REQUIRE(unit.getNeighbour(1, 0).weight() == 0.3F);
}

TEST_CASE("SparseDirectedAcyclicGraph graph with three nodes") {
  SparseDirectedAcyclicGraph<float> unit{3};
  unit.connect(0, 1, 1.3F);
  unit.connect(0, 2, 0.3F);
  unit.connect(1, 2, 2.0F);

  REQUIRE(unit.getNeighbourhoodSize(0) == 2);
  REQUIRE(unit.getNeighbour(0, 0).weight() == 1.3F);
  REQUIRE(unit.getNeighbour(0, 1).weight() == 0.3F);

  REQUIRE(unit.getNeighbourhoodSize(1) == 1);
  REQUIRE(unit.getNeighbour(1, 0).weight() == 2.0F);
}

TEST_CASE("When connecting one node to a cycle, throw exception") {
  SparseDirectedAcyclicGraph<float> unit{1};
  REQUIRE_THROWS_AS(unit.connect(0, 0, 0.5F), std::logic_error);
}

TEST_CASE("When connecting two nodes to a cycle, throw exception") {
  SparseDirectedAcyclicGraph<float> unit{2};
  unit.connect(1, 0, 0.3F);
  REQUIRE_THROWS_AS(unit.connect(0, 1, 0.5F), std::logic_error);
}

TEST_CASE("When connecting three nodes to a cycle, throw exception") {
  SparseDirectedAcyclicGraph<float> unit{3};
  unit.connect(1, 0, 0.3F);
  unit.connect(2, 1, 0.3F);
  REQUIRE_THROWS_AS(unit.connect(0, 2, 0.5F), std::logic_error);
}

TEST_CASE("When connecting multiple nodes to a cycle, throw exception") {
  SparseDirectedAcyclicGraph<float> unit{5};
  unit.connect(1, 0, 0.3F);
  unit.connect(2, 0, 1.4F);
  unit.connect(3, 2, 4.5F);
  unit.connect(4, 3, 3.6F);
  unit.connect(4, 0, 2.7F);
  REQUIRE_THROWS_AS(unit.connect(0, 4, 0.5F), std::logic_error);
  REQUIRE_THROWS_AS(unit.connect(2, 2, 0.5F), std::logic_error);
}

TEST_CASE("getDescendingOrderId of SparseDirectedAcyclicGraph Graph with single root") {
  SparseDirectedAcyclicGraph<float> unit{5};
  unit.connect(1, 0, 0.3F);
  unit.connect(2, 0, 1.3F);
  unit.connect(3, 2, 1.2F);
  unit.connect(4, 2, 1.1F);
  const auto result = unit.getDescendingOrderId();
  REQUIRE(result == std::vector<NodeId>{0, 1, 2, 3, 4});
}

TEST_CASE("getDescendingOrderId of SparseDirectedAcyclicGraph Graph with two disconnected graphs") {
  SparseDirectedAcyclicGraph<float> unit{5};
  unit.connect(1, 0, 0.3F);
  unit.connect(3, 0, 1.3F);
  unit.connect(2, 4, 1.1F);
  const auto result = unit.getDescendingOrderId();
  REQUIRE(result == std::vector<NodeId>{0, 1, 3, 4, 2});
}

TEST_CASE(
    "getDescendingOrderId of SparseDirectedAcyclicGraph Graph with two roots shared by node") {
  SparseDirectedAcyclicGraph<float> unit{5};
  unit.connect(1, 0, 0.3F);
  unit.connect(3, 0, 1.3F);
  unit.connect(4, 0, 1.3F);
  unit.connect(4, 2, 1.1F);
  const auto result = unit.getDescendingOrderId();
  REQUIRE(result == std::vector<NodeId>{0, 1, 2, 3, 4});
}

TEST_CASE("Graph ostream operator for float") {
  SparseDirectedAcyclicGraph<float> unit{5};
  unit.connect(1, 0, 0.3F);
  unit.connect(2, 0, 1.3F);
  unit.connect(3, 2, 1.2F);
  unit.connect(4, 2, 1.1F);
  std::ostringstream stream;
  stream << unit;
  REQUIRE(stream.str() == R"(n0 ->
n1 -> n0[0.3]
n2 -> n0[1.3]
n3 -> n2[1.2]
n4 -> n2[1.1])");
}

TEST_CASE("Graph ostream operator for integer") {
  SparseDirectedAcyclicGraph<int> unit{6};
  unit.connect(1, 0, 0);
  unit.connect(2, 0, 1);
  unit.connect(3, 2, 4);
  unit.connect(4, 2, 80);
  unit.connect(4, 0, 32);
  unit.connect(5, 0, 3);
  unit.connect(5, 2, 5);
  std::ostringstream stream{};
  stream << unit;
  REQUIRE(stream.str() == R"(n0 ->
n1 -> n0[0]
n2 -> n0[1]
n3 -> n2[4]
n4 -> n2[80] n0[32]
n5 -> n0[3] n2[5])");
}
} // namespace TMIV::Common::Graph
