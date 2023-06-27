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

#ifndef _TMIV_COMMON_GRAPH_H_
#define _TMIV_COMMON_GRAPH_H_

#include <type_traits>
#include <vector>

namespace TMIV::Common::Graph {
using NodeId = std::size_t;

template <typename T> class Link {
public:
  Link(NodeId n, T w) : m_id(n), m_weight(w) {}
  [[nodiscard]] auto id() const -> NodeId { return m_id; }
  [[nodiscard]] auto weight() const -> T { return m_weight; }

private:
  NodeId m_id{};
  T m_weight{};
};

template <typename T> class SparseDirectedAcyclicGraph {
public:
  explicit SparseDirectedAcyclicGraph(std::size_t nb_nodes = 0) : m_adjacencyList{nb_nodes} {}

  void addNode() { m_adjacencyList.emplace_back(std::vector<Link<T>>{}); }
  void connect(NodeId start, NodeId destination, T weight);

  [[nodiscard]] auto getNumberOfNodes() const -> std::size_t { return m_adjacencyList.size(); }
  [[nodiscard]] auto getNeighbourhoodSize(NodeId node) const -> std::size_t {
    return m_adjacencyList[node].size();
  }
  [[nodiscard]] auto getNeighbour(NodeId node, std::size_t id) const -> Link<T> {
    return m_adjacencyList[node][id];
  }
  [[nodiscard]] auto getNeighbourhood(NodeId id) const -> const std::vector<Link<T>> & {
    return m_adjacencyList[id];
  }
  [[nodiscard]] auto getDescendingOrderId() const -> std::vector<NodeId>;

private:
  [[nodiscard]] auto getIdOfFirstRootNode() const -> NodeId;
  [[nodiscard]] auto isCyclic() const -> bool;
  [[nodiscard]] auto isCyclic(NodeId nodeId, std::vector<bool> &visited,
                              std::vector<bool> &nodesInCurrentPath) const -> bool;

  std::vector<std::vector<Link<T>>> m_adjacencyList;
};
} // namespace TMIV::Common::Graph

//! \brief Send the graph g to the stream os.
template <typename WeightType, std::enable_if_t<std::is_arithmetic_v<WeightType>, bool> = true>
auto operator<<(std::ostream &os,
                const TMIV::Common::Graph::SparseDirectedAcyclicGraph<WeightType> &g)
    -> std::ostream & {
  for (std::size_t i = 0; i < g.getNumberOfNodes(); i++) {
    os << "n" << i << " ->";
    for (std::size_t j = 0; j < g.getNeighbourhoodSize(i); j++) {
      auto l = g.getNeighbour(i, j);
      os << " n" << l.id() << "[" << l.weight() << "]";
    }
    if (i != (g.getNumberOfNodes() - 1)) {
      os << '\n';
    }
  }

  return os;
}

#include "Graph.hpp"

#endif
