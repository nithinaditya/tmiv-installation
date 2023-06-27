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
#error "Include the .h instead of the .hpp."
#endif

#include <algorithm>
#include <fmt/format.h>
#include <utility>

namespace TMIV::Common::Graph {

template <typename T>
void SparseDirectedAcyclicGraph<T>::connect(const NodeId start, const NodeId destination,
                                            const T weight) {
  m_adjacencyList[start].push_back(Link<T>(destination, weight));

  if (isCyclic()) {
    m_adjacencyList[start].pop_back();
    throw std::logic_error(fmt::format("Adding a connection from {} to {} would make the graph "
                                       "cyclic! Did not add the connection.",
                                       start, destination));
  }
}

template <typename T>
auto SparseDirectedAcyclicGraph<T>::getDescendingOrderId() const -> std::vector<NodeId> {
  std::vector<bool> hasBeenMarked(getNumberOfNodes(), false);
  std::vector<NodeId> descendingOrderIds{};

  const auto rootId = getIdOfFirstRootNode();
  descendingOrderIds.push_back(rootId);
  hasBeenMarked[rootId] = true;

  while (descendingOrderIds.size() < getNumberOfNodes()) {
    std::vector<NodeId> shouldBeMarked;

    for (NodeId nodeId = 0; nodeId < getNumberOfNodes(); nodeId++) {
      if (!hasBeenMarked[nodeId]) {
        const auto numberOfMarkedParents = static_cast<std::size_t>(
            std::count_if(getNeighbourhood(nodeId).cbegin(), getNeighbourhood(nodeId).cend(),
                          [&hasBeenMarked = std::as_const(hasBeenMarked)](const auto &node) {
                            return hasBeenMarked[node.id()];
                          }));

        if (numberOfMarkedParents == getNeighbourhoodSize(nodeId)) {
          shouldBeMarked.push_back(nodeId);
        }
      }
    }

    for (const auto nodeId : shouldBeMarked) {
      descendingOrderIds.push_back(nodeId);
      hasBeenMarked[nodeId] = true;
    }
  }

  return descendingOrderIds;
}

template <typename T> auto SparseDirectedAcyclicGraph<T>::getIdOfFirstRootNode() const -> NodeId {
  const auto rootNode = std::find_if(m_adjacencyList.cbegin(), m_adjacencyList.cend(),
                                     [](const auto &links) { return links.empty(); });
  assert(rootNode != m_adjacencyList.cend() &&
         "Could not find a root node! This may not happen for an acyclic graph.");
  return static_cast<NodeId>(std::distance(m_adjacencyList.cbegin(), rootNode));
}

template <typename T> auto SparseDirectedAcyclicGraph<T>::isCyclic() const -> bool {
  std::vector<bool> visited(m_adjacencyList.size(), false);
  std::vector<bool> nodesInCurrentPath(m_adjacencyList.size(), false);

  for (NodeId i = 0; i < getNumberOfNodes(); i++) {
    if (isCyclic(i, visited, nodesInCurrentPath)) {
      return true;
    }
  }
  return false;
}

template <typename T>
auto SparseDirectedAcyclicGraph<T>::isCyclic(const NodeId nodeId, std::vector<bool> &visited,
                                             std::vector<bool> &nodesInCurrentPath) const -> bool {
  if (!visited[nodeId]) {
    visited[nodeId] = true;
    nodesInCurrentPath[nodeId] = true;

    for (const auto &childNode : m_adjacencyList[nodeId]) {
      const auto childNodeId = childNode.id();
      if (!visited[childNodeId] && isCyclic(childNodeId, visited, nodesInCurrentPath)) {
        return true;
      }
      if (nodesInCurrentPath[childNodeId]) {
        return true;
      }
    }
  }

  nodesInCurrentPath[nodeId] = false;
  return false;
}
} // namespace TMIV::Common::Graph
