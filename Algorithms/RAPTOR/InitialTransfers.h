/**********************************************************************************

 Copyright (c) 2023-2025 Patrick Steil
 Copyright (c) 2019-2022 KIT ITI Algorithmics Group

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/
#pragma once

#include "../../DataStructures/RAPTOR/TransferModes.h"
#include "../CH/Query/BucketQuery.h"
#include "../CH/Query/CHQuery.h"

namespace RAPTOR {

using DijkstraInitialTransfers = CH::Query<TransferGraph, false, false, true>;
using CoreCHInitialTransfers = CH::Query<CHGraph, true, false, true>;
using BucketCHInitialTransfers = CH::BucketQuery<CHGraph, true, false>;

class TransitiveInitialTransfers {
 public:
  using Graph = TransferGraph;

  TransitiveInitialTransfers(const TransferGraph& forwardGraph,
                             const TransferGraph& backwardGraph)
      : graph{forwardGraph, backwardGraph},
        distance{std::vector<int>(forwardGraph.numVertices(), INFTY),
                 std::vector<int>(backwardGraph.numVertices(), INFTY)},
        root{Vertex(0), Vertex(0)},
        reachedPOIs{std::vector<Vertex>(), std::vector<Vertex>()},
        targetDistance(INFTY) {}

  template <typename ATTRIBUTE>
  TransitiveInitialTransfers(const TransferGraph& forwardGraph,
                             const TransferGraph& backwardGraph,
                             const Vertex::ValueType, const ATTRIBUTE)
      : TransitiveInitialTransfers(forwardGraph, backwardGraph) {}

  template <bool TARGET_PRUNING = true>
  inline void run(const Vertex from, const Vertex to,
                  const double = 1) noexcept {
    clear<FORWARD>();
    clear<BACKWARD>();
    targetDistance = INFTY;

    root[FORWARD] = from;
    root[BACKWARD] = to;

    setDistances<FORWARD>();
    setDistances<BACKWARD>();

    if (root[BACKWARD] < graph[FORWARD].numVertices()) {
      targetDistance = getForwardDistance(root[BACKWARD]);
    }
  }

  inline int getDistance(const Vertex = noVertex) const noexcept {
    return targetDistance;
  }

  inline const std::vector<int>& getForwardDistance() const noexcept {
    return distance[FORWARD];
  }

  inline int getForwardDistance(const Vertex vertex) const noexcept {
    return distance[FORWARD][vertex];
  }

  inline const std::vector<int>& getBackwardDistance() const noexcept {
    return distance[BACKWARD];
  }

  inline int getBackwardDistance(const Vertex vertex) const noexcept {
    return distance[BACKWARD][vertex];
  }

  inline const std::vector<Vertex>& getForwardPOIs() const noexcept {
    return reachedPOIs[FORWARD];
  }

  inline const std::vector<Vertex>& getBackwardPOIs() const noexcept {
    return reachedPOIs[BACKWARD];
  }

 private:
  template <int DIRECTION>
  inline void clear() noexcept {
    reachedPOIs[DIRECTION].clear();
    if (root[DIRECTION] >= graph[DIRECTION].numVertices()) return;
    distance[DIRECTION][root[DIRECTION]] = INFTY;
    for (const Edge edge : graph[DIRECTION].edgesFrom(root[DIRECTION])) {
      const Vertex vertex = graph[DIRECTION].get(ToVertex, edge);
      distance[DIRECTION][vertex] = INFTY;
    }
  }

  template <int DIRECTION>
  inline void setDistances() noexcept {
    if (root[DIRECTION] >= graph[DIRECTION].numVertices()) return;
    distance[DIRECTION][root[DIRECTION]] = 0;
    reachedPOIs[DIRECTION].emplace_back(root[DIRECTION]);
    for (const Edge edge : graph[DIRECTION].edgesFrom(root[DIRECTION])) {
      const Vertex vertex = graph[DIRECTION].get(ToVertex, edge);
      distance[DIRECTION][vertex] = graph[DIRECTION].get(TravelTime, edge);
      reachedPOIs[DIRECTION].emplace_back(vertex);
    }
  }

  TransferGraph graph[2];
  std::vector<int> distance[2];
  Vertex root[2];
  std::vector<Vertex> reachedPOIs[2];
  int targetDistance;
};

template <size_t NUM_MODES, typename INITIAL_TRANSFERS>
class MultimodalInitialTransfers {
 public:
  inline static constexpr size_t NumTransferModes = NUM_MODES;
  using InitialTransferType = INITIAL_TRANSFERS;
  using Graph = typename InitialTransferType::Graph;
  using Type =
      MultimodalInitialTransfers<NumTransferModes, InitialTransferType>;

  MultimodalInitialTransfers(
      TransitiveInitialTransfers& transitiveInitialTransfers,
      std::vector<InitialTransferType>& transfers,
      const std::vector<size_t>& modes, const size_t numVertices,
      const Vertex::ValueType endOfPOIs)
      : transitiveInitialTransfers(transitiveInitialTransfers),
        initialTransfers(transfers),
        modes(modes),
        distance{std::vector<int>(numVertices, INFTY),
                 std::vector<int>(numVertices, INFTY)},
        reachedPOIs{IndexedSet<false, Vertex>(endOfPOIs),
                    IndexedSet<false, Vertex>(endOfPOIs)},
        targetDistance(INFTY) {
    AssertMsg(initialTransfers.size() == NumTransferModes,
              "Wrong number of modes");
    AssertMsg(modes.size() == NumTransferModes, "Wrong number of modes");
  }

  template <bool TARGET_PRUNING = true>
  inline void run(const Vertex from, const Vertex to,
                  const double targetPruningFactor = 1) noexcept {
    clear<FORWARD>();
    clear<BACKWARD>();
    targetDistance = INFTY;

    transitiveInitialTransfers.template run<TARGET_PRUNING>(
        from, to, targetPruningFactor);
    evaluateTransfers<TARGET_PRUNING>(transitiveInitialTransfers, 0);
    for (const size_t i : modes) {
      initialTransfers[i].template run<TARGET_PRUNING>(from, to,
                                                       targetPruningFactor);
      evaluateTransfers<TARGET_PRUNING>(initialTransfers[i],
                                        TransferModeOverhead[modes[i]]);
    }
  }

  inline int getDistance(const Vertex = noVertex) const noexcept {
    return targetDistance;
  }

  inline const std::vector<int>& getForwardDistance() const noexcept {
    return distance[FORWARD];
  }

  inline int getForwardDistance(const Vertex vertex) const noexcept {
    return distance[FORWARD][vertex];
  }

  inline const std::vector<int>& getBackwardDistance() const noexcept {
    return distance[BACKWARD];
  }

  inline int getBackwardDistance(const Vertex vertex) const noexcept {
    return distance[BACKWARD][vertex];
  }

  inline const std::vector<Vertex>& getForwardPOIs() const noexcept {
    return reachedPOIs[FORWARD].getValues();
  }

  inline const std::vector<Vertex>& getBackwardPOIs() const noexcept {
    return reachedPOIs[BACKWARD].getValues();
  }

 private:
  template <int DIRECTION>
  inline void clear() noexcept {
    for (const Vertex vertex : reachedPOIs[DIRECTION]) {
      distance[DIRECTION][vertex] = INFTY;
    }
    reachedPOIs[DIRECTION].clear();
  }

  template <bool TARGET_PRUNING, typename TRANSFERS>
  inline void evaluateTransfers(TRANSFERS& transfers,
                                const int overhead) noexcept {
    for (const Vertex vertex : transfers.getForwardPOIs()) {
      reachedPOIs[FORWARD].insert(vertex);
      distance[FORWARD][vertex] =
          std::min(distance[FORWARD][vertex],
                   transfers.getForwardDistance(vertex) + overhead);
    }
    for (const Vertex vertex : transfers.getBackwardPOIs()) {
      reachedPOIs[BACKWARD].insert(vertex);
      distance[BACKWARD][vertex] =
          std::min(distance[BACKWARD][vertex],
                   transfers.getBackwardDistance(vertex) + overhead);
    }
    targetDistance =
        std::min(targetDistance, transfers.getDistance() + overhead);
  }

  TransitiveInitialTransfers& transitiveInitialTransfers;
  std::vector<InitialTransferType>& initialTransfers;
  const std::vector<size_t>& modes;

  std::vector<int> distance[2];
  IndexedSet<false, Vertex> reachedPOIs[2];
  int targetDistance;
};

}  // namespace RAPTOR
