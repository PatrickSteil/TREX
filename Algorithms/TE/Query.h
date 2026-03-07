/**********************************************************************************

 Copyright (c) 2023-2025 Patrick Steil

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"{), to deal in
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

#include <limits>
#include <queue>
#include <set>
#include <vector>

#include "../../DataStructures/Container/radix_heap.h"
#include "../../DataStructures/TE/Data.h"
#include "Profiler.h"

namespace TE {

static constexpr uint64_t DIST_MASK = 0xffffffffull;

template <typename PROFILER = NoProfiler, bool NODE_BLOCKING = false>
class Query {
public:
  using Profiler = PROFILER;

  Query(const Data &data)
      : data(data), weight(data.timeExpandedGraph[TravelTime]),
        label(data.timeExpandedGraph.numVertices(), UINT64_MAX),
        parent(data.timeExpandedGraph.numVertices(), noVertex), Q(),
        timeStamp(0) {

    profiler.registerPhases({PHASE_CLEAR, PHASE_FIND_FIRST_VERTEX, PHASE_RUN});
    profiler.registerMetrics({METRIC_SEETLED_VERTICES, METRIC_RELAXED_EDGES,
                              METRIC_FOUND_SOLUTIONS,
                              METRIC_POPPED_BUT_IGNORED});
  }

  int run(const StopId source, const int departureTime,
          const StopId target) noexcept {
    profiler.start();

    AssertMsg(data.isStop(source), "Source is not valid!");
    AssertMsg(data.isStop(target), "Target is not valid!");
    AssertMsg(0 <= departureTime, "Time is negative!");

    profiler.startPhase();
    Vertex firstReachableNode = Vertex(
        data.getFirstReachableDepartureVertexAtStop(source, departureTime));
    if (firstReachableNode == data.numberOfTEVertices()) [[unlikely]]
      return -1;
    AssertMsg(data.isDepartureEvent(firstReachableNode),
              "Invalid departure vertex!");
    profiler.donePhase(PHASE_FIND_FIRST_VERTEX);

    profiler.startPhase();
    clear();
    profiler.donePhase(PHASE_CLEAR);

    profiler.startPhase();
    addSource(firstReachableNode, 0);

    auto targetPruning = [&]() {
      Vertex front = getQFront();
      return (data.timeExpandedGraph.get(StopVertex, front) == target);
    };

    auto settle = [&](const Vertex /*v*/) {
      profiler.countMetric(METRIC_SEETLED_VERTICES);
    };

    auto pruneEdge = [&](const Vertex /*from*/, const Edge /*e*/) {
      profiler.countMetric(METRIC_RELAXED_EDGES);
      return false;
    };

    run(settle, targetPruning, pruneEdge);
    profiler.donePhase(PHASE_RUN);

    Vertex finalVertex = getQFront();
    AssertMsg(finalVertex == noVertex ||
                  data.timeExpandedGraph.get(StopVertex, finalVertex) == target,
              "last vertex was neither noVertex or at the target?");

    if (finalVertex != noVertex) {
      profiler.countMetric(METRIC_FOUND_SOLUTIONS);
    }

    profiler.done();
    return (finalVertex == noVertex ? -1 : getDistance(finalVertex));
  }

  inline const Profiler &getProfiler() const noexcept { return profiler; }

private:
  void clear() noexcept {
    Q.clear();
    timeStamp++;
  }

  void addSource(Vertex source, std::uint32_t dist = 0) noexcept {
    setLabel(source, dist);
    Q.push(static_cast<uint32_t>(source), dist);
  }

  inline uint32_t getDistance(Vertex v) const noexcept {
    uint64_t entry = label[v];
    if ((entry >> 32) != timeStamp)
      return UINT32_MAX;
    return static_cast<uint32_t>(entry);
  }

  inline void setLabel(Vertex v, uint32_t dist) noexcept {
    label[v] = (uint64_t(timeStamp) << 32) | dist;
  }

  inline bool visited(Vertex v) const noexcept {
    return (label[v] >> 32) == timeStamp;
  }

  Vertex getQFront() noexcept {
    return Q.empty() ? noVertex : Vertex(Q.top_key());
  }

  template <typename SETTLE, typename STOP, typename PRUNE_EDGE>
  void run(const SETTLE &settle, const STOP &stop,
           const PRUNE_EDGE &pruneEdge) noexcept {
    while (!Q.empty()) {
      if (stop())
        break;

      auto [u, dist] = Q.topAndPop();

      uint64_t uEntry = label[u];
      uint32_t uDist = static_cast<uint32_t>(uEntry);

      if (dist != uDist) [[unlikely]] {
        profiler.countMetric(METRIC_POPPED_BUT_IGNORED);
        continue;
      }

      const Edge begin = data.timeExpandedGraph.beginEdgeFrom(Vertex(u));
      const Edge end = data.timeExpandedGraph.beginEdgeFrom(Vertex(u + 1));

      for (Edge e = begin; e < end; ++e) {
#ifdef ENABLE_PREFETCH
        constexpr int OFFSET = 16;
        if (Edge(e + OFFSET < end)) {
          Vertex next = data.timeExpandedGraph.get(ToVertex, Edge(e + OFFSET));
          __builtin_prefetch(&label[next]);
          __builtin_prefetch(&weight[e + OFFSET]);
        }
#endif

        if (pruneEdge(Vertex(u), e))
          continue;

        Vertex v = data.timeExpandedGraph.get(ToVertex, e);

        uint64_t entry = label[v];
        uint32_t vTime = entry >> 32;
        uint32_t vDist = static_cast<uint32_t>(entry);

        if (vTime != timeStamp) {
          vDist = UINT32_MAX;
        }

        uint32_t alt = uDist + weight[e];

        if (alt < vDist) {
          label[v] = (uint64_t(timeStamp) << 32) | alt;
          parent[v] = Vertex(u);
          Q.push(static_cast<uint32_t>(v), alt);
        }
      }

      settle(Vertex(u));
    }
  }

  const Data &data;
  std::vector<int> weight;
  std::vector<std::uint64_t> label;
  std::vector<Vertex> parent;
  std::vector<std::uint32_t> labelTimeStamp;

  radix_heap::pair_radix_heap<uint32_t, uint32_t> Q;
  std::uint32_t timeStamp;
  Profiler profiler;
};

} // namespace TE
