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

#include <algorithm>

#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/Timer.h"
#include "ShortcutSearch.h"

namespace RAPTOR::ULTRA {

template <bool DEBUG = false, bool COUNT_OPTIMAL_CANDIDATES = false,
          bool IGNORE_ISOLATED_CANDIDATES = false>
class Builder {
 public:
  inline static constexpr bool Debug = DEBUG;
  inline static constexpr bool CountOptimalCandidates =
      COUNT_OPTIMAL_CANDIDATES;
  inline static constexpr bool IgnoreIsolatedCandidates =
      IGNORE_ISOLATED_CANDIDATES;
  using Type = Builder<Debug, CountOptimalCandidates, IgnoreIsolatedCandidates>;

 public:
  Builder(const Data& data) : data(data) {
    shortcutGraph.addVertices(data.numberOfStops());
    for (const Vertex vertex : shortcutGraph.vertices()) {
      shortcutGraph.set(Coordinates, vertex,
                        data.transferGraph.get(Coordinates, vertex));
    }
  }

  void computeShortcuts(const ThreadPinning& threadPinning,
                        const int witnessTransferLimit = 15 * 60,
                        const int minDepartureTime = -never,
                        const int maxDepartureTime = never,
                        const bool verbose = true) noexcept {
    if (verbose)
      std::cout << "Computing shortcuts with " << threadPinning.numberOfThreads
                << " threads." << std::endl;

    size_t optimalCandidates = 0;
    Progress progress(data.numberOfStops(), verbose);
    omp_set_num_threads(threadPinning.numberOfThreads);
#pragma omp parallel
    {
      threadPinning.pinThread();

      DynamicTransferGraph localShortcutGraph = shortcutGraph;
      ShortcutSearch<Debug, CountOptimalCandidates, IgnoreIsolatedCandidates>
          shortcutSearch(data, localShortcutGraph, witnessTransferLimit);

#pragma omp for schedule(dynamic)
      for (size_t i = 0; i < data.numberOfStops(); i++) {
        shortcutSearch.run(StopId(i), minDepartureTime, maxDepartureTime);
        progress++;
      }

#pragma omp critical
      {
        if constexpr (CountOptimalCandidates) {
          optimalCandidates += shortcutSearch.getNumberOfOptimalCandidates();
        }
        for (const Vertex from : shortcutGraph.vertices()) {
          for (const Edge edge : localShortcutGraph.edgesFrom(from)) {
            const Vertex to = localShortcutGraph.get(ToVertex, edge);
            if (!shortcutGraph.hasEdge(from, to)) {
              shortcutGraph.addEdge(from, to).set(
                  TravelTime, localShortcutGraph.get(TravelTime, edge));
            } else {
              AssertMsg(shortcutGraph.get(TravelTime,
                                          shortcutGraph.findEdge(from, to)) ==
                            localShortcutGraph.get(TravelTime, edge),
                        "Edge from "
                            << from << " to " << to
                            << " has inconclusive travel time ("
                            << shortcutGraph.get(
                                   TravelTime, shortcutGraph.findEdge(from, to))
                            << ", " << localShortcutGraph.get(TravelTime, edge)
                            << ")");
            }
          }
        }
      }
    }
    progress.finished();
    if constexpr (CountOptimalCandidates) {
      std::cout << "#Optimal candidates: "
                << String::prettyInt(optimalCandidates) << std::endl;
    } else {
      suppressUnusedParameterWarning(optimalCandidates);
    }
  }

  inline const DynamicTransferGraph& getShortcutGraph() const noexcept {
    return shortcutGraph;
  }

  inline DynamicTransferGraph& getShortcutGraph() noexcept {
    return shortcutGraph;
  }

 private:
  const Data& data;
  DynamicTransferGraph shortcutGraph;
};

}  // namespace RAPTOR::ULTRA
