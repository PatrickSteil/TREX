/**********************************************************************************

 Copyright (c) 2023-2025 Patrick Steil

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

// Builder for Number Of Cells = 2 on all levels

#include <cmath>
#include <vector>

#include "../../../DataStructures/TREX/TREXData.h"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/String/String.h"
#include "../../TripBased/Query/Profiler.h"
#include "TransferSearch.h"

namespace TripBased {

class Builder {
public:
  Builder(TREXData &data) : data(data), search(data), stopEvents() {}

  void collectUsingMasks(const uint64_t LEVELMASK, const uint64_t TARGETMASK) {
    /* profiler.startPhase(); */
    auto masksMatches = [&](StopId stop) {
      return (data.getCellIdOfStop(stop) & LEVELMASK) == TARGETMASK;
    };

    for (StopId stop(0); stop < data.numberOfStops(); ++stop) {
      // does the global id match the mask and does it have a neighbor where the
      // mask does not match?
      if (!masksMatches(stop))
        continue;

      // check the neighbors in compact layoutGraph
      AssertMsg(data.layoutGraph.isVertex(stop),
                "Stop is not in the layoutgraph");

      for (const RAPTOR::RouteSegment &route :
           data.routesContainingStop(stop)) {
        // EDGE CASE: a stop is not a border stop (of a route) if it's at either
        // end (start or end)
        if (route.stopIndex == 0)
          continue;

        // check if the next / previous stop in stop array of route is in
        // another cell
        RAPTOR::RouteSegment neighbourSeg(route.routeId,
                                          StopIndex(route.stopIndex - 1));

        // check if neighbour in same cell
        if (!masksMatches(data.raptorData.stopOfRouteSegment(neighbourSeg))) {
          // add all stop events of this route
          for (TripId trip : data.tripsOfRoute(route.routeId)) {
            /* profiler.countMetric(COLLECTED_STOPEVENTS); */
            stopEvents.push_back(
                std::make_pair(trip, StopIndex(route.stopIndex - 1)));
          }
        }
      }
    }
    /* profiler.donePhase(COLLECT_STOPEVENTS); */
  }

  void printInfo() {
    /* profiler.printStatisticsAsCSV(); */
    search.getProfiler().printStatisticsAsCSV();
  }

  void run(const uint64_t LEVELMASK, const uint64_t TARGETMASK) {
    collectUsingMasks(LEVELMASK, TARGETMASK);

    /* profiler.startPhase(); */
    for (auto &element : stopEvents) {
      search.run(element.first, element.second, LEVELMASK, TARGETMASK);
    }

    /* profiler.donePhase(SCAN_THE_CELL_INSIDE); */
    stopEvents.clear();
  }

  TREXData &data;
  TransferSearch<TripBased::AggregateProfiler> search;
  std::vector<std::pair<TripId, StopIndex>> stopEvents;

  /* AggregateProfiler profiler; */
};

inline void Customize(TREXData &data, const bool verbose = true) {
  data.createCompactLayoutGraph();
  data.addInformationToStopEventGraph();

  uint64_t LEVELMASK = (~0);
  uint64_t TARGETMASK = 0;

  Builder bobTheBuilder(data);

  for (int level(0); level < data.getNumberOfLevels(); ++level) {
    int numberOfCellsOnThisLevel = (1 << (data.getNumberOfLevels() - level));

    if (verbose)
      std::cout << "**** Level: " << level << ", " << numberOfCellsOnThisLevel
                << " cells! ****" << std::endl;

    Progress progress(numberOfCellsOnThisLevel);

    // get all valid TARGETMASKs
    for (int target(0); target < numberOfCellsOnThisLevel; ++target) {
      TARGETMASK = target << level;
      bobTheBuilder.run(LEVELMASK, TARGETMASK);

      ++progress;
    }

    /* search.addCollectShortcuts(); */
    progress.finished();

    if (verbose) {
      std::cout << "##### Stats for Level " << level << std::endl;
      bobTheBuilder.printInfo();
    }
    bobTheBuilder.search.getProfiler().reset();
    bobTheBuilder.search.resetStats();

    LEVELMASK <<= 1;
  }
}

inline void Customize(TREXData &data, const int numberOfThreads,
                      const int pinMultiplier = 1, const bool verbose = true) {
  data.createCompactLayoutGraph();
  data.addInformationToStopEventGraph();

  uint64_t LEVELMASK = (~0);
  uint64_t TARGETMASK = 0;

  const int numCores = numberOfCores();
  omp_set_num_threads(numberOfThreads);

  for (int level(0); level < data.getNumberOfLevels(); ++level) {
    int numberOfCellsOnThisLevel = (1 << (data.getNumberOfLevels() - level));
    if (verbose)
      std::cout << "**** Level: " << level << ", " << numberOfCellsOnThisLevel
                << " cells! ****" << std::endl;

    Progress progress(numberOfCellsOnThisLevel);

#pragma omp parallel
    {
      int threadId = omp_get_thread_num();
      pinThreadToCoreId((threadId * pinMultiplier) % numCores);
      AssertMsg(omp_get_num_threads() == numberOfThreads,
                "Number of threads is " << omp_get_num_threads()
                                        << ", but should be " << numberOfThreads
                                        << "!");

      Builder bobTheBuilder(data);

// get all valid TARGETMASKs
#pragma omp for schedule(dynamic, 1)
      for (int target = 0; target < numberOfCellsOnThisLevel; ++target) {
        TARGETMASK = target << level;
        bobTheBuilder.run(LEVELMASK, TARGETMASK);
        ++progress;
      }

      bobTheBuilder.search.getProfiler().reset();
      bobTheBuilder.search.resetStats();
    }

    LEVELMASK <<= 1;
    progress.finished();
  }
}
} // namespace TripBased
