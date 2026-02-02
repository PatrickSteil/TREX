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

#include <omp.h>
#include <tbb/global_control.h>

#include <cmath>
#include <execution>
#include <vector>

#include "../../../DataStructures/TREX/TREXData.h"
#include "../../../ExternalLibs/ips4o/ips4o.hpp"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/String/String.h"
#include "../../TripBased/Query/Profiler.h"
#include "TransferSearchIBEs.h"
#include "Types.h"

namespace TripBased {
// IBE <=> Incomming Border Event
// Idea: use the lowest 8 bit for the stopindex, then the rest for the tripid
typedef uint32_t PackedIBE;

// Bit mask for the PackedIBE
constexpr uint32_t TRIPOFFSET = 8;

// Bit mask for the PackedIBE
constexpr uint32_t STOPINDEX_MASK = ((1 << 8) - 1);

class Builder {
public:
  Builder(TREXData &data, const int numberOfThreads = 1,
          const int pinMultiplier = 1)
      : data(data), numberOfThreads(numberOfThreads),
        pinMultiplier(pinMultiplier),
        fromStopEventId(data.stopEventGraph.numEdges()),
        edgeLabels(data.stopEventGraph.numEdges()),
        routeLabels(data.raptorData.numberOfRoutes()), seekers(), IBEs() {
    // set number of threads
    tbb::global_control c(tbb::global_control::max_allowed_parallelism,
                          numberOfThreads);
    omp_set_num_threads(numberOfThreads);

    for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
      edgeLabels[edge].stopEvent =
          StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
      edgeLabels[edge].trip =
          data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
      edgeLabels[edge].firstEvent =
          data.firstStopEventOfTrip[edgeLabels[edge].trip];

      fromStopEventId[edge] = StopEventId(from);
    }

    for (const RouteId route : data.raptorData.routes()) {
      const size_t numberOfStops = data.numberOfStopsInRoute(route);
      const size_t numberOfTrips = data.raptorData.numberOfTripsInRoute(route);
      const RAPTOR::StopEvent *stopEvents =
          data.raptorData.firstTripOfRoute(route);
      routeLabels[route].numberOfTrips = numberOfTrips;
      routeLabels[route].departureTimes.resize((numberOfStops - 1) *
                                               numberOfTrips);
      for (size_t trip = 0; trip < numberOfTrips; trip++) {
        for (size_t stopIndex = 0; stopIndex + 1 < numberOfStops; stopIndex++) {
          routeLabels[route]
              .departureTimes[(stopIndex * numberOfTrips) + trip] =
              stopEvents[(trip * numberOfStops) + stopIndex].departureTime;
        }
      }
    }

    seekers.reserve(numberOfThreads);
    for (int i = 0; i < numberOfThreads; ++i)
      seekers.emplace_back(data, fromStopEventId, edgeLabels, routeLabels);

    profiler.registerMetrics({METRIC_TREX_COLLECTED_IBES});
    profiler.registerPhases({
        PHASE_TREX_COLLECT_IBES,
        PHASE_TREX_SORT_IBES,
        PHASE_TREX_FILTER_IBES,
    });
  }

  inline void collectAllIBEsOnLowestLevel() noexcept {
    profiler.startPhase();
    IBEs.reserve(data.numberOfStopEvents());

    // TODO to only allow 'time range based' IBE
    /* int minTime = 7 * 60 * 60; */
    /* int maxTime = 8 * 60 * 60; */

    auto inSameCell = [&](auto a, auto b) {
      return (data.getCellIdOfStop(a) == data.getCellIdOfStop(b));
    };

    auto tripTooEarly = [&]([[maybe_unused]] auto trip,
                            [[maybe_unused]] auto stopIndex) {
      /* auto& event = data.getStopEvent(trip, stopIndex); */
      /* return minTime > event.departureTime; */
      return false;
    };

    auto tripTooLate = [&]([[maybe_unused]] auto trip,
                           [[maybe_unused]] auto stopIndex) {
      /* auto& event = data.getStopEvent(trip, stopIndex); */
      /* return event.departureTime > maxTime; */
      return false;
    };

    for (StopId stop(0); stop < data.numberOfStops(); ++stop) {
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

        // check if neighbour is *not* in the same cell
        if (!inSameCell(stop,
                        data.raptorData.stopOfRouteSegment(neighbourSeg))) {
          // add all stop events of this route
          for (TripId trip : data.tripsOfRoute(route.routeId)) {
            if (tripTooEarly(trip, StopIndex(route.stopIndex - 1)))
              continue;
            if (tripTooLate(trip, StopIndex(route.stopIndex - 1)))
              break;
            profiler.countMetric(METRIC_TREX_COLLECTED_IBES);
            IBEs.push_back((trip << TRIPOFFSET) | (route.stopIndex - 1));
          }
        }
      }
    }
    profiler.donePhase(PHASE_TREX_COLLECT_IBES);
  }

  inline void filterIrrelevantIBEs(uint8_t level) {
    // 'level' denotes the level, for which we filter the IBEs
    // ... in other word: after this method, IBEs should contain all IBEs which
    // cross at this level, not lower levels

    profiler.startPhase();
    IBEs.erase(std::remove_if(
                   std::execution::par, IBEs.begin(), IBEs.end(),
                   [&](PackedIBE ibe) {
                     auto trip = TripId(ibe >> TRIPOFFSET);
                     auto stopIndex = StopIndex(ibe & STOPINDEX_MASK);

                     auto fromStop = data.getStop(trip, stopIndex);
                     auto toStop = data.getStop(trip, StopIndex(stopIndex + 1));
                     return !((data.getCellIdOfStop(fromStop) ^
                               data.getCellIdOfStop(toStop)) >>
                              level);
                   }),
               IBEs.end());
    profiler.donePhase(PHASE_TREX_FILTER_IBES);
  }

  template <bool SORT_IBES = true, bool VERBOSE = true>
  inline void run() noexcept {
    profiler.start();
    collectAllIBEsOnLowestLevel();

    assert(!IBEs.empty());

    if (SORT_IBES) {
      profiler.startPhase();
      /* std::sort(std::execution::par, IBEs.begin(), IBEs.end()); */
      ips4o::parallel::sort(IBEs.begin(), IBEs.end());
      profiler.donePhase(PHASE_TREX_SORT_IBES);
    }

    const int numCores = numberOfCores();

    // now for every level, we have an invariant: IBEs contains exactly the IBEs
    // we need on this level
    for (uint8_t level(0); level < data.getNumberOfLevels(); ++level) {
      if (VERBOSE)
        std::cout << "Starting Level " << (int)level
                  << " [IBEs: " << IBEs.size() << "]... " << std::endl;

      Progress progress(IBEs.size());

#pragma omp parallel
      {
#pragma omp for schedule(dynamic)
        for (size_t i = 0; i < IBEs.size(); ++i) {
          int threadId = omp_get_thread_num();
          pinThreadToCoreId((threadId * pinMultiplier) % numCores);
          AssertMsg(omp_get_num_threads() == numberOfThreads,
                    "Number of threads is " << omp_get_num_threads()
                                            << ", but should be "
                                            << numberOfThreads << "!");

          auto &values = IBEs[i];
          seekers[threadId].run(TripId(values >> TRIPOFFSET),
                                StopIndex(values & STOPINDEX_MASK), level);
          ++progress;
        }
      }

      progress.finished();

      if (level < data.getNumberOfLevels() - 1)
        filterIrrelevantIBEs(level + 1);

      if (VERBOSE)
        std::cout << "done!\n";
    }
    profiler.done();
  }

  inline AggregateProfiler &getProfiler() noexcept { return profiler; }

  TREXData &data;
  const int numberOfThreads;
  const int pinMultiplier;

  // same as FromVertex, but for the stopEventGraph, it is not defined
  // we need to extract quickly the event from which the transfer was possible
  std::vector<StopEventId> fromStopEventId;

  std::vector<EdgeLabel> edgeLabels;
  std::vector<RouteLabel> routeLabels;

  std::vector<TransferSearch<TripBased::NoProfiler>> seekers;
  std::vector<PackedIBE> IBEs;
  AggregateProfiler profiler;
};
} // namespace TripBased
