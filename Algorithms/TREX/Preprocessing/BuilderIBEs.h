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

#include <atomic>
#include <cmath>
#include <execution>
#include <omp.h>
#include <tbb/global_control.h>
#include <vector>

#include "../../../DataStructures/Container/AtomicBool.h"
#include "../../../DataStructures/TREX/TREXData.h"
#include "../../../ExternalLibs/ips4o/ips4o.hpp"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/String/String.h"
#include "../../TripBased/Query/Profiler.h"
#include "../../TripBased/Query/Types.h"
#include "TransferSearchIBEs.h"

namespace TripBased {

struct PackedIBE {
  uint32_t tripId : 24;
  uint32_t stopIndex : 8;

  constexpr PackedIBE(const TripId trip = noTripId,
                      const StopIndex stop = noStopIndex)
      : tripId(static_cast<uint32_t>(trip)),
        stopIndex(static_cast<uint32_t>(stop)) {}

  constexpr TripId getTripId() const noexcept { return TripId(tripId); }

  constexpr StopIndex getStopIndex() const noexcept {
    return StopIndex(stopIndex);
  }

  friend constexpr bool operator<(const PackedIBE &a,
                                  const PackedIBE &b) noexcept {
    return (static_cast<uint32_t>(a.tripId) << 8 | a.stopIndex) <
           (static_cast<uint32_t>(b.tripId) << 8 | b.stopIndex);
  }

  friend constexpr bool operator==(const PackedIBE &a,
                                   const PackedIBE &b) noexcept {
    return a.tripId == b.tripId && a.stopIndex == b.stopIndex;
  }
};

static_assert(sizeof(PackedIBE) == 4);
static_assert(std::is_trivially_copyable_v<PackedIBE>);

class Builder {
public:
  Builder(TREXData &data, const int numberOfThreads = 1,
          const int pinMultiplier = 1)
      : data(data), numberOfThreads(numberOfThreads),
        pinMultiplier(pinMultiplier),
        edgeLabels(data.stopEventGraph.numEdges()),
        routeLabels(data.raptorData.numberOfRoutes()),
        cellIdOfEvent(data.numberOfStopEvents()), seekers(), IBEs(),
        updatedCell((1 << data.numberOfLevels)),
        latestArrTimeOfAffectedCell((1 << data.numberOfLevels), 0) {
    tbb::global_control c(tbb::global_control::max_allowed_parallelism,
                          numberOfThreads);
    omp_set_num_threads(numberOfThreads);

    for (size_t event = 0; event < data.numberOfStopEvents(); ++event) {
      const StopId stop = data.getStopOfStopEvent(StopEventId(event));
      AssertMsg(data.raptorData.isStop(Vertex(stop)), "Stop is not a stop!");
      cellIdOfEvent[event] = (uint16_t)data.getCellIdOfStop(stop);
    }

    for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
      const StopEventId toStopEvent =
          StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
      AssertMsg(toStopEvent < data.numberOfStopEvents(),
                "StopEventId is out of bounds?");
      edgeLabels[edge].setTrip(
          data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)]);
      edgeLabels[edge].setFirstEvent(
          data.firstStopEventOfTrip[edgeLabels[edge].getTrip()]);
      edgeLabels[edge].setStopIndex(
          StopIndex(toStopEvent - edgeLabels[edge].getFirstEvent()));

      AssertMsg(cellIdOfEvent[from] == cellIdOfEvent[toStopEvent - 1],
                "CellIDs should change during transfer!");
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
      seekers.emplace_back(data, edgeLabels, routeLabels, cellIdOfEvent,
                           updatedCell);

    profiler.registerMetrics({METRIC_TREX_COLLECTED_IBES});
    profiler.registerPhases({
        PHASE_TREX_COLLECT_IBES,
        PHASE_TREX_PREPARE_AFFECTED_CELLS,
        PHASE_TREX_SORT_IBES,
        PHASE_TREX_FILTER_IBES,
    });
  }

  inline void prepareLookup(const std::vector<TripId> &affectedTrips) noexcept {
    profiler.startPhase();
    latestArrTimeOfAffectedCell.assign((1 << data.numberOfLevels), 0);

    for (auto &value : updatedCell) {
      value.set_true();
    }

    for (const auto trip : affectedTrips) {
      for (StopEventId runner = data.firstStopEventOfTrip[trip];
           runner < data.firstStopEventOfTrip[trip + 1]; ++runner) {
        const std::uint16_t cId = cellIdOfEvent[runner];
        AssertMsg(static_cast<std::size_t>(cId) < updatedCell.size(),
                  "CellId of event should be in bounds!");
        updatedCell[cId].set_true();

        latestArrTimeOfAffectedCell[cId] = std::max(
            latestArrTimeOfAffectedCell[cId],
            static_cast<std::uint32_t>(data.arrivalEvents[runner].arrivalTime));
      }
    }

#ifndef NDEBUG
    for (std::size_t i = 0; i < updatedCell.size(); ++i) {
      if (updatedCell[i].getValue()) {
        AssertMsg(latestArrTimeOfAffectedCell[i] > 0,
                  "LatestArrTime should not be 0 for an affected cell! ( "
                      << (int)latestArrTimeOfAffectedCell[i] << " )");
      }
    }
#endif

    profiler.donePhase(PHASE_TREX_PREPARE_AFFECTED_CELLS);
  }

  inline void collectAllIBEs() noexcept {
    profiler.startPhase();

    IBEs.reserve(data.numberOfStopEvents());

    auto sameCellId = [&](auto a, auto b) {
      return (data.getCellIdOfStop(a) == data.getCellIdOfStop(b));
    };

    for (StopId stop(0); stop < data.numberOfStops(); ++stop) {
      for (const RAPTOR::RouteSegment &route :
           data.routesContainingStop(stop)) {
        if (route.stopIndex == 0)
          continue;

        RAPTOR::RouteSegment neighbourSeg(route.routeId,
                                          StopIndex(route.stopIndex - 1));

        const StopId prevStop =
            data.raptorData.stopOfRouteSegment(neighbourSeg);

        if (!sameCellId(prevStop, stop)) {
          for (TripId trip : data.tripsOfRoute(route.routeId)) {
            profiler.countMetric(METRIC_TREX_COLLECTED_IBES);
            IBEs.emplace_back(trip, StopIndex(route.stopIndex - 1));
          }
        }
      }
    }
    profiler.donePhase(PHASE_TREX_COLLECT_IBES);
  }

  inline void filterIrrelevantIBEs(uint8_t level) {
    profiler.startPhase();
    IBEs.erase(std::remove_if(
                   std::execution::par, IBEs.begin(), IBEs.end(),
                   [&](const PackedIBE &ibe) {
                     auto trip = ibe.getTripId();
                     auto stopIndex = ibe.getStopIndex();
                     StopEventId firstEvent = data.firstStopEventOfTrip[trip];

                     const uint16_t fromCellId =
                         cellIdOfEvent[firstEvent + stopIndex];
                     const uint16_t toCellId =
                         cellIdOfEvent[firstEvent + stopIndex + 1];

                     // TODO a) checkl that load is cheap

                     const uint16_t parentCellId = (toCellId >> level);

                     assert(parentCellId < updatedCell.size());
                     bool keep = ((fromCellId ^ toCellId) >> level) &&
                                 updatedCell[parentCellId].getValue();
                     return !keep;
                   }),
               IBEs.end());
    profiler.donePhase(PHASE_TREX_FILTER_IBES);
  }

  // we processed level-1, now prepare for level
  inline void bubbleUpLatestArrivalTime(uint8_t level) {
    assert(level <= data.numberOfLevels);
    const std::size_t nrCells = (1 << (data.numberOfLevels - level - 1));

    for (std::size_t cId = 0; cId < nrCells; ++cId) {
      const auto prevValue = latestArrTimeOfAffectedCell[cId];
      latestArrTimeOfAffectedCell[cId] = 0;
      const std::size_t parentId = (cId >> level);
      latestArrTimeOfAffectedCell[parentId] =
          std::max(latestArrTimeOfAffectedCell[parentId], prevValue);
    }
  }

  template <bool SORT_IBES = true, bool VERBOSE = true>
  inline void run(const std::vector<TripId> &affectedTrips) noexcept {
    profiler.start();
    prepareLookup(affectedTrips);
    /* collectAffectedIBEs(affectedTrips); */
    collectAllIBEs();

    assert(!IBEs.empty());
    {
      if (SORT_IBES) {
        profiler.startPhase();
        /* std::sort(std::execution::par, IBEs.begin(), IBEs.end(), [](const
         * PackedIBE &a, const PackedIBE &b) { return a < b; }); */
        ips4o::parallel::sort(
            IBEs.begin(), IBEs.end(),
            [](const PackedIBE &a, const PackedIBE &b) { return a < b; });
        profiler.donePhase(PHASE_TREX_SORT_IBES);
      }
    }

    std::vector<PackedIBE> allLowerIBEs(IBEs);

    const int numCores = numberOfCores();

    auto filterIBEs = [&](const uint8_t level) {
      for (const auto &ibe : allLowerIBEs) {
        const StopEventId fromEvent =
            data.getStopEventId(ibe.getTripId(), ibe.getStopIndex());
        /* collect IBE on current level iff
        1) it is an IBE @ level

        2) cell@level is marked as updated => collect all IBEs for this
        cell@level

        3) departureTime(IBE) <= latest possible arrival time of
        cell@level verify arrival time pruning for the cell on current level
        */
        auto fromCellId = cellIdOfEvent[fromEvent];
        auto toCellId = cellIdOfEvent[fromEvent + 1];
        auto parentCellId = toCellId >> level;

        // 1)
        bool keep = ((fromCellId ^ toCellId) >> level) != 0;
        // 2)
        keep &= updatedCell[parentCellId].getValue();

        /* keep &= ((uint32_t)data.departureTime(event) > */
        /*          latestArrTimeOfAffectedCell[cell]); */

        if (keep) {
          IBEs.emplace_back(ibe.getTripId(), ibe.getStopIndex());
        }
      }
    };

    for (uint8_t level(0); level < data.getNumberOfLevels(); ++level) {
      if (VERBOSE)
        std::cout << "Starting Level " << (int)level
                  << " [IBEs: " << IBEs.size() << "]... " << std::endl;

      IBEs.clear();
      filterIBEs(level);

      for (auto &value : updatedCell) {
        value.reset();
      }

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

          const auto &ibe = IBEs[i];
          seekers[threadId].run(ibe.getTripId(), ibe.getStopIndex(), level);
          ++progress;
        }
      }

      progress.finished();

      if (level < data.getNumberOfLevels() - 1) {
        bubbleUpLatestArrivalTime(level + 1);
      }

      if (VERBOSE) {
        std::cout << "done!\n";
      }
    }

    const std::size_t numEdges = data.stopEventGraph.numEdges();
#pragma omp parallel for
    for (Edge edge = Edge(0); edge < numEdges; ++edge) {
      data.stopEventGraph.set(LocalLevel, edge, edgeLabels[edge].getRank());
    }

    profiler.done();
  }

  template <bool SORT_IBES = true, bool VERBOSE = true>
  inline void run() noexcept {
    profiler.start();
    collectAllIBEs();

    assert(!IBEs.empty());

    if (SORT_IBES) {
      profiler.startPhase();
      /* std::sort(std::execution::par, IBEs.begin(), IBEs.end(), [](const
       * PackedIBE &a, const PackedIBE &b) { return a < b; }); */
      ips4o::parallel::sort(
          IBEs.begin(), IBEs.end(),
          [](const PackedIBE &a, const PackedIBE &b) { return a < b; });
      profiler.donePhase(PHASE_TREX_SORT_IBES);
    }
    const int numCores = numberOfCores();

    for (uint8_t level(0); level < data.getNumberOfLevels(); ++level) {
      if (VERBOSE)
        std::cout << "Starting Level " << (int)level
                  << " [IBEs: " << IBEs.size() << "]... " << std::endl;

      for (auto &value : updatedCell) {
        value.reset();
      }

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

          const auto &ibe = IBEs[i];
          seekers[threadId].run(ibe.getTripId(), ibe.getStopIndex(), level);
          ++progress;
        }
      }

      progress.finished();

      if (level < data.getNumberOfLevels() - 1)
        filterIrrelevantIBEs(level + 1);

      if (VERBOSE) {
        std::cout << "done!\n";
      }
    }

    const std::size_t numEdges = data.stopEventGraph.numEdges();
#pragma omp parallel for
    for (Edge edge = Edge(0); edge < numEdges; ++edge) {
      data.stopEventGraph.set(LocalLevel, edge, edgeLabels[edge].getRank());
    }

    profiler.done();
  }

  inline AggregateProfiler &getProfiler() noexcept { return profiler; }

  TREXData &data;
  const int numberOfThreads;
  const int pinMultiplier;
  std::vector<EdgeLabel> edgeLabels;
  std::vector<RouteLabel> routeLabels;
  std::vector<uint16_t> cellIdOfEvent;

  std::vector<TransferSearch<TripBased::NoProfiler>> seekers;
  std::vector<PackedIBE> IBEs;
  AggregateProfiler profiler;

  std::vector<PaddedAtomicBool> updatedCell;
  std::vector<std::uint32_t> latestArrTimeOfAffectedCell;
};
} // namespace TripBased
