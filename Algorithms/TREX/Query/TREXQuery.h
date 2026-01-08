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

#define SHOW_DEBUG false

#include <array>
#include <bit>

#include "../../../DataStructures/Container/MultiQueue.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/Graph/Utils/Conversion.h"
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../../DataStructures/TREX/TREXData.h"
#include "../../TripBased/Query/ProfileReachedIndexSIMD.h"
#include "../../TripBased/Query/Profiler.h"

namespace TripBased {

template <typename PROFILER = NoProfiler> class TREXQuery {
public:
  using Profiler = PROFILER;
  using Type = TREXQuery<Profiler>;

private:
  struct TripLabel {
    TripLabel(const StopEventId begin = noStopEvent,
              const StopEventId end = noStopEvent, const u_int32_t parent = -1)
        : begin(begin), end(end), parent(parent) {}
    StopEventId begin;
    StopEventId end;
    u_int32_t parent;
  };

  struct EdgeRange {
    EdgeRange() : begin(noEdge), end(noEdge) {}
    Edge begin;
    Edge end;
  };

  struct EdgeLabel {
    EdgeLabel(const StopEventId firstEvent = noStopEvent,
              const TripId trip = noTripId,
              const StopEventId stopEvent = noStopEvent,
              const uint16_t cellId = 0, const uint8_t localLevel = 0)
        : firstEvent(firstEvent), trip(trip), stopEvent(stopEvent),
          cellId(cellId), localLevel(localLevel) {}
    StopEventId firstEvent;
    TripId trip;
    StopEventId stopEvent;
    uint16_t cellId;
    uint8_t localLevel;
    uint8_t hop;
  };

  struct RouteLabel {
    RouteLabel() : numberOfTrips(0) {}
    inline StopIndex end() const noexcept {
      return StopIndex(departureTimes.size() / numberOfTrips);
    }
    u_int32_t numberOfTrips;
    std::vector<int> departureTimes;
  };

  struct TargetLabel {
    TargetLabel(const int arrivalTime = INFTY, const u_int32_t parent = -1)
        : arrivalTime(arrivalTime), parent(parent) {}

    int arrivalTime;
    u_int32_t parent;
  };

public:
  TREXQuery(TREXData &data)
      : data(data), reverseTransferGraph(data.raptorData.transferGraph),
        transferFromSource(data.numberOfStops(), INFTY),
        transferToTarget(data.numberOfStops(), INFTY), lastSource(StopId(0)),
        lastTarget(StopId(0)), reachedRoutes(data.numberOfRoutes()), queue(),
        edgeRanges(data.numberOfStopEvents()), reachedIndex(data),
        targetLabels(1), minArrivalTime(INFTY),
        edgeLabels(data.stopEventGraph.numEdges()),
        routeLabels(data.numberOfRoutes()), sourceStop(noStop),
        targetStop(noStop), sourceDepartureTime(never),
        transferPerLevel(data.getNumberOfLevels() + 1, 0), numQueries(0) {
    reverseTransferGraph.revert();

    for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
      edgeLabels[edge].trip =
          data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
      edgeLabels[edge].firstEvent =
          data.firstStopEventOfTrip[edgeLabels[edge].trip];
      edgeLabels[edge].stopEvent =
          StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
      edgeLabels[edge].localLevel = data.stopEventGraph.get(LocalLevel, edge);
      edgeLabels[edge].hop = data.stopEventGraph.get(Hop, edge);
      edgeLabels[edge].cellId = ((uint16_t)data.getCellIdOfStop(
          data.getStopOfStopEvent(StopEventId(from))));
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
    profiler.registerPhases(
        {PHASE_SCAN_INITIAL, PHASE_EVALUATE_INITIAL, PHASE_SCAN_TRIPS});
    profiler.registerMetrics({METRIC_ROUNDS, METRIC_SCANNED_TRIPS,
                              METRIC_SCANNED_STOPS, METRIC_RELAXED_TRANSFERS,
                              METRIC_ENQUEUES, METRIC_ADD_JOURNEYS,
                              DISCARDED_EDGE});
  }

  inline void run(const Vertex source, const int departureTime,
                  const Vertex target) noexcept {
    AssertMsg(data.isStop(source), "Source " << source << " is not a stop!");
    AssertMsg(data.isStop(target), "Target " << target << " is not a stop!");
    run(StopId(source), departureTime, StopId(target));
  }

  inline void run(const StopId source, const int departureTime,
                  const StopId target) noexcept {
    profiler.start();
    clear();
    sourceStop = source;
    targetStop = target;
    sourceCellId = data.getCellIdOfStop(sourceStop);
    targetCellId = data.getCellIdOfStop(targetStop);
    sourceDepartureTime = departureTime;

    computeInitialAndFinalTransfers();
    evaluateInitialTransfers();
    scanTrips(16);
    profiler.done();
  }

  inline int getEarliestArrivalTime() const noexcept {
    return targetLabels.back().arrivalTime;
  }

  inline int getEarliestArrivalNumberOfTrips() const noexcept {
    const int eat = targetLabels.back().arrivalTime;
    for (size_t i = 0; i < targetLabels.size(); ++i) {
      if (targetLabels[i].arrivalTime == eat)
        return i;
    }
    return -1;
  }

  inline std::vector<RAPTOR::Journey> getJourneys() const noexcept {
    std::vector<RAPTOR::Journey> result;
    int bestArrivalTime = INFTY;
    for (const TargetLabel &label : targetLabels) {
      if (label.arrivalTime >= bestArrivalTime)
        continue;
      bestArrivalTime = label.arrivalTime;
      result.emplace_back(getJourney(label));
    }
    return result;
  }

  inline std::vector<RAPTOR::ArrivalLabel> getArrivals() const noexcept {
    std::vector<RAPTOR::ArrivalLabel> result;
    for (size_t i = 0; i < targetLabels.size(); ++i) {
      if (targetLabels[i].arrivalTime >= INFTY)
        continue;
      if ((result.size() >= 1) &&
          (result.back().arrivalTime == targetLabels[i].arrivalTime))
        continue;
      result.emplace_back(targetLabels[i].arrivalTime, i);
    }
    return result;
  }

  inline Profiler &getProfiler() noexcept { return profiler; }

  inline void showTransferLevels() noexcept {
    std::cout << "# of relaxed transfers per level" << std::endl;

    for (size_t level(0); level < transferPerLevel.size(); ++level)
      std::cout << level << "\t"
                << (double)transferPerLevel[level] / (double)numQueries
                << std::endl;
  }

private:
  inline void clear() noexcept {
    queue.clear_all();
    reachedIndex.clear();
    targetLabels.resize(1);
    targetLabels[0] = TargetLabel();
    minArrivalTime = INFTY;

    ++numQueries;
  }

  inline void computeInitialAndFinalTransfers() noexcept {
    profiler.startPhase();
    transferFromSource[lastSource] = INFTY;
    for (const Edge edge :
         data.raptorData.transferGraph.edgesFrom(lastSource)) {
      const Vertex stop = data.raptorData.transferGraph.get(ToVertex, edge);
      transferFromSource[stop] = INFTY;
    }
    transferToTarget[lastTarget] = INFTY;
    for (const Edge edge : reverseTransferGraph.edgesFrom(lastTarget)) {
      const Vertex stop = reverseTransferGraph.get(ToVertex, edge);
      transferToTarget[stop] = INFTY;
    }
    transferFromSource[sourceStop] = 0;
    for (const Edge edge :
         data.raptorData.transferGraph.edgesFrom(sourceStop)) {
      const Vertex stop = data.raptorData.transferGraph.get(ToVertex, edge);
      transferFromSource[stop] =
          data.raptorData.transferGraph.get(TravelTime, edge);
    }
    transferToTarget[targetStop] = 0;
    if (sourceStop == targetStop) [[unlikely]]
      addTargetLabel(sourceDepartureTime);
    for (const Edge edge : reverseTransferGraph.edgesFrom(targetStop)) {
      const Vertex stop = reverseTransferGraph.get(ToVertex, edge);
      if (stop == sourceStop) [[unlikely]]
        addTargetLabel(sourceDepartureTime +
                       reverseTransferGraph.get(TravelTime, edge));
      transferToTarget[stop] = reverseTransferGraph.get(TravelTime, edge);
    }
    lastSource = sourceStop;
    lastTarget = targetStop;
    profiler.donePhase(PHASE_SCAN_INITIAL);
  }

  inline void evaluateInitialTransfers() noexcept {
    profiler.startPhase();
    reachedRoutes.clear();
    for (const RAPTOR::RouteSegment &route :
         data.raptorData.routesContainingStop(sourceStop)) {
      reachedRoutes.insert(route.routeId);
    }
    for (const Edge edge :
         data.raptorData.transferGraph.edgesFrom(sourceStop)) {
      const Vertex stop = data.raptorData.transferGraph.get(ToVertex, edge);
      for (const RAPTOR::RouteSegment &route :
           data.raptorData.routesContainingStop(StopId(stop))) {
        reachedRoutes.insert(route.routeId);
      }
    }
    reachedRoutes.sort();
    auto &routesToLoopOver = reachedRoutes.getValues();
    for (size_t i(0); i < routesToLoopOver.size(); ++i) {
      const RouteId route = routesToLoopOver[i];

#ifdef ENABLE_PREFETCH
      if (i + 4 < routesToLoopOver.size()) {
        __builtin_prefetch(&routeLabels[routesToLoopOver[i + 4]]);
        __builtin_prefetch(&data.firstTripOfRoute[routesToLoopOver[i + 4]]);
        __builtin_prefetch(
            data.raptorData.stopArrayOfRoute(routesToLoopOver[i + 4]));
      }
#endif
      const RouteLabel &label = routeLabels[route];
      const StopIndex endIndex = label.end();
      const TripId firstTrip = data.firstTripOfRoute[route];
      const StopId *stops = data.raptorData.stopArrayOfRoute(route);
      TripId tripIndex = noTripId;
      for (StopIndex stopIndex(0); stopIndex < endIndex; stopIndex++) {
        const int timeFromSource = transferFromSource[stops[stopIndex]];
        if (timeFromSource == INFTY)
          continue;
        const int stopDepartureTime = sourceDepartureTime + timeFromSource;
        const u_int32_t labelIndex = stopIndex * label.numberOfTrips;
        if (tripIndex >= label.numberOfTrips) {
          tripIndex = std::lower_bound(
              TripId(0), TripId(label.numberOfTrips), stopDepartureTime,
              [&](const TripId trip, const int time) {
                return label.departureTimes[labelIndex + trip] < time;
              });
          if (tripIndex >= label.numberOfTrips)
            continue;
        } else {
          if (label.departureTimes[labelIndex + tripIndex - 1] <
              stopDepartureTime)
            continue;
          --tripIndex;
          while ((tripIndex > 0) &&
                 (label.departureTimes[labelIndex + tripIndex - 1] >=
                  stopDepartureTime)) {
            --tripIndex;
          }
        }
        enqueue(firstTrip + tripIndex, StopIndex(stopIndex + 1));
        if (tripIndex == 0)
          break;
      }
    }
    profiler.donePhase(PHASE_EVALUATE_INITIAL);
  }

  inline void scanTrips(const uint8_t MAX_ROUNDS = 16) noexcept {
    profiler.startPhase();
    u_int8_t currentRound = 0;
    while (currentRound < MAX_ROUNDS &&
           queue.laterQueueHasElement(currentRound)) {
      if (SHOW_DEBUG)
        std::cout << "CurrentRound " << (int)currentRound << ", "
                  << queue.size(currentRound) << std::endl;
      profiler.countMetric(METRIC_ROUNDS);
      targetLabels.emplace_back(targetLabels.back());

      while (!queue.empty(currentRound)) {
        TripLabel &label = queue.front(currentRound);
        profiler.countMetric(METRIC_SCANNED_TRIPS);
        for (StopEventId j = label.begin; j < label.end; j++) {
          profiler.countMetric(METRIC_SCANNED_STOPS);
          if (data.arrivalEvents[j].arrivalTime >= minArrivalTime) {
            label.end = j;
            break;
          }
          const int timeToTarget = transferToTarget[data.arrivalEvents[j].stop];
          if (timeToTarget != INFTY) {
            addTargetLabel(data.arrivalEvents[j].arrivalTime + timeToTarget,
                           -1);
          }
        }

        // do not relax into a 17'th round or some
        if (currentRound + 1 < MAX_ROUNDS) {
          // TODO unroll later
          for (StopEventId j = label.begin; j < label.end; ++j) {
            const auto edgeRangeBegin =
                data.stopEventGraph.beginEdgeFrom(Vertex(j));
            const auto edgeRangeEnd =
                data.stopEventGraph.beginEdgeFrom(Vertex(j + 1));
            for (Edge edge = edgeRangeBegin; edge < edgeRangeEnd; edge++) {
              profiler.countMetric(METRIC_RELAXED_TRANSFERS);
              enqueue(edge, -1, currentRound, j);
            }
          }
        }
        queue.pop(currentRound);
      }

      ++currentRound;
    }
    profiler.donePhase(PHASE_SCAN_TRIPS);
  }

  inline void enqueue(const TripId trip, const StopIndex index) noexcept {
    profiler.countMetric(METRIC_ENQUEUES);

    if (SHOW_DEBUG) {
      std::cout << "Enqueue trip " << (int)trip << " at index " << (int)index
                << " to 0 queue!\n";
      std::cout << "\tReached Index: " << (int)reachedIndex(trip, 1) << "\n";
    }

    if (reachedIndex.alreadyReached(trip, index, 1))
      return;
    const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
    queue.push(0, TripLabel(StopEventId(firstEvent + index),
                            StopEventId(firstEvent + reachedIndex(trip, 1))));
    reachedIndex.update(trip, index, 1);
  }

  inline void enqueue(const Edge edge, const size_t parent,
                      const int currentRound, const StopEventId from) noexcept {
    assert(currentRound + 1 < 16);
    profiler.countMetric(METRIC_ENQUEUES);
    const EdgeLabel &label = edgeLabels[edge];

    if (currentRound + label.hop >= 16)
      return;

    if (data.arrivalEvents[label.stopEvent].arrivalTime >= minArrivalTime)
      return;

    if (reachedIndex.alreadyReached(label.trip,
                                    label.stopEvent - label.firstEvent,
                                    currentRound + label.hop)) [[likely]]
      return;

    if (SHOW_DEBUG) {
      std::cout << "Current Round: " << (int)currentRound << " .. ";
      std::cout << "Transfer from " << (int)from << " to "
                << (int)label.stopEvent << std::endl;
      std::cout << "Enqueue trip " << (int)label.trip << " at index "
                << (int)(label.stopEvent - label.firstEvent) << ", route "
                << (int)data.routeOfTrip[label.trip] << "\n";
      std::cout << "\tLabel Stop "
                << (int)data.getStopOfStopEvent(StopEventId(label.stopEvent))
                << " and level " << std::bitset<16>(label.localLevel)
                << " and hop " << (int)label.hop << "\n";
      std::cout << "\tReached Index: "
                << (int)reachedIndex(label.trip, currentRound + label.hop)
                << "\n";
    }

    int lclSource = 16 - std::countl_zero(static_cast<uint16_t>(label.cellId ^
                                                                sourceCellId));
    int lclTarget = 16 - std::countl_zero(static_cast<uint16_t>(label.cellId ^
                                                                targetCellId));

    auto lcl = std::min(lclSource, lclTarget);
    if (SHOW_DEBUG) {
      std::cout << "Query s: " << sourceStop << " -> t: " << targetStop
                << ", s : "
                << data.getStopOfStopEvent(StopEventId(label.stopEvent - 1))
                << ", lcl: " << (int)lcl << ", sourceLcl: " << (int)lclSource
                << ", targetLcl: " << (int)lclTarget << std::endl;
    }

    assert(label.hop < 16);
    assert(label.hop > 0);
    assert(label.hop + currentRound > 0);

    if (lcl > 0 && !(label.localLevel & (1 << (lcl - 1)))) {
      profiler.countMetric(DISCARDED_EDGE);
      reachedIndex.update(label.trip,
                          StopIndex(label.stopEvent - label.firstEvent),
                          currentRound + label.hop);
      return;
    }

    if (SHOW_DEBUG) {
      std::cout << "\t> Enqueue\n";
    }
    queue.push(currentRound + label.hop,
               TripLabel(StopEventId(label.stopEvent),
                         StopEventId(label.firstEvent +
                                     reachedIndex(label.trip,
                                                  currentRound + label.hop)),
                         parent));
    reachedIndex.update(label.trip,
                        StopIndex(label.stopEvent - label.firstEvent),
                        currentRound + label.hop);
  }

  inline void addTargetLabel(const int newArrivalTime,
                             const u_int32_t parent = -1) noexcept {
    profiler.countMetric(METRIC_ADD_JOURNEYS);

    if (SHOW_DEBUG) {
      std::cout << "Add Arr Time: " << newArrivalTime
                << ", Nr Trips: " << (targetLabels.size() - 1) << "\n";
      std::cout << "\tPrev Arr Time: " << targetLabels.back().arrivalTime
                << std::endl;
    }
    if (newArrivalTime < targetLabels.back().arrivalTime) {
      AssertMsg(newArrivalTime < minArrivalTime, "Min Arrival Time is off?");
      if (SHOW_DEBUG) {
        std::cout << "Setting a new arrival time " << newArrivalTime
                  << " with nr trips " << (targetLabels.size() - 1)
                  << std::endl;
      }
      targetLabels.back() = TargetLabel(newArrivalTime, parent);
      minArrivalTime = newArrivalTime;
    }
  }

  inline RAPTOR::Journey
  getJourney(const TargetLabel & /* targetLabel */) const noexcept {
    RAPTOR::Journey result;
    return result;
  }

  inline std::pair<StopEventId, Edge>
  getParent(const TripLabel &parentLabel,
            const StopEventId departureStopEvent) const noexcept {
    for (StopEventId i = parentLabel.begin; i < parentLabel.end; ++i) {
      for (const Edge edge : data.stopEventGraph.edgesFrom(Vertex(i))) {
        if (edgeLabels[edge].stopEvent == departureStopEvent)
          return std::make_pair(i, edge);
      }
    }
    Ensure(false, "Could not find parent stop event!");
    return std::make_pair(noStopEvent, noEdge);
  }

  inline std::pair<StopEventId, Edge>
  getParent(const TripLabel &parentLabel,
            const TargetLabel &targetLabel) const noexcept {
    // Final transfer to target may start exactly at parentLabel.end if it has
    // length 0
    const TripId trip = data.tripOfStopEvent[parentLabel.begin];
    const StopEventId end = data.firstStopEventOfTrip[trip + 1];
    for (StopEventId i = parentLabel.begin; i < end; ++i) {
      const int timeToTarget = transferToTarget[data.arrivalEvents[i].stop];
      if (timeToTarget == INFTY)
        continue;
      if (data.arrivalEvents[i].arrivalTime + timeToTarget ==
          targetLabel.arrivalTime)
        return std::make_pair(i, noEdge);
    }
    Ensure(false, "Could not find parent stop event!");
    return std::make_pair(noStopEvent, noEdge);
  }

private:
  TREXData &data;

  TransferGraph reverseTransferGraph;
  std::vector<int> transferFromSource;
  std::vector<int> transferToTarget;
  StopId lastSource;
  StopId lastTarget;

  uint16_t sourceCellId;
  uint16_t targetCellId;

  IndexedSet<false, RouteId> reachedRoutes;

  MultiQueue<TripLabel, 16> queue;

  std::vector<EdgeRange> edgeRanges;
  ProfileReachedIndexSIMD reachedIndex;

  std::vector<TargetLabel> targetLabels;
  int minArrivalTime;

  std::vector<EdgeLabel> edgeLabels;
  std::vector<RouteLabel> routeLabels;

  StopId sourceStop;
  StopId targetStop;
  int sourceDepartureTime;

  Profiler profiler;
  std::vector<uint64_t> transferPerLevel;
  size_t numQueries;
};

} // namespace TripBased
