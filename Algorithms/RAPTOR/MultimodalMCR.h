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

#include <vector>

#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Container/Map.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../DataStructures/RAPTOR/Entities/Bags.h"
#include "../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../DataStructures/RAPTOR/MultimodalData.h"
#include "../../Helpers/Vector/Vector.h"
#include "../CH/CH.h"
#include "InitialTransfers.h"
#include "Profiler.h"

namespace RAPTOR {

template <bool TARGET_PRUNING, size_t NUM_MODES, typename PROFILER = NoProfiler>
class MultimodalMCR {
 public:
  inline static constexpr bool TargetPruning = TARGET_PRUNING;
  inline static constexpr size_t NumTransferModes = NUM_MODES;
  using Profiler = PROFILER;
  using Type = MultimodalMCR<TargetPruning, NumTransferModes, Profiler>;
  using InitialTransferGraph = CHGraph;
  using SourceType = Vertex;
  using ParetoLabel = MultimodalParetoLabel<NumTransferModes>;

 public:
  struct Label {
    Label()
        : arrivalTime(never),
          transferTime{INFTY},
          parentStop(noStop),
          parentIndex(-1),
          parentDepartureTime(never),
          routeId(noRouteId) {}

    Label(const int sourceDepartureTime, const StopId sourceStop)
        : arrivalTime(sourceDepartureTime),
          transferTime{0},
          parentStop(sourceStop),
          parentIndex(-1),
          parentDepartureTime(sourceDepartureTime),
          routeId(noRouteId) {}

    Label(const Label& parentLabel, const StopId stop, const size_t parentIndex,
          const int extraTime = 0)
        : arrivalTime(parentLabel.arrivalTime + extraTime),
          parentStop(stop),
          parentIndex(parentIndex),
          parentDepartureTime(parentLabel.arrivalTime),
          transferId(noEdge) {
      std::copy(std::begin(parentLabel.transferTime),
                std::end(parentLabel.transferTime), std::begin(transferTime));
    }

    template <typename ROUTE_LABEL>
    Label(const ROUTE_LABEL& routeLabel, const StopIndex stopIndex,
          const StopId parentStop, const RouteId route)
        : arrivalTime(routeLabel.getArrivalTime(stopIndex)),
          parentStop(parentStop),
          parentIndex(routeLabel.parentIndex),
          parentDepartureTime(routeLabel.parentDepartureTime()),
          routeId(route) {
      std::copy(std::begin(routeLabel.transferTime),
                std::end(routeLabel.transferTime), std::begin(transferTime));
    }

    template <typename OTHER_LABEL>
    Label(const OTHER_LABEL& parentLabel, const int parentDepartureTime)
        : arrivalTime(parentLabel.arrivalTime),
          parentStop(parentLabel.parentStop),
          parentIndex(parentLabel.parentIndex),
          parentDepartureTime(parentDepartureTime),
          transferId(noEdge) {
      std::copy(std::begin(parentLabel.transferTime),
                std::end(parentLabel.transferTime), std::begin(transferTime));
    }

    int arrivalTime;
    int transferTime[NumTransferModes];

    StopId parentStop;
    size_t parentIndex;
    int parentDepartureTime;
    union {
      RouteId routeId;
      Edge transferId;
    };

    template <typename OTHER_LABEL>
    inline bool dominates(const OTHER_LABEL& other) const noexcept {
      if (arrivalTime > other.arrivalTime) return false;
      for (size_t i = 0; i < NumTransferModes; i++) {
        if (transferTime[i] > other.transferTime[i]) return false;
      }
      return true;
    }
  };

  struct BestLabel {
    BestLabel() : arrivalTime(never), transferTime{INFTY} {}

    BestLabel(const int arrivalTime)
        : arrivalTime(arrivalTime), transferTime{0} {}

    template <typename LABEL>
    BestLabel(const LABEL& label) : arrivalTime(label.arrivalTime) {
      std::copy(std::begin(label.transferTime), std::end(label.transferTime),
                std::begin(transferTime));
    }

    template <typename LABEL>
    inline bool dominates(const LABEL& other) const noexcept {
      if (arrivalTime > other.arrivalTime) return false;
      for (size_t i = 0; i < NumTransferModes; i++) {
        if (transferTime[i] > other.transferTime[i]) return false;
      }
      return true;
    }

    int arrivalTime;
    int transferTime[NumTransferModes];
  };

  struct RouteLabel {
    RouteLabel() {}

    RouteLabel(const StopEvent* trip, const Label& label,
               const StopIndex parentStop, const size_t parentIndex)
        : trip(trip), parentStop(parentStop), parentIndex(parentIndex) {
      std::copy(std::begin(label.transferTime), std::end(label.transferTime),
                std::begin(transferTime));
    }

    inline bool dominates(const RouteLabel& other) const noexcept {
      if (trip > other.trip) return false;
      for (size_t i = 0; i < NumTransferModes; i++) {
        if (transferTime[i] > other.transferTime[i]) return false;
      }
      return true;
    }

    inline int getArrivalTime(const StopIndex stopIndex) const noexcept {
      return trip[stopIndex].arrivalTime;
    }

    inline int parentDepartureTime() const noexcept {
      return trip[parentStop].departureTime;
    }

    const StopEvent* trip;
    int transferTime[NumTransferModes];
    StopIndex parentStop;
    size_t parentIndex;
  };

  struct DijkstraLabel {
    DijkstraLabel()
        : arrivalTime(never),
          transferTime{INFTY},
          parentStop(noStop),
          parentIndex(-1) {}

    DijkstraLabel(const int departureTime, const StopId sourceStop,
                  const size_t mode)
        : arrivalTime(departureTime + TransferModeOverhead[mode]),
          transferTime{0},
          parentStop(sourceStop),
          parentIndex(0) {
      transferTime[mode] = TransferModeOverhead[mode];
    }

    DijkstraLabel(const DijkstraLabel& parentLabel, const size_t mode,
                  const int travelTime = 0)
        : arrivalTime(parentLabel.arrivalTime + travelTime),
          parentStop(parentLabel.parentStop),
          parentIndex(parentLabel.parentIndex) {
      std::copy(std::begin(parentLabel.transferTime),
                std::end(parentLabel.transferTime), std::begin(transferTime));
      transferTime[mode] += travelTime;
    }

    DijkstraLabel(const Label& parentLabel, const StopId parentStop,
                  const size_t parentIndex, const size_t mode,
                  const int travelTime = 0)
        : arrivalTime(parentLabel.arrivalTime + TransferModeOverhead[mode] +
                      travelTime),
          parentStop(parentStop),
          parentIndex(parentIndex) {
      std::copy(std::begin(parentLabel.transferTime),
                std::end(parentLabel.transferTime), std::begin(transferTime));
      transferTime[mode] += TransferModeOverhead[mode] + travelTime;
    }

    inline int getKey() const noexcept {
      int result = arrivalTime;
      for (size_t i = 0; i < NumTransferModes; i++) {
        result += transferTime[i];
      }
      return result;
    }

    inline bool hasSmallerKey(const DijkstraLabel* const other) const noexcept {
      return getKey() < other->getKey();
    }

    template <typename OTHER_LABEL>
    inline bool dominates(const OTHER_LABEL& other) const noexcept {
      if (arrivalTime > other.arrivalTime) return false;
      for (size_t i = 0; i < NumTransferModes; i++) {
        if (transferTime[i] > other.transferTime[i]) return false;
      }
      return true;
    }

    int arrivalTime;
    int transferTime[NumTransferModes];
    StopId parentStop;
    size_t parentIndex;
  };

  using BagType = Bag<Label>;
  using BestBagType = Bag<BestLabel>;
  using Round = std::vector<BagType>;
  using RouteBagType = RouteBag<RouteLabel>;
  using DijkstraBagType = DijkstraBag<DijkstraLabel>;

 public:
  MultimodalMCR(const MultimodalData& data, const std::vector<CH::CH>& chData,
                const Profiler& profilerTemplate = Profiler())
      : data(data),
        bestBagByRoute(data.raptorData.numberOfStops() + 1),
        bestBagByTransfer(data.raptorData.numberOfStops() + 1),
        stopsUpdatedByRoute(data.raptorData.numberOfStops() + 1),
        stopsUpdatedByTransfer(data.raptorData.numberOfStops() + 1),
        routesServingUpdatedStops(data.raptorData.numberOfRoutes()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        targetStop(noStop),
        sourceDepartureTime(intMax),
        profiler(profilerTemplate) {
    AssertMsg(data.raptorData.hasImplicitBufferTimes(),
              "Departure buffer times have to be implicit!");
    AssertMsg(data.modes.size() == NumTransferModes, "Wrong number of modes");
    AssertMsg(chData.size() == NumTransferModes, "Wrong number of modes");
    for (const CH::CH& ch : chData) {
      initialTransfers.emplace_back(ch, FORWARD,
                                    data.raptorData.numberOfStops());
    }
    profiler.registerExtraRounds(
        {EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION});
    profiler.registerPhases(
        {PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
    for (const size_t mode : data.modes) {
      profiler.registerPhases({getProfilerTransferPhase(mode)});
    }
    profiler.registerMetrics({METRIC_ROUTES, METRIC_ROUTE_SEGMENTS,
                              METRIC_VERTICES, METRIC_EDGES,
                              METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
    profiler.initialize();
    for (size_t i = 0; i < NumTransferModes; i++) {
      dijkstraBags[i].resize(getTransferGraph(i).numVertices());
    }
  }

  inline void run(const Vertex source, const int departureTime,
                  const Vertex target,
                  const size_t maxRounds = INFTY) noexcept {
    profiler.start();
    profiler.startExtraRound(EXTRA_ROUND_CLEAR);
    clear();
    profiler.doneRound();

    profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
    profiler.startPhase();
    initialize(source, departureTime, target);
    profiler.donePhase(PHASE_INITIALIZATION);
    profiler.startPhase();
    relaxTransitiveTransfers();
    profiler.donePhase(PHASE_TRANSFERS);
    for (size_t mode = 0; mode < data.modes.size(); mode++) {
      profiler.startPhase();
      relaxInitialTransfers(mode);
      profiler.donePhase(getProfilerTransferPhase(data.modes[mode]));
    }
    profiler.doneRound();

    for (size_t i = 0; i < maxRounds; i++) {
      profiler.startRound();
      profiler.startPhase();
      startNewRound();
      profiler.donePhase(PHASE_INITIALIZATION);
      profiler.startPhase();
      collectRoutesServingUpdatedStops();
      profiler.donePhase(PHASE_COLLECT);
      profiler.startPhase();
      scanRoutes();
      profiler.donePhase(PHASE_SCAN);
      if (stopsUpdatedByRoute.empty()) {
        profiler.doneRound();
        break;
      }
      profiler.startPhase();
      startNewRound();
      profiler.donePhase(PHASE_INITIALIZATION);
      profiler.startPhase();
      relaxTransitiveTransfers();
      profiler.donePhase(PHASE_TRANSFERS);
      for (size_t mode = 0; mode < data.modes.size(); mode++) {
        profiler.startPhase();
        relaxIntermediateTransfers(mode);
        profiler.donePhase(getProfilerTransferPhase(data.modes[mode]));
      }
      profiler.doneRound();
    }
    profiler.done();
  }

  inline std::vector<Journey> getJourneys() const noexcept {
    return getJourneys(targetStop);
  }

  inline std::vector<Journey> getJourneys(const Vertex vertex) const noexcept {
    const StopId target =
        (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
    std::vector<Journey> journeys;
    for (size_t round = 0; round < rounds.size(); round += 2) {
      const size_t trueRound = std::min(round + 1, rounds.size() - 1);
      for (size_t i = 0; i < rounds[trueRound][target].size(); i++) {
        getJourney(journeys, trueRound, target, i);
      }
    }
    return journeys;
  }

  inline std::vector<ParetoLabel> getResults() const noexcept {
    return getResults(targetStop);
  }

  inline std::vector<ParetoLabel> getResults(const StopId stop) const noexcept {
    std::vector<ParetoLabel> result;
    for (size_t round = 0; round < rounds.size(); round += 2) {
      const size_t trueRound = std::min(round + 1, rounds.size() - 1);
      for (const Label& label : rounds[trueRound][stop].labels) {
        result.emplace_back(label, round / 2);
      }
    }
    return result;
  }

  template <bool RESET_CAPACITIES = false>
  inline void clear() noexcept {
    stopsUpdatedByRoute.clear();
    stopsUpdatedByTransfer.clear();
    routesServingUpdatedStops.clear();
    sourceVertex = noVertex;
    targetVertex = noVertex;
    targetStop = StopId(data.raptorData.numberOfStops());
    for (size_t i = 0; i < NumTransferModes; i++) {
      queue[i].clear();
    }
    if constexpr (RESET_CAPACITIES) {
      std::vector<Round>().swap(rounds);
      std::vector<BestBagType>(bestBagByRoute.size()).swap(bestBagByRoute);
      std::vector<BestBagType>(bestBagByTransfer.size())
          .swap(bestBagByTransfer);
      for (size_t i = 0; i < NumTransferModes; i++) {
        std::vector<DijkstraBagType>(dijkstraBags[i].size())
            .swap(dijkstraBags[i]);
      }
    } else {
      rounds.clear();
      Vector::fill(bestBagByRoute);
      Vector::fill(bestBagByTransfer);
      for (size_t i = 0; i < NumTransferModes; i++)
        Vector::fill(dijkstraBags[i]);
    }
  }

  inline void reset() noexcept { clear<true>(); }

  inline Profiler& getProfiler() noexcept { return profiler; }

 private:
  inline void initialize(const Vertex source, const int departureTime,
                         const Vertex target) noexcept {
    sourceVertex = source;
    targetVertex = target;
    if (data.raptorData.isStop(target)) {
      targetStop = StopId(target);
    }
    sourceDepartureTime = departureTime;

    startNewRound();
    if (data.raptorData.isStop(source)) {
      profiler.countMetric(METRIC_STOPS_BY_TRIP);
      currentRound()[sourceVertex].mergeUndominated(
          Label(sourceDepartureTime, StopId(sourceVertex)));
      bestBagByRoute[sourceVertex].mergeUndominated(
          BestLabel(sourceDepartureTime));
      stopsUpdatedByRoute.insert(StopId(source));
    }
    startNewRound();
  }

  inline void collectRoutesServingUpdatedStops() noexcept {
    for (const StopId stop : stopsUpdatedByTransfer) {
      for (const RouteSegment& route :
           data.raptorData.routesContainingStop(stop)) {
        AssertMsg(data.raptorData.isRoute(route.routeId),
                  "Route " << route.routeId << " is out of range!");
        AssertMsg(
            data.raptorData
                    .stopIds[data.raptorData.firstStopIdOfRoute[route.routeId] +
                             route.stopIndex] == stop,
            "RAPTOR data contains invalid route segments!");
        if (route.stopIndex + 1 ==
            data.raptorData.numberOfStopsInRoute(route.routeId))
          continue;
        if (routesServingUpdatedStops.contains(route.routeId)) {
          routesServingUpdatedStops[route.routeId] = std::min(
              routesServingUpdatedStops[route.routeId], route.stopIndex);
        } else {
          routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
        }
      }
    }
  }

  inline void scanRoutes() noexcept {
    stopsUpdatedByRoute.clear();
    for (const RouteId route : routesServingUpdatedStops.getKeys()) {
      profiler.countMetric(METRIC_ROUTES);
      StopIndex stopIndex = routesServingUpdatedStops[route];
      const size_t tripSize = data.raptorData.numberOfStopsInRoute(route);
      AssertMsg(stopIndex < tripSize - 1,
                "Cannot scan a route starting at/after the last stop (Route: "
                    << route << ", StopIndex: " << stopIndex
                    << ", TripSize: " << tripSize << ")!");

      const StopId* stops = data.raptorData.stopArrayOfRoute(route);
      StopId stop = stops[stopIndex];

      const StopEvent* firstTrip = data.raptorData.firstTripOfRoute(route);
      const StopEvent* lastTrip = data.raptorData.lastTripOfRoute(route);

      RouteBagType routeBag;

      while (stopIndex < tripSize - 1) {
        for (size_t i = 0; i < previousRound()[stop].size(); i++) {
          const Label& label = previousRound()[stop][i];
          const StopEvent* trip = firstTrip;
          while ((trip < lastTrip) &&
                 (trip[stopIndex].departureTime < label.arrivalTime)) {
            trip += tripSize;
          }
          if (trip[stopIndex].departureTime < label.arrivalTime) continue;
          routeBag.merge(RouteLabel(trip, label, stopIndex, i));
        }
        stopIndex++;
        stop = stops[stopIndex];
        profiler.countMetric(METRIC_ROUTE_SEGMENTS);
        for (const RouteLabel& label : routeBag.labels) {
          arrivalByRoute(
              stop, Label(label, stopIndex, stops[label.parentStop], route));
        }
      }
    }
  }

  inline void relaxInitialTransfers(const size_t mode) noexcept {
    const DijkstraLabel sourceLabel(sourceDepartureTime, StopId(sourceVertex),
                                    mode);
    dijkstraBags[mode][sourceVertex].initialize(sourceLabel);
    initialTransfers[mode].template run<true>(sourceVertex, targetVertex);
    for (const Vertex stop : initialTransfers[mode].getForwardPOIs()) {
      if (stop == targetStop || stop == sourceVertex) continue;
      AssertMsg(data.raptorData.isStop(stop),
                "Reached POI " << stop << " is not a stop!");
      AssertMsg(initialTransfers[mode].getForwardDistance(stop) != INFTY,
                "Vertex " << stop << " was not reached!");
      const DijkstraLabel dijkstraLabel(
          sourceLabel, mode, initialTransfers[mode].getForwardDistance(stop));
      dijkstraBags[mode][stop].initialize(dijkstraLabel);
      arrivalByTransfer(StopId(stop), dijkstraLabel);
    }
    if (initialTransfers[mode].getDistance() != INFTY) {
      const DijkstraLabel targetLabel(sourceLabel, mode,
                                      initialTransfers[mode].getDistance());
      dijkstraBags[mode][targetVertex].initialize(targetLabel);
      arrivalByTransfer(targetStop, targetLabel);
    }
  }

  inline void relaxTransitiveTransfers() noexcept {
    stopsUpdatedByTransfer.clear();
    routesServingUpdatedStops.clear();
    for (const StopId stop : stopsUpdatedByRoute) {
      stopsUpdatedByTransfer.insert(stop);
      const BagType& bag = previousRound()[stop];
      currentRound()[stop].resize(bag.size());
      for (size_t i = 0; i < bag.size(); i++) {
        currentRound()[stop][i] = Label(bag[i], stop, i);
      }
    }
    for (const StopId stop : stopsUpdatedByRoute) {
      const BagType& bag = previousRound()[stop];
      for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stop)) {
        profiler.countMetric(METRIC_EDGES);
        const Vertex toStop = data.raptorData.transferGraph.get(ToVertex, edge);
        AssertMsg(data.raptorData.isStop(toStop),
                  "Graph contains edges to non-stop vertices!");
        const int travelTime =
            data.raptorData.transferGraph.get(TravelTime, edge);
        for (size_t i = 0; i < bag.size(); i++) {
          const Label newLabel(bag[i], stop, i, travelTime);
          arrivalByTransfer(StopId(toStop), newLabel);
        }
      }
    }
  }

  inline void relaxIntermediateTransfers(const size_t mode) noexcept {
    AssertMsg(queue[mode].empty(),
              "Queue still has " << queue[mode].size() << " elements!");
    for (const StopId stop : stopsUpdatedByRoute) {
      if (initialTransfers[mode].getBackwardDistance(stop) == INFTY) continue;
      const BagType& bag = previousRound()[stop];
      for (size_t i = 0; i < bag.size(); i++) {
        const DijkstraLabel targetLabel(
            bag[i], stop, i, mode,
            initialTransfers[mode].getBackwardDistance(stop));
        if (dijkstraBags[mode][targetVertex].template merge<false>(
                targetLabel)) {
          arrivalByTransfer(targetStop, targetLabel);
        }
      }
    }
    for (const StopId stop : stopsUpdatedByRoute) {
      const BagType& bag = previousRound()[stop];
      for (size_t i = 0; i < bag.size(); i++) {
        arrivalByEdge(mode, stop, DijkstraLabel(bag[i], stop, i, mode));
      }
    }
    dijkstra(mode);
  }

  inline void dijkstra(const size_t mode) noexcept {
    while (!queue[mode].empty()) {
      DijkstraBagType* uBag = queue[mode].extractFront();
      const DijkstraLabel& uLabel = uBag->extractFront();
      if (!uBag->heapEmpty()) queue[mode].update(uBag);
      const Vertex u = Vertex(uBag - &(dijkstraBags[mode][0]));
      for (const Edge edge : getTransferGraph(mode).edgesFrom(u)) {
        const Vertex v = getTransferGraph(mode).get(ToVertex, edge);
        if (v == targetVertex || v == uLabel.parentStop) continue;
        profiler.countMetric(METRIC_EDGES);
        const DijkstraLabel vLabel(
            uLabel, mode, getTransferGraph(mode).get(TravelTime, edge));
        arrivalByEdge(mode, v, vLabel);
      }
      if (data.raptorData.isStop(u)) {
        arrivalByTransfer(StopId(u), uLabel);
      }
      profiler.countMetric(METRIC_VERTICES);
    }
  }

  inline const TransferGraph& getTransferGraph(
      const size_t mode) const noexcept {
    return data.getTransferGraph(data.modes[mode]);
  }

  inline Round& currentRound() noexcept {
    AssertMsg(!rounds.empty(),
              "Cannot return current round, because no round exists!");
    return rounds.back();
  }

  inline Round& previousRound() noexcept {
    AssertMsg(
        rounds.size() >= 2,
        "Cannot return previous round, because less than two rounds exist!");
    return rounds[rounds.size() - 2];
  }

  inline void startNewRound() noexcept {
    rounds.emplace_back(data.raptorData.numberOfStops() + 1);
  }

  template <typename LABEL>
  inline bool checkTargetPruning(const LABEL& label) noexcept {
    if (bestBagByRoute[targetStop].dominates(label)) return true;
    if (bestBagByTransfer[targetStop].dominates(label)) return true;
    return false;
  }

  inline void arrivalByRoute(const StopId stop, const Label& label) noexcept {
    AssertMsg(data.raptorData.isStop(stop),
              "Stop " << stop << " is out of range!");
    if constexpr (TargetPruning)
      if (checkTargetPruning(label)) return;
    if (!bestBagByRoute[stop].merge(BestLabel(label))) return;
    currentRound()[stop].mergeUndominated(label);
    profiler.countMetric(METRIC_STOPS_BY_TRIP);
    stopsUpdatedByRoute.insert(stop);
  }

  inline bool arrivalByEdge(const size_t mode, const Vertex vertex,
                            const DijkstraLabel& label) noexcept {
    AssertMsg(label.arrivalTime >= sourceDepartureTime,
              "Arriving by route BEFORE departing from the source (source "
              "departure time: "
                  << String::secToTime(sourceDepartureTime) << " ["
                  << sourceDepartureTime
                  << "], arrival time: " << String::secToTime(label.arrivalTime)
                  << " [" << label.arrivalTime << "])!");
    if constexpr (TargetPruning)
      if (checkTargetPruning(label)) return false;
    if (!dijkstraBags[mode][vertex].template merge<true>(label)) return false;
    queue[mode].update(&dijkstraBags[mode][vertex]);
    return true;
  }

  template <typename LABEL>
  inline void arrivalByTransfer(const StopId stop,
                                const LABEL& label) noexcept {
    AssertMsg(data.raptorData.isStop(stop) || stop == targetStop,
              "Stop " << stop << " is out of range!");
    AssertMsg(label.arrivalTime >= sourceDepartureTime,
              "Arriving by route BEFORE departing from the source (source "
              "departure time: "
                  << String::secToTime(sourceDepartureTime) << " ["
                  << sourceDepartureTime
                  << "], arrival time: " << String::secToTime(label.arrivalTime)
                  << " [" << label.arrivalTime << "])!");
    if (bestBagByRoute[stop].dominates(label)) return;
    if (!bestBagByTransfer[stop].merge(BestLabel(label))) return;
    profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
    const int parentDepartureTime =
        (label.parentStop == sourceVertex)
            ? sourceDepartureTime
            : previousRound()[label.parentStop][label.parentIndex].arrivalTime;
    currentRound()[stop].mergeUndominated(Label(label, parentDepartureTime));
    if (data.raptorData.isStop(stop)) stopsUpdatedByTransfer.insert(stop);
  }

  inline void getJourney(std::vector<Journey>& journeys, size_t round,
                         StopId stop, size_t index) const noexcept {
    Journey journey;
    do {
      AssertMsg(round != size_t(-1),
                "Backtracking parent pointers did "
                "not pass through the source stop!");
      const Label& label = rounds[round][stop][index];
      journey.emplace_back(label.parentStop, stop, label.parentDepartureTime,
                           label.arrivalTime, round % 2 == 0, label.routeId);
      stop = label.parentStop;
      index = label.parentIndex;
      round--;
    } while (journey.back().from != sourceVertex);
    journeys.emplace_back(Vector::reverse(journey));
  }

 private:
  const MultimodalData& data;

  std::vector<CoreCHInitialTransfers> initialTransfers;

  std::vector<Round> rounds;
  std::vector<BestBagType> bestBagByRoute;
  std::vector<BestBagType> bestBagByTransfer;

  IndexedSet<false, StopId> stopsUpdatedByRoute;
  IndexedSet<false, StopId> stopsUpdatedByTransfer;
  IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

  Vertex sourceVertex;
  Vertex targetVertex;
  StopId targetStop;
  int sourceDepartureTime;

  std::vector<DijkstraBagType> dijkstraBags[NumTransferModes];
  ExternalKHeap<2, DijkstraBagType> queue[NumTransferModes];

  Profiler profiler;
};

}  // namespace RAPTOR
