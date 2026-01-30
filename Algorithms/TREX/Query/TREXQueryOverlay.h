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

#include <array>
#include <vector>

#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/Graph/SimpleGraph.h"
#include "../../../DataStructures/Graph/Utils/Conversion.h"
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../../DataStructures/TREX/TREXData.h"
#include "../../TripBased/Query/Profiler.h"
#include "../../TripBased/Query/ReachedIndex.h"

namespace TripBased {

template <typename PROFILER = NoProfiler> class TREXQuery {
public:
  using Profiler = PROFILER;
  using Type = TREXQuery<Profiler>;

private:
  struct TripLabel {
    TripLabel(const StopEventId begin = noStopEvent,
              const StopEventId end = noStopEvent, const uint32_t parent = -1,
              const uint16_t lcl = 0)
        : begin(begin), end(end), parent(parent), lcl(lcl) {}
    StopEventId begin;
    StopEventId end;
    uint32_t parent;
    uint16_t lcl;
  };

  struct EdgeRange {
    EdgeRange() : begin(noEdge), end(noEdge) {}
    Edge begin;
    Edge end;
  };

  struct EdgeLabel {
    EdgeLabel(const StopEventId firstEvent = noStopEvent,
              const TripId trip = noTripId,
              const StopIndex stopEvent = noStopIndex)
        : firstEvent(firstEvent), trip(trip), stopEvent(stopEvent) {}
    StopEventId firstEvent;
    TripId trip;
    StopIndex stopEvent;
  };

  struct RouteLabel {
    RouteLabel() : numberOfTrips(0) {}
    inline StopIndex end() const noexcept {
      return StopIndex(departureTimes.size() / numberOfTrips);
    }
    uint32_t numberOfTrips;
    std::vector<int> departureTimes;
  };

  struct TargetLabel {
    TargetLabel(const int arrivalTime = INFTY, const uint32_t parent = -1)
        : arrivalTime(arrivalTime), parent(parent) {}

    int arrivalTime;
    uint32_t parent;
  };

public:
  TREXQuery(TREXData &data)
      : data(data), reverseTransferGraph(data.raptorData.transferGraph),
        transferFromSource(data.numberOfStops(), INFTY),
        transferToTarget(data.numberOfStops(), INFTY), lastSource(StopId(0)),
        lastTarget(StopId(0)), reachedRoutes(data.numberOfRoutes()),
        queue(data.numberOfStopEvents()), queueSize(0), reachedIndex(data),
        targetLabels(1), minArrivalTime(INFTY),
        edgeLabels(data.numberOfLevels + 1, std::vector<EdgeLabel>()),
        routeLabels(data.numberOfRoutes()),
        cellIdOfEvent(data.numberOfStopEvents(), 0), sourceStop(noStop),
        targetStop(noStop), sourceDepartureTime(never),
        transferPerLevel(data.getNumberOfLevels() + 1, 0), numQueries(0),
        overlayGraphs(),
        edgeRangeLookup(data.numberOfLevels + 1,
                        std::vector<StopEventId>(data.numberOfStopEvents())) {
    reverseTransferGraph.revert();

    // fill the overlayGraphs _per level_
    int numOverlayGraphs = data.numberOfLevels + 1;
    overlayGraphs.reserve(numOverlayGraphs);

    for (int i = 0; i < numOverlayGraphs; ++i) {
      overlayGraphs.emplace_back();
    }

    AssertMsg(overlayGraphs.size() ==
                  static_cast<std::size_t>(numOverlayGraphs),
              "The number of overlay graphs is off!");

    std::vector<std::tuple<std::uint32_t, std::uint32_t, uint16_t>>
        edgesToInsert;
    edgesToInsert.reserve(data.stopEventGraph.numEdges());

    for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
      edgesToInsert.emplace_back(
          (std::uint32_t)from,
          (std::uint32_t)data.stopEventGraph.get(ToVertex, edge),
          data.stopEventGraph.get(LocalLevel, edge));
    }

    for (int i = 0; i < numOverlayGraphs; ++i) {
      std::sort(edgesToInsert.begin(), edgesToInsert.end());
      overlayGraphs[i].fromEdgeList(edgesToInsert,
                                    data.stopEventGraph.numVertices());
      edgeLabels[i].resize(edgesToInsert.size());

      for (std::size_t edge = 0; edge < edgesToInsert.size(); ++edge) {
        auto [from, to, rank] = edgesToInsert[edge];
        AssertMsg(to < data.numberOfStopEvents(), "To StopEventId is invalid!");
        AssertMsg(from < data.numberOfStopEvents(),
                  "From StopEventId is invalid!");
        AssertMsg(rank < 16, "Rank is invalid!");

        edgeLabels[i][edge].trip = data.tripOfStopEvent[to];
        edgeLabels[i][edge].firstEvent =
            data.firstStopEventOfTrip[edgeLabels[i][edge].trip];
        edgeLabels[i][edge].stopEvent =
            StopIndex(to - edgeLabels[i][edge].firstEvent + 1);
      }

      edgesToInsert.erase(std::remove_if(edgesToInsert.begin(),
                                         edgesToInsert.end(),
                                         [i](const auto &e) {
                                           return std::get<2>(e) <= (uint16_t)i;
                                         }),
                          edgesToInsert.end());
    }

    AssertMsg(edgesToInsert.size() == 0, "The Edge Graph still has edges?");

    for (int i = 0; i < numOverlayGraphs; ++i) {
      std::cout << "Overlay Graph " << i << ": " << overlayGraphs[i].numEdges()
                << "\n";
      /* overlayGraphs[i].showStats(); */
    }

    auto inSameCell = [&](const StopId a, const StopId b,
                          const int level) -> bool {
      assert(level >= 0 && level < 16);
      return (data.getCellIdOfStop(a) >> level) ==
             (data.getCellIdOfStop(b) >> level);
    };

    // fill the edge range lookup
    for (const RouteId route : data.routes()) {
      const StopId *stopsOfRoute = data.raptorData.stopArrayOfRoute(route);
      const std::size_t nrStopsInRoute = data.numberOfStopsInRoute(route);

      for (int level = 0; level < data.numberOfLevels + 1; ++level) {
        std::vector<std::uint8_t> lengths(nrStopsInRoute, 1);

        if (nrStopsInRoute > 1) {
          std::size_t segmentEnd = nrStopsInRoute;

          for (std::size_t i = nrStopsInRoute - 1; i-- > 0;) {
            if (!inSameCell(stopsOfRoute[i], stopsOfRoute[i + 1], level)) {
              segmentEnd = i + 1;
            }

            const std::size_t len = segmentEnd - i;

            AssertMsg(len > 0, "Length must be >= 1");
            AssertMsg(len <= nrStopsInRoute - i, "Length exceeds trip suffix");
            AssertMsg(len < 256, "The length in a subtrip is larger than 255!");

            lengths[i] = static_cast<std::uint8_t>(len);
          }
        }

        AssertMsg(static_cast<std::size_t>(level) < edgeRangeLookup.size(),
                  "Level " << level
                           << " is not a valid index into edgeRangeLookup!");

        for (std::size_t tripOffset = 0;
             tripOffset < data.raptorData.numberOfTripsInRoute(route);
             ++tripOffset) {
          const std::size_t startIndex =
              data.raptorData.firstStopEventOfRoute[route] +
              tripOffset * nrStopsInRoute;
          AssertMsg(startIndex < edgeRangeLookup[level].size(),
                    "Start Index out of bounds!");

          for (std::size_t i = 0; i < nrStopsInRoute; ++i) {
            const StopEventId eventId(startIndex + i);
            AssertMsg(eventId < edgeRangeLookup[level].size(),
                      "EventId is out of range!");
            edgeRangeLookup[level][eventId] = StopEventId(eventId + lengths[i]);
          }
        }
      }
    }

    for (size_t event = 0; event < data.numberOfStopEvents(); ++event) {
      const StopId stop = data.getStopOfStopEvent(StopEventId(event));
      AssertMsg(data.raptorData.isStop(Vertex(stop)), "Stop is not a stop!");
      cellIdOfEvent[event] = (uint16_t)data.getCellIdOfStop(stop);
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
      result.emplace_back(RAPTOR::Journey());
      /* result.emplace_back(getJourney(label)); */
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

    /* std::cout << "# of irrelvant events: " << irrelevantEvents /
     * (double)numQueries << std::endl; */
  }

private:
  inline void clear() noexcept {
    queueSize = 0;
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
    if (sourceStop == targetStop)
      addTargetLabel(sourceDepartureTime);
    for (const Edge edge : reverseTransferGraph.edgesFrom(targetStop)) {
      const Vertex stop = reverseTransferGraph.get(ToVertex, edge);
      if (stop == sourceStop)
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
        const uint32_t labelIndex = stopIndex * label.numberOfTrips;
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
    u_int8_t currentRoundNumber = 0;
    size_t roundBegin = 0;
    size_t roundEnd = queueSize;
    while (roundBegin < roundEnd && currentRoundNumber < MAX_ROUNDS) {
      ++currentRoundNumber;

      profiler.countMetric(METRIC_ROUNDS);
      targetLabels.emplace_back(targetLabels.back());
      for (size_t i = roundBegin; i < roundEnd; ++i) {
#ifdef ENABLE_PREFETCH
        if (i + 4 < roundEnd) {
          __builtin_prefetch(&data.arrivalEvents[queue[i + 4].begin]);
        }
#endif

        const TripLabel &label = queue[i];

        profiler.countMetric(METRIC_SCANNED_TRIPS);
        for (StopEventId j = label.begin; j < label.end; j++) {
          profiler.countMetric(METRIC_SCANNED_STOPS);
          if (data.arrivalEvents[j].arrivalTime >= minArrivalTime)
            break;
          const int timeToTarget = transferToTarget[data.arrivalEvents[j].stop];
          if (timeToTarget != INFTY) {
            addTargetLabel(data.arrivalEvents[j].arrivalTime + timeToTarget, i);
          }
        }
      }

      for (size_t i = roundBegin; i < roundEnd; i++) {
#ifdef ENABLE_PREFETCH
        if (i + 4 < roundEnd) {
          __builtin_prefetch(&data.arrivalEvents[queue[i + 4].begin]);
        }
#endif

        TripLabel &label = queue[i];
        for (StopEventId j = label.begin; j < label.end; j++) {
          if (data.arrivalEvents[j].arrivalTime >= minArrivalTime) {
            label.end = j;
            break;
          }
        }
      }

      /*
     for (size_t i = roundBegin; i < roundEnd; i++) {
#ifdef ENABLE_PREFETCH
       if (i + 4 < roundEnd) {
         __builtin_prefetch(&cellIdOfEvent[queue[i + 4].begin]);
       }
#endif

       const TripLabel &label = queue[i];

       for (StopEventId j = label.begin; j < label.end; j++) {
         uint16_t lcl = static_cast<std::uint16_t>(std::min(
             std::bit_width<uint16_t>(cellIdOfEvent[j] ^ sourceCellId),
             std::bit_width<uint16_t>(cellIdOfEvent[j] ^ targetCellId)));
         AssertMsg(0 <= lcl && lcl < 16,
                   "LCL (" << (int)lcl << ") is invalid!\n");
         AssertMsg(static_cast<std::size_t>(lcl) < overlayGraphs.size(),
                   "LCL value ("
                       << lcl
                       << ") cannot be used as index into overlayGraphs!");
         const auto &curGraph = overlayGraphs[lcl];

         const std::size_t beginEdgeRange = curGraph.beginEdge(Vertex(j));
         const std::size_t endEdgeRange = curGraph.endEdge(Vertex(j));
         for (std::size_t edge = beginEdgeRange; edge < endEdgeRange; ++edge) {
           profiler.countMetric(METRIC_RELAXED_TRANSFERS);
           enqueue(edge, i, lcl);
         }
       }
     }
     */

      for (size_t i = roundBegin; i < roundEnd; i++) {
#ifdef ENABLE_PREFETCH
        if (i + 4 < roundEnd) {
          __builtin_prefetch(&cellIdOfEvent[queue[i + 4].begin]);
        }
#endif

        const TripLabel &label = queue[i];

        for (StopEventId j = label.begin; j < label.end;) {
          int lcl = static_cast<int>(std::min(
              std::bit_width<uint16_t>(cellIdOfEvent[j] ^ sourceCellId),
              std::bit_width<uint16_t>(cellIdOfEvent[j] ^ targetCellId)));

          AssertMsg(static_cast<std::size_t>(lcl) < overlayGraphs.size(),
                    "LCL value ("
                        << lcl
                        << ") cannot be used as index into overlayGraphs!");
          int lowerLcl = std::max(lcl - 1, 0);
          const StopEventId nextStopEventOutside = edgeRangeLookup[lowerLcl][j];

          const StopEventId endOfConsecutiveLCL =
              std::min(nextStopEventOutside, label.end);

          const auto &curGraph = overlayGraphs[lcl];

          const std::size_t beginEdgeRange = curGraph.beginEdge(Vertex(j));
          const std::size_t endEdgeRange =
              curGraph.beginEdge(Vertex(endOfConsecutiveLCL));

          for (std::size_t edge = beginEdgeRange; edge < endEdgeRange; ++edge) {
            profiler.countMetric(METRIC_RELAXED_TRANSFERS);
            enqueue(edge, i, lcl);
          }

          j = endOfConsecutiveLCL;
        }
      }
      roundBegin = roundEnd;
      roundEnd = queueSize;
    }
    profiler.donePhase(PHASE_SCAN_TRIPS);
  }

  inline void enqueue(const TripId trip, const StopIndex index) noexcept {
    profiler.countMetric(METRIC_ENQUEUES);
    if (reachedIndex.alreadyReached(trip, index))
      return;
    const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
    queue[queueSize] = TripLabel(StopEventId(firstEvent + index),
                                 StopEventId(firstEvent + reachedIndex(trip)));
    ++queueSize;
    AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
    reachedIndex.update(trip, index);
  }

  inline void enqueue(const std::size_t edge, const size_t parent,
                      const int lcl) noexcept {
    profiler.countMetric(METRIC_ENQUEUES);
    const EdgeLabel &label = edgeLabels[lcl][edge];

    if (reachedIndex.alreadyReached(label.trip, label.stopEvent)) [[likely]]
      return;

    queue[queueSize] = TripLabel(
        StopEventId(label.stopEvent + label.firstEvent),
        StopEventId(label.firstEvent + reachedIndex(label.trip)), parent, lcl);
    ++queueSize;
    AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
    reachedIndex.update(label.trip, StopIndex(label.stopEvent));
  }

  inline void addTargetLabel(const int newArrivalTime,
                             const uint32_t parent = -1) noexcept {
    profiler.countMetric(METRIC_ADD_JOURNEYS);
    if (newArrivalTime < targetLabels.back().arrivalTime) {
      targetLabels.back() = TargetLabel(newArrivalTime, parent);
      minArrivalTime = newArrivalTime;
    }
  }

  inline RAPTOR::Journey
  getJourney(const TargetLabel &targetLabel) const noexcept {
    RAPTOR::Journey result;
    uint32_t parent = targetLabel.parent;
    if (parent == uint32_t(-1)) {
      result.emplace_back(sourceStop, targetStop, sourceDepartureTime,
                          targetLabel.arrivalTime, false);
      return result;
    }
    StopEventId departureStopEvent = noStopEvent;
    Vertex departureStop = targetStop;
    int lastTime(sourceDepartureTime);
    while (parent != uint32_t(-1)) {
      AssertMsg(parent < queueSize, "Parent " << parent << " is out of range!");
      const TripLabel &label = queue[parent];
      StopEventId arrivalStopEvent;
      Edge edge;
      std::tie(arrivalStopEvent, edge) =
          (departureStopEvent == noStopEvent)
              ? getParent(label, targetLabel)
              : getParent(label, StopEventId(departureStopEvent + 1));

      const StopId arrivalStop = data.getStopOfStopEvent(arrivalStopEvent);
      const int arrivalTime =
          data.raptorData.stopEvents[arrivalStopEvent].arrivalTime;
      const int transferArrivalTime =
          (edge == noEdge)
              ? targetLabel.arrivalTime
              : arrivalTime + data.stopEventGraph.get(TravelTime, edge);
      result.emplace_back(arrivalStop, departureStop, arrivalTime,
                          transferArrivalTime, edge);

      departureStopEvent = StopEventId(label.begin - 1);
      departureStop = data.getStopOfStopEvent(departureStopEvent);
      const RouteId route = data.getRouteOfStopEvent(departureStopEvent);
      const int departureTime =
          data.raptorData.stopEvents[departureStopEvent].departureTime;
      lastTime = departureTime;
      result.emplace_back(departureStop, arrivalStop, departureTime,
                          arrivalTime, true, route);

      parent = label.parent;
    }
    const int timeFromSource = transferFromSource[departureStop];
    result.emplace_back(sourceStop, departureStop,
                        sourceDepartureTime + timeFromSource, lastTime, noEdge);
    Vector::reverse(result);
    return result;
  }

  inline std::pair<StopEventId, Edge>
  getParent(const TripLabel &parentLabel,
            const StopEventId departureStopEvent) const noexcept {
    for (StopEventId i = parentLabel.begin; i < parentLabel.end; ++i) {
      for (const Edge edge : data.stopEventGraph.edgesFrom(Vertex(i))) {
        if (edgeLabels[edge].stopEvent + edgeLabels[edge].firstEvent ==
            departureStopEvent)
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

  std::vector<TripLabel> queue;
  size_t queueSize;
  ReachedIndex reachedIndex;

  std::vector<TargetLabel> targetLabels;
  int minArrivalTime;

  std::vector<std::vector<EdgeLabel>> edgeLabels;
  std::vector<RouteLabel> routeLabels;
  std::vector<uint16_t> cellIdOfEvent;

  StopId sourceStop;
  StopId targetStop;
  int sourceDepartureTime;

  Profiler profiler;
  std::vector<uint64_t> transferPerLevel;
  size_t numQueries;

  std::vector<SimpleGraph<std::uint32_t>> overlayGraphs;

  std::vector<std::vector<StopEventId>> edgeRangeLookup;
  /* std::vector<std::vector<std::vector<std::uint8_t>>> edgeRangeLookup; */
};

} // namespace TripBased
