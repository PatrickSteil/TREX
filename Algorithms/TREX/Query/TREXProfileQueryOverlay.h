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

#include "../../../DataStructures/Container/Queue.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/Graph/SimpleGraph.h"
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../../DataStructures/RAPTOR/Entities/RouteSegment.h"
#include "../../../DataStructures/TREX/TREXData.h"
#include "../../../Helpers/String/String.h"

#ifdef USE_SIMD
#include "../../TripBased/Query/ProfileReachedIndexSIMD.h"
#else
#include "../../TripBased/Query/ProfileReachedIndex.h"
#endif

#include "../../TripBased/Query/Profiler.h"
#include "../../TripBased/Query/Types.h"

#if defined(__GNUC__) || defined(__clang__)
#define RESTRICT __restrict
#elif defined(_MSC_VER)
#define RESTRICT __restrict
#else
#define RESTRICT
#endif

namespace TripBased {

template <typename PROFILER = NoProfiler> class TREXProfileQueryOverlay {
public:
  using Profiler = PROFILER;
  using Type = TREXProfileQueryOverlay<Profiler>;

private:
  struct TripStopIndex {
    TripStopIndex(const TripId trip = noTripId,
                  const StopIndex stopIndex = StopIndex(-1),
                  const int depTime = never)
        : trip(trip), stopIndex(stopIndex), depTime(depTime) {}

    TripId trip;
    StopIndex stopIndex;
    int depTime;
  };

  // compact information for an event
  struct EventLookup {
    StopId stop;
    uint32_t arrTime;

    EventLookup(const StopId stop = noStop, uint32_t arrTime = 0)
        : stop(stop), arrTime(arrTime) {}
  };

  struct QueueElementTargetCell {
    QueueElementTargetCell(const StopEventId begin = noStopEvent,
                           const StopEventId end = noStopEvent,
                           const uint32_t originalId = 0)
        : begin(begin), end(end), originalId(originalId) {}
    StopEventId begin;
    StopEventId end;
    uint32_t originalId;
  };

  struct TripLabel {
  private:
    // 8 bytes for hot scan data
    uint32_t _begin;
    uint32_t _end;

    // 4 bytes for metadata (Total: 12 bytes)
    // We pack boardingEvent, parent, and lcl into 64 bits internally
    // to manage the 27-bit requirements.
    struct Packed {
      uint64_t boarding : 27;
      uint64_t parent : 27;
      uint64_t lcl : 10; // Extra room for levels or flags
    } _meta;

  public:
    TripLabel(const StopEventId begin = noStopEvent,
              const StopEventId end = noStopEvent, const uint32_t parent = -1,
              const StopEventId boardingEvent = noStopEvent,
              const uint8_t lcl = 0)
        : _begin(begin), _end(end) {
      setParent(parent);
      setBoardingEvent(boardingEvent);
      setLcl(lcl);
    }

    inline StopEventId begin() const noexcept { return StopEventId(_begin); }
    inline StopEventId end() const noexcept { return StopEventId(_end); }

    inline uint32_t parent() const noexcept {
      return (_meta.parent == 0x7FFFFFF) ? uint32_t(-1)
                                         : uint32_t(_meta.parent);
    }

    inline uint8_t lcl() const noexcept {
      return static_cast<uint8_t>(_meta.lcl);
    }

    inline StopEventId boardingEvent() const noexcept {
      return StopEventId(_meta.boarding);
    }

    inline void setBegin(const StopEventId val) noexcept { _begin = val; }
    inline void setEnd(const StopEventId val) noexcept { _end = val; }

    inline void setParent(const uint32_t val) noexcept {
      if (val == uint32_t(-1)) {
        _meta.parent = 0x7FFFFFF;
      } else {
        assert(val < (1 << 27));
        _meta.parent = val;
      }
    }

    inline void setLcl(const uint8_t val) noexcept { _meta.lcl = val; }

    inline void setBoardingEvent(const StopEventId val) noexcept {
      assert(val < (1 << 27) || val == noStopEvent);
      _meta.boarding = val;
    }
  };

  struct EdgeRange {
    EdgeRange() : begin(noEdge), end(noEdge) {}
    Edge begin;
    Edge end;
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

    void clear() {
      arrivalTime = INFTY;
      parent = -1;
    }

    int arrivalTime;
    uint32_t parent;
  };

public:
  TREXProfileQueryOverlay(const TREXData &data)
      : data(data), reverseTransferGraph(data.raptorData.transferGraph),
        transferFromSource(data.numberOfStops(), INFTY),
        transferToTarget(data.numberOfStops(), INFTY), lastSource(StopId(0)),
        lastTarget(StopId(0)), reachedRoutes(data.numberOfRoutes()),
        queue(data.numberOfStopEvents()), tmpQueue(data.numberOfStopEvents()),
        targetCellQueue(data.numberOfStopEvents()),
        edgeRanges(data.numberOfStopEvents()), reachedIndex(data),
        targetLabels(1), minArrivalTimeFastLookUp(16, INFTY),
        edgeLabels(data.numberOfLevels + 1, std::vector<EdgeLabel>()),
        eventLookup(data.numberOfStopEvents()),
        eventArrTimes(data.numberOfStopEvents()),
        cellIdOfEvent(data.numberOfStopEvents(), 0), sourceStop(noStop),
        targetStop(noStop), minDepartureTime(never), maxDepartureTime(never),
        targetLabelChanged(16, false),
        routeLabels(data.stopEventGraph.numEdges()), overlayGraphs(),
        edgeRangeLookup(data.numberOfStopEvents()) {
    for (auto &a : edgeRangeLookup) {
      a.fill(noStopEvent);
    }
    collectedDepTimes.reserve(
        data.raptorData.numberOfTrips()); // can be adjusted
    allJourneys.reserve(32);
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

        edgeLabels[i][edge].setTrip(data.tripOfStopEvent[to]);
        edgeLabels[i][edge].setFirstEvent(
            data.firstStopEventOfTrip[edgeLabels[i][edge].getTrip()]);
        edgeLabels[i][edge].setStopIndex(
            StopIndex(to - edgeLabels[i][edge].getFirstEvent() + 1));
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

        for (std::size_t tripOffset = 0;
             tripOffset < data.raptorData.numberOfTripsInRoute(route);
             ++tripOffset) {
          const std::size_t startIndex =
              data.raptorData.firstStopEventOfRoute[route] +
              tripOffset * nrStopsInRoute;
          AssertMsg(startIndex < edgeRangeLookup.size(),
                    "Start Index out of bounds!");

          for (std::size_t i = 0; i < nrStopsInRoute; ++i) {
            const StopEventId eventId(startIndex + i);
            AssertMsg(eventId < edgeRangeLookup.size(),
                      "EventId is out of range!");
            edgeRangeLookup[eventId][level] = StopEventId(eventId + lengths[i]);
          }
        }
      }
    }

#pragma omp parallel for
    for (size_t event = 0; event < data.numberOfStopEvents(); ++event) {
      const StopId stop = data.getStopOfStopEvent(StopEventId(event));
      AssertMsg(data.raptorData.isStop(stop), "Stop is not a stop!");
      cellIdOfEvent[event] = (uint16_t)data.getCellIdOfStop(stop);
    }

#pragma omp parallel for
    for (size_t event = 0; event < data.numberOfStopEvents(); ++event) {
      eventLookup[event] = EventLookup(data.arrivalEvents[event].stop,
                                       data.arrivalEvents[event].arrivalTime);
      eventArrTimes[event] = data.arrivalEvents[event].arrivalTime;
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
        {PHASE_SCAN_INITIAL, PHASE_COLLECT_DEPTIMES, PHASE_SCAN_TRIPS});
    profiler.registerMetrics({METRIC_ROUNDS, METRIC_SCANNED_TRIPS,
                              METRIC_SCANNED_STOPS, METRIC_RELAXED_TRANSFERS,
                              METRIC_ENQUEUES, METRIC_ADD_JOURNEYS});
  }

  inline void run(const Vertex source, const Vertex target,
                  const int minDepartureTime,
                  const int maxDepartureTime) noexcept {
    AssertMsg(data.isStop(source), "Source " << source << " is not a stop!");
    AssertMsg(data.isStop(target), "Target " << target << " is not a stop!");
    AssertMsg(minDepartureTime <= maxDepartureTime,
              "Minimum Departure Time needs to smaller or equal to the Maximum "
              "Departure Time!");
    run(StopId(source), StopId(target), minDepartureTime, maxDepartureTime);
  }

  inline void run(const StopId source, const StopId target,
                  const int minDepTime, const int maxDepTime) noexcept {
    profiler.start();
    sourceStop = source;
    targetStop = target;
    sourceCellId = data.getCellIdOfStop(sourceStop);
    targetCellId = data.getCellIdOfStop(targetStop);
    minDepartureTime = minDepTime;
    maxDepartureTime = maxDepTime;
    std::vector<RAPTOR::Journey> journeyOfRound;

    // clear everything
    clear();
    computeInitialAndFinalTransfers();
    evaluateInitialTransfers();
    scanTrips();
    journeyOfRound = getJourneys();
    allJourneys.insert(allJourneys.end(), journeyOfRound.begin(),
                       journeyOfRound.end());
    targetLabelChanged.assign(16, false);

    collectDepartures();
    // note: this vector is not duplicate free
    size_t i(0), j(0);
    while (i < collectedDepTimes.size()) {
      // perform one "normal" query
      queue.clear();
      while (j < collectedDepTimes.size() &&
             collectedDepTimes[i].depTime == collectedDepTimes[j].depTime) {
        enqueue(collectedDepTimes[j].trip,
                StopIndex(collectedDepTimes[j].stopIndex + 1),
                data.firstStopEventOfTrip[collectedDepTimes[j].trip],
                static_cast<std::uint32_t>(-1));
        ++j;
      }
      scanTrips();
      journeyOfRound = getJourneys();
      allJourneys.insert(allJourneys.end(), journeyOfRound.begin(),
                         journeyOfRound.end());
      i = j;
      targetLabelChanged.assign(16, false);
    }
    profiler.done();
  }

  inline void evaluateInitialTransfers() noexcept {
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
    auto &valuesToLoopOver = reachedRoutes.getValues();

    for (size_t i = 0; i < valuesToLoopOver.size(); ++i) {
#ifdef ENABLE_PREFETCH
      if (i + 16 < valuesToLoopOver.size()) {
        __builtin_prefetch(&(routeLabels[valuesToLoopOver[i + 16]]));
        __builtin_prefetch(&(data.firstTripOfRoute[valuesToLoopOver[i + 16]]));
      }
#endif

      const RouteId route = valuesToLoopOver[i];
      const RouteLabel &label = routeLabels[route];
      const StopIndex endIndex = label.end();
      const TripId firstTrip = data.firstTripOfRoute[route];
      const StopId *stops = data.raptorData.stopArrayOfRoute(route);
      TripId tripIndex = noTripId;
      for (StopIndex stopIndex(0); stopIndex < endIndex; stopIndex++) {
        const int timeFromSource = transferFromSource[stops[stopIndex]];
        if (timeFromSource == INFTY)
          continue;
        const int stopDepartureTime = 24 * 60 * 60 + timeFromSource;
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
          tripIndex--;
          while ((tripIndex > 0) &&
                 (label.departureTimes[labelIndex + tripIndex - 1] >=
                  stopDepartureTime)) {
            tripIndex--;
          }
        }
        const TripId thisTrip(firstTrip + tripIndex);
        enqueue(thisTrip, StopIndex(stopIndex + 1),
                data.firstStopEventOfTrip[thisTrip],
                static_cast<std::uint32_t>(-1));
        if (tripIndex == 0)
          break;
      }
    }
  }

  inline Profiler &getProfiler() noexcept { return profiler; }

  const std::vector<TripStopIndex> &getCollectedDepTimes() noexcept {
    return collectedDepTimes;
  }

  inline std::vector<RAPTOR::Journey> getAllJourneys() const noexcept {
    return allJourneys;
  }

  inline std::vector<RAPTOR::Journey> getJourneys() const noexcept {
    std::vector<RAPTOR::Journey> result;
    int bestArrivalTime = INFTY;
    int counter(0);
    AssertMsg(targetLabels.size() <= 16, "TargetLabel Size "
                                             << (int)targetLabels.size()
                                             << " is out of bounds!");
    for (const TargetLabel &label : targetLabels) {
      AssertMsg(counter < (int)targetLabelChanged.size(),
                "Counter " << counter << " is out of bounds!");
      if ((!targetLabelChanged[counter++]) ||
          label.arrivalTime >= bestArrivalTime)
        continue;
      bestArrivalTime = label.arrivalTime;
      result.emplace_back(getJourney(label));
    }
    return result;
  }

private:
  inline void clear() noexcept {
    queue.clear();
    tmpQueue.clear();
    targetCellQueue.clear();

    reachedIndex.clear();
    targetLabels.assign(16, TargetLabel());
    minArrivalTimeFastLookUp.assign(16, INFTY);
    allJourneys.clear();
    targetLabelChanged.assign(16, false);
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
      addTargetLabel(minDepartureTime);
    for (const Edge edge : reverseTransferGraph.edgesFrom(targetStop)) {
      const Vertex stop = reverseTransferGraph.get(ToVertex, edge);
      if (stop == sourceStop)
        addTargetLabel(minDepartureTime +
                       reverseTransferGraph.get(TravelTime, edge));
      transferToTarget[stop] = reverseTransferGraph.get(TravelTime, edge);
    }
    lastSource = sourceStop;
    lastTarget = targetStop;
    profiler.donePhase(PHASE_SCAN_INITIAL);
  }

  inline void collectDepartures() noexcept {
    profiler.startPhase();
    collectedDepTimes.clear();
    // get all reachable routes (meaning also by footpaths)
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
    for (const RouteId route : reachedRoutes) {
      const size_t numberOfStops = data.numberOfStopsInRoute(route);
      const StopId *stops = data.raptorData.stopArrayOfRoute(route);
      const RAPTOR::StopEvent *stopEvents =
          data.raptorData.firstTripOfRoute(route);
      const TripId firstTrip = data.firstTripOfRoute[route];
      for (size_t stopEventIndex = 0;
           stopEventIndex < data.raptorData.numberOfStopEventsInRoute(route);
           ++stopEventIndex) {
        if ((stopEventIndex + 1) % numberOfStops == 0)
          continue;
        const StopId stop = stops[stopEventIndex % numberOfStops];
        const int walkingTime = transferFromSource[stop];
        if (walkingTime == INFTY)
          continue;
        const int departureTime =
            stopEvents[stopEventIndex].departureTime - walkingTime;
        if (departureTime < minDepartureTime ||
            departureTime >= maxDepartureTime)
          continue;
        collectedDepTimes.push_back(TripStopIndex(
            TripId(firstTrip + (int)(stopEventIndex / numberOfStops)),
            StopIndex(stopEventIndex % numberOfStops), departureTime));
      }
    }
    // sort collectedDepTimes desc
    std::stable_sort(collectedDepTimes.begin(), collectedDepTimes.end(),
                     [](const TripStopIndex a, const TripStopIndex b) {
                       return a.depTime > b.depTime;
                     });
    profiler.donePhase(PHASE_COLLECT_DEPTIMES);
  }

  inline void scanTrips() noexcept {
    profiler.startPhase();
    std::size_t roundBegin = 0;
    std::size_t roundEnd = 0;
    uint8_t n = 1;

    const EventLookup *RESTRICT eventLookupPtr = eventLookup.data();
    const std::uint32_t *RESTRICT eventArrTimesPtr = eventArrTimes.data();
    const uint16_t *RESTRICT cellIdPtr = cellIdOfEvent.data();
    const auto *RESTRICT edgeRangeLookupPtr = edgeRangeLookup.data();

    while (!tmpQueue.empty() && n < 16) {
      profiler.countMetric(METRIC_ROUNDS);

      // loop over simple queue and split trip segments
      const std::size_t tmpQueueSize = tmpQueue.size();
      for (std::size_t i = 0; i < tmpQueueSize; ++i) {

#ifdef ENABLE_PREFETCH
        if (i + 8 < tmpQueueSize) {
          __builtin_prefetch(&edgeRangeLookupPtr[tmpQueue[i + 8].begin()]);
        }
#endif

        const auto &tripSegm = tmpQueue[i];
        StopEventId runner(tripSegm.begin());
        const StopEventId originalBoardingEvent = tripSegm.begin();

        while (runner < tripSegm.end()) {
          int lcl = static_cast<int>(std::min(
              std::bit_width<uint16_t>(cellIdPtr[runner] ^ sourceCellId),
              std::bit_width<uint16_t>(cellIdPtr[runner] ^ targetCellId)));

          AssertMsg(static_cast<std::size_t>(lcl) < overlayGraphs.size(),
                    "LCL value ("
                        << lcl
                        << ") cannot be used as index into overlayGraphs!");
          int lowerLcl = std::max(lcl - 1, 0);
          const StopEventId nextStopEventOutside =
              edgeRangeLookupPtr[runner][lowerLcl];

          const StopEventId endOfConsecutiveLCL =
              std::min(nextStopEventOutside, tripSegm.end());

          queue.emplace(runner, endOfConsecutiveLCL, tripSegm.parent(),
                        originalBoardingEvent, lcl);
          if (cellIdPtr[runner] == targetCellId) [[unlikely]] {
            const std::size_t idx = queue.size() - 1;
            targetCellQueue.emplace(runner, endOfConsecutiveLCL, idx);
          }

          runner = endOfConsecutiveLCL;
        }
      }

      // try to update arrival times by only processing level zero trip
      // segments
      const std::size_t targetCellQueueSize = targetCellQueue.size();
      for (std::size_t i = 0; i < targetCellQueueSize; ++i) {
        const QueueElementTargetCell &label = targetCellQueue[i];
        profiler.countMetric(METRIC_SCANNED_LEVEL_ZERO_TRIPS);

#ifdef ENABLE_PREFETCH
        if (i + 16 < targetCellQueueSize) {
          __builtin_prefetch(&eventLookupPtr[targetCellQueue[i + 16].begin]);
        }
#endif

        for (StopEventId j = label.begin; j < label.end;) {
          profiler.countMetric(METRIC_SCANNED_LEVEL_ZERO_STOPS);
          if (eventLookupPtr[j].arrTime >=
              static_cast<uint32_t>(minArrivalTimeFastLookUp[n]))
            break;
          const int timeToTarget = transferToTarget[eventLookupPtr[j].stop];
          if (timeToTarget != INFTY) {
            addTargetLabel(eventLookupPtr[j].arrTime + timeToTarget,
                           label.originalId, n);
          }
          j++;
        }
      }

      if (n == 15)
        break;

      roundBegin = roundEnd;
      roundEnd = queue.size();

      tmpQueue.clear();
      targetCellQueue.clear();

      for (size_t i = roundBegin; i < roundEnd; i++) {
#ifdef ENABLE_PREFETCH
        if (i + 16 < roundEnd) {
          __builtin_prefetch(&eventArrTimesPtr[queue[i + 16].begin()]);
        }
#endif

        profiler.countMetric(METRIC_SCANNED_STOPS);
        TripLabel &label = queue[i];
        bool tooLate = (eventArrTimesPtr[label.begin()] >=
                        static_cast<uint32_t>(minArrivalTimeFastLookUp[n]));
        label.setEnd(tooLate ? label.begin() : label.end());
      }

      for (size_t i = roundBegin; i < roundEnd; i++) {
#ifdef ENABLE_PREFETCH
        if (i + 16 < roundEnd) {
          overlayGraphs[queue[i + 16].lcl()].prefetchAdj(queue[i + 16].begin());
          overlayGraphs[queue[i + 16].lcl()].prefetchAdj(queue[i + 16].end());
        }
#endif

        profiler.countMetric(METRIC_SCANNED_TRIPS);
        const TripLabel &label = queue[i];

        if (label.begin() == label.end()) [[unlikely]] {
          continue;
        }

        AssertMsg(label.lcl() < overlayGraphs.size(),
                  "Label.lcl (" << (int)label.lcl() << ") is out of bounds!");

        const auto &currentGraph = overlayGraphs[label.lcl()];

        const std::size_t beginEdgeRange =
            currentGraph.beginEdge(Vertex(label.begin()));
        const std::size_t endEdgeRange =
            currentGraph.beginEdge(Vertex(label.end()));

        const EdgeLabel *RESTRICT edgeLabelsPtr =
            edgeLabels[label.lcl()].data();

        for (std::size_t edge = beginEdgeRange; edge < endEdgeRange; ++edge) {
#ifdef ENABLE_PREFETCH
          if (edge + 16 < endEdgeRange) {
            __builtin_prefetch(&edgeLabelsPtr[edge + 16]);
          }
#endif

          profiler.countMetric(METRIC_RELAXED_TRANSFERS);
          const EdgeLabel edgeLabel = edgeLabelsPtr[edge];
          enqueue(edgeLabel.getTrip(), edgeLabel.getStopIndex(),
                  edgeLabel.getFirstEvent(), i, n);
        }
      }

      ++n;
    }
    profiler.donePhase(PHASE_SCAN_TRIPS);
  }

  inline void enqueue(const TripId trip, const StopIndex index,
                      const StopEventId firstEvent, const std::uint32_t parent,
                      const int n = 0) noexcept {
    profiler.countMetric(METRIC_ENQUEUES);
    const StopIndex endOfTripSeg = StopIndex(reachedIndex(trip, n + 1));

    if (endOfTripSeg <= index) [[likely]] {
      return;
    }

    reachedIndex.update(trip, index, n + 1);

    const StopEventId beginStopEventId = StopEventId(firstEvent + index);
    const StopEventId endStopEventId = StopEventId(firstEvent + endOfTripSeg);

    AssertMsg(beginStopEventId < endStopEventId, "Begin should be < End!");
    AssertMsg(beginStopEventId < data.numberOfStopEvents(),
              "StopEvent out of bounds!");
    AssertMsg(endStopEventId <= data.numberOfStopEvents(),
              "StopEvent out of bounds!");

    tmpQueue.emplace(beginStopEventId, endStopEventId, parent);
  }

  inline void addTargetLabel(const int newArrivalTime,
                             const u_int32_t parent = -1,
                             const u_int8_t n = 0) noexcept {
    profiler.countMetric(METRIC_ADD_JOURNEYS);
    AssertMsg(n < minArrivalTimeFastLookUp.size(),
              "n " << (int)n << " is out of bounds!");
    if ((uint32_t)newArrivalTime < minArrivalTimeFastLookUp[n]) {
      targetLabels[n].arrivalTime = newArrivalTime;
      targetLabels[n].parent = parent;

      targetLabelChanged[n] = true;

      minArrivalTimeFastLookUp[n] = (uint32_t)newArrivalTime;
// std::fill(minArrivalTimeFastLookUp.begin() + n,
// minArrivalTimeFastLookUp.end(), newArrivalTime);

// do i need to remove everything with greater n and greater arrivaltime? =>
// yes, i think
#pragma omp simd
      for (int i = n + 1; i < 16; ++i) {
        if (targetLabels[i].arrivalTime > targetLabels[n].arrivalTime) {
          targetLabels[i].clear();
        }
        if (minArrivalTimeFastLookUp[i] > (uint32_t)newArrivalTime) {
          minArrivalTimeFastLookUp[i] = (uint32_t)newArrivalTime;
        }
      }
    }
  }

  inline RAPTOR::Journey
  getJourney(const TargetLabel &targetLabel) const noexcept {
    RAPTOR::Journey result;
    u_int32_t parent = targetLabel.parent;
    if (parent == u_int32_t(-1)) {
      result.emplace_back(sourceStop, targetStop, minDepartureTime,
                          targetLabel.arrivalTime, false);
      return result;
    }
    StopEventId departureStopEvent = noStopEvent;
    Vertex departureStop = targetStop;
    int lastTime(minDepartureTime);
    while (parent != u_int32_t(-1)) {
      AssertMsg(parent < queue.size(),
                "Parent " << parent << " is out of range!");
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

      departureStopEvent = StopEventId(label.boardingEvent() - 1);
      departureStop = data.getStopOfStopEvent(departureStopEvent);
      const RouteId route = data.getRouteOfStopEvent(departureStopEvent);
      const int departureTime =
          data.raptorData.stopEvents[departureStopEvent].departureTime;
      lastTime = departureTime;
      result.emplace_back(departureStop, arrivalStop, departureTime,
                          arrivalTime, true, route);

      parent = label.parent();
    }
    const int timeFromSource = transferFromSource[departureStop];
    result.emplace_back(sourceStop, departureStop, lastTime - timeFromSource,
                        lastTime, noEdge);
    Vector::reverse(result);
    return result;
  }

  inline std::pair<StopEventId, Edge>
  getParent(const TripLabel &parentLabel,
            const StopEventId departureStopEvent) const noexcept {
    int lcl = parentLabel.lcl();
    AssertMsg(static_cast<std::size_t>(lcl) < overlayGraphs.size(),
              "LCL is out of bounds!");
    const auto &currentGraph = overlayGraphs[lcl];

    for (StopEventId i = parentLabel.begin(); i < parentLabel.end(); ++i) {
      const std::size_t beginEdgeRange = currentGraph.beginEdge(Vertex(i));
      const std::size_t endEdgeRange = currentGraph.beginEdge(Vertex(i + 1));

      for (std::size_t edge = beginEdgeRange; edge < endEdgeRange; ++edge) {
        if (edgeLabels[lcl][edge].getStopEvent() == departureStopEvent)
          return std::make_pair(i, Edge(edge));
      }
    }
    Ensure(false, "Could not find parent stop event using departureStopEvent!");
    return std::make_pair(noStopEvent, noEdge);
  }

  /*
  inline std::pair<StopEventId, Edge>
  getParent(const TripLabel &parentLabel,
            const StopEventId departureStopEvent) const noexcept {
    int lcl = parentLabel.lcl();
    const auto &currentGraph = overlayGraphs[lcl];

    // Use full trip extent, not the trimmed label.end()
    const TripId trip = data.tripOfStopEvent[parentLabel.begin()];
    const StopEventId fullEnd = data.firstStopEventOfTrip[trip + 1];

    for (StopEventId i = parentLabel.begin(); i < fullEnd; ++i) {
      const std::size_t beginEdgeRange = currentGraph.beginEdge(Vertex(i));
      const std::size_t endEdgeRange = currentGraph.beginEdge(Vertex(i + 1));

      for (std::size_t edge = beginEdgeRange; edge < endEdgeRange; ++edge) {
        if (edgeLabels[lcl][edge].getStopEvent() == departureStopEvent)
          return std::make_pair(i, Edge(edge));
      }
    }
    Ensure(false, "Could not find parent stop event using departureStopEvent!");
    return std::make_pair(noStopEvent, noEdge);
  }
  */

  inline std::pair<StopEventId, Edge>
  getParent(const TripLabel &parentLabel,
            const TargetLabel &targetLabel) const noexcept {
    // Final transfer to target may start exactly at parentLabel.end if it has
    // length 0
    const TripId trip = data.tripOfStopEvent[parentLabel.begin()];
    const StopEventId end = data.firstStopEventOfTrip[trip + 1];
    for (StopEventId i = parentLabel.begin(); i < end; ++i) {
      const int timeToTarget = transferToTarget[eventLookup[i].stop];
      if (timeToTarget == INFTY)
        continue;
      if ((int)(eventLookup[i].arrTime + timeToTarget) ==
          targetLabel.arrivalTime)
        return std::make_pair(i, noEdge);
    }
    Ensure(false, "Could not find parent stop event using TargetLabel!");
    return std::make_pair(noStopEvent, noEdge);
  }

private:
  const TREXData &data;

  TransferGraph reverseTransferGraph;
  std::vector<int> transferFromSource;
  std::vector<int> transferToTarget;
  StopId lastSource;
  StopId lastTarget;

  uint16_t sourceCellId;
  uint16_t targetCellId;

  IndexedSet<false, RouteId> reachedRoutes;

  PreallocatedQueue<TripLabel> queue;
  PreallocatedQueue<TripLabel> tmpQueue;
  PreallocatedQueue<QueueElementTargetCell> targetCellQueue;

  std::vector<EdgeRange> edgeRanges;

#ifdef USE_SIMD
  ProfileReachedIndexSIMD reachedIndex;
#else
  ProfileReachedIndex reachedIndex;
#endif

  std::vector<TargetLabel> targetLabels;
  std::vector<std::uint32_t> minArrivalTimeFastLookUp;

  std::vector<std::vector<EdgeLabel>> edgeLabels;
  std::vector<EventLookup> eventLookup;
  std::vector<std::uint32_t> eventArrTimes;
  std::vector<uint16_t> cellIdOfEvent;

  StopId sourceStop;
  StopId targetStop;
  int minDepartureTime;
  int maxDepartureTime;

  std::vector<TripStopIndex> collectedDepTimes;
  std::vector<RAPTOR::Journey> allJourneys;
  std::vector<bool> targetLabelChanged;

  std::vector<RouteLabel> routeLabels;
  Profiler profiler;

  std::vector<SimpleGraph<std::uint32_t>> overlayGraphs;

  std::vector<std::array<StopEventId, 17>> edgeRangeLookup;
};

} // namespace TripBased
