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
#include <execution>
#include <vector>

#include "../../../DataStructures/Container/Queue.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/Graph/SimpleGraph.h"
#include "../../../DataStructures/Graph/Utils/Conversion.h"
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../../DataStructures/TREX/TREXData.h"
#include "../../TripBased/Query/Profiler.h"
#include "../../TripBased/Query/ReachedIndex.h"
#include "../../TripBased/Query/Types.h"

#if defined(__GNUC__) || defined(__clang__)
#define RESTRICT __restrict
#elif defined(_MSC_VER)
#define RESTRICT __restrict
#else
#define RESTRICT
#endif

namespace TripBased {

template <typename PROFILER = NoProfiler>
class TREXQueryOverlay {
public:
    using Profiler = PROFILER;
    using Type = TREXQueryOverlay<Profiler>;

private:
    struct QueueElementTargetCell {
        QueueElementTargetCell(const StopEventId begin = noStopEvent, const StopEventId end = noStopEvent,
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
            uint64_t lcl : 10;  // Extra room for levels or flags
        } _meta;

    public:
        TripLabel(const StopEventId begin = noStopEvent, const StopEventId end = noStopEvent,
                  const uint32_t parent = -1, const StopEventId boardingEvent = noStopEvent, const uint8_t lcl = 0)
            : _begin(begin), _end(end) {
            setParent(parent);
            setBoardingEvent(boardingEvent);
            setLcl(lcl);
        }

        inline StopEventId begin() const noexcept { return StopEventId(_begin); }
        inline StopEventId end() const noexcept { return StopEventId(_end); }

        inline uint32_t parent() const noexcept {
            return (_meta.parent == 0x7FFFFFF) ? uint32_t(-1) : uint32_t(_meta.parent);
        }

        inline uint8_t lcl() const noexcept { return static_cast<uint8_t>(_meta.lcl); }

        inline StopEventId boardingEvent() const noexcept { return StopEventId(_meta.boarding); }

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

    struct TargetLabel {
        TargetLabel(const int arrivalTime = INFTY, const uint32_t parent = -1)
            : arrivalTime(arrivalTime), parent(parent) {}

        int arrivalTime;
        uint32_t parent;
    };

public:
    TREXQueryOverlay(TREXData& data)
        : data(data),
          transfers(data),
          cellIdOfStop(data.cellIds),
          cellIdOfEvent(data.numberOfStopEvents(), 0),
          transferFromSource(data.numberOfStops(), INFTY),
          transferToTarget(data.numberOfStops(), INFTY),
          lastSource(StopId(0)),
          lastTarget(StopId(0)),
          reachedRoutes(data.numberOfRoutes()),
          queue(data.numberOfStopEvents()),
          tmpQueue(data.numberOfStopEvents()),
          targetCellQueue(data.numberOfStopEvents()),
          reachedIndex(data),
          targetLabels(1),
          minArrivalTime(INFTY),
          sourceStop(noStop),
          targetStop(noStop),
          sourceDepartureTime(never),
          edgeRangeLookup(data) {
#pragma omp parallel for
        for (size_t event = 0; event < data.numberOfStopEvents(); ++event) {
            const StopId stop = data.getStopOfStopEvent(StopEventId(event));
            AssertMsg(data.raptorData.isStop(stop), "Stop is not a stop!");
            cellIdOfEvent[event] = (uint16_t)data.getCellIdOfStop(stop);
        }

        profiler.registerPhases({PHASE_SCAN_INITIAL, PHASE_EVALUATE_INITIAL, PHASE_SCAN_TRIPS, PHASE_GET_JOURNEYS});
        profiler.registerMetrics({METRIC_ROUNDS, METRIC_SCANNED_TRIPS, METRIC_SCANNED_LEVEL_ZERO_TRIPS,
                                  METRIC_SCANNED_STOPS, METRIC_SCANNED_LEVEL_ZERO_STOPS, METRIC_RELAXED_TRANSFERS,
                                  METRIC_ENQUEUES, METRIC_ADD_JOURNEYS, DISCARDED_EDGE});
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target) noexcept {
        run(StopId(source), departureTime, StopId(target));
    }

    inline void run(const StopId source, const int departureTime, const StopId target) noexcept {
        profiler.start();
        clear();
        sourceStop = source;
        targetStop = target;
        sourceCellId = cellIdOfStop[sourceStop];
        targetCellId = cellIdOfStop[targetStop];
        sourceDepartureTime = departureTime;

        computeInitialAndFinalTransfers();
        evaluateInitialTransfers();
        scanTrips(16);
        profiler.done();
    }

    inline std::vector<RAPTOR::Journey> getJourneys() noexcept {
        profiler.startPhase();
        std::vector<RAPTOR::Journey> result;
        int bestArrivalTime = INFTY;
        for (const TargetLabel& label : targetLabels) {
            if (label.arrivalTime >= bestArrivalTime) continue;
            bestArrivalTime = label.arrivalTime;
            result.emplace_back(getJourney(label));
        }
        profiler.donePhase(PHASE_GET_JOURNEYS);
        return result;
    }

    inline RAPTOR::Journey getJourney(const TargetLabel& targetLabel) const noexcept {
        RAPTOR::Journey result;
        uint32_t parent = targetLabel.parent;
        if (parent == uint32_t(-1)) {
            result.emplace_back(sourceStop, targetStop, sourceDepartureTime, targetLabel.arrivalTime, false);
            return result;
        }
        StopEventId departureStopEvent = noStopEvent;
        Vertex departureStop = targetStop;
        int lastTime(sourceDepartureTime);
        while (parent != uint32_t(-1)) {
            AssertMsg(parent < queue.size(), "Parent " << parent << " is out of range!");
            const TripLabel& label = queue[parent];
            StopEventId arrivalStopEvent;
            Edge edge;
            std::tie(arrivalStopEvent, edge) = (departureStopEvent == noStopEvent)
                                                   ? getParent(label, targetLabel)
                                                   : getParent(label, StopEventId(departureStopEvent + 1));

            const StopId arrivalStop = data.eventLookup[arrivalStopEvent].stop;
            const int arrivalTime = data.eventLookup[arrivalStopEvent].arrTime;
            const int transferArrivalTime =
                (edge == noEdge) ? targetLabel.arrivalTime : arrivalTime + transfers.travelTime[edge];
            result.emplace_back(arrivalStop, departureStop, arrivalTime, transferArrivalTime, edge);

            departureStopEvent = StopEventId(label.boardingEvent() - 1);
            departureStop = data.eventLookup[departureStopEvent].stop;
            const RouteId route = data.getRouteOfStopEvent(departureStopEvent);
            const int departureTime = data.eventDepTimes[departureStopEvent];
            lastTime = departureTime;
            result.emplace_back(departureStop, arrivalStop, departureTime, arrivalTime, true, route);

            parent = label.parent();
        }
        const int timeFromSource = transferFromSource[departureStop];
        result.emplace_back(sourceStop, departureStop, sourceDepartureTime + timeFromSource, lastTime, noEdge);
        Vector::reverse(result);
        return result;
    }

    inline std::pair<StopEventId, Edge> getParent(const TripLabel& parentLabel,
                                                  const StopEventId departureStopEvent) const noexcept {
        for (StopEventId i = parentLabel.begin(); i < parentLabel.end(); ++i) {
            for (Edge edge = transfers.beginOut[0][i]; edge < transfers.beginOut[0][i + 1]; ++edge) {
                if (transfers.labels[0][edge].getStopEvent() == departureStopEvent)
                    return std::make_pair(i, Edge(edge));
            }
        }
        Ensure(false, "Could not find parent stop event!");
        return std::make_pair(noStopEvent, noEdge);
    }

    inline std::pair<StopEventId, Edge> getParent(const TripLabel& parentLabel,
                                                  const TargetLabel& targetLabel) const noexcept {
        // Final transfer to target may start exactly at parentLabel.end if it has length 0
        const TripId trip = data.tripOfStopEvent[parentLabel.begin()];
        const StopEventId end = data.firstStopEventOfTrip[trip + 1];
        for (StopEventId i = parentLabel.begin(); i < end; ++i) {
            const int timeToTarget = transferToTarget[data.eventLookup[i].stop];
            if (timeToTarget == INFTY) continue;
            if (static_cast<int>(data.eventLookup[i].arrTime) + timeToTarget == targetLabel.arrivalTime)
                return std::make_pair(i, noEdge);
        }
        Ensure(false, "Could not find parent stop event!");
        return std::make_pair(noStopEvent, noEdge);
    }

    inline std::vector<RAPTOR::ArrivalLabel> getArrivals() const noexcept {
        std::vector<RAPTOR::ArrivalLabel> result;
        for (size_t i = 0; i < targetLabels.size(); ++i) {
            if (targetLabels[i].arrivalTime >= INFTY) continue;
            if ((result.size() >= 1) && (result.back().arrivalTime == targetLabels[i].arrivalTime)) continue;
            result.emplace_back(targetLabels[i].arrivalTime, i);
        }
        return result;
    }

    inline Profiler& getProfiler() noexcept { return profiler; }

private:
    inline void clear() noexcept {
        queue.clear();
        tmpQueue.clear();
        targetCellQueue.clear();

        reachedIndex.clear();
        targetLabels.resize(1);
        targetLabels[0] = TargetLabel();
        minArrivalTime = INFTY;
    }

    inline void computeInitialAndFinalTransfers() noexcept {
        profiler.startPhase();
        transferFromSource[lastSource] = INFTY;
        for (const Edge edge : data.transferGraph.edgesFrom(lastSource)) {
            const Vertex stop = data.transferGraph.get(ToVertex, edge);
            transferFromSource[stop] = INFTY;
        }
        transferToTarget[lastTarget] = INFTY;
        for (const Edge edge : data.reverseTransferGraph.edgesFrom(lastTarget)) {
            const Vertex stop = data.reverseTransferGraph.get(ToVertex, edge);
            transferToTarget[stop] = INFTY;
        }
        transferFromSource[sourceStop] = 0;
        for (const Edge edge : data.transferGraph.edgesFrom(sourceStop)) {
            const Vertex stop = data.transferGraph.get(ToVertex, edge);
            transferFromSource[stop] = data.transferGraph.get(TravelTime, edge);
        }
        transferToTarget[targetStop] = 0;
        if (sourceStop == targetStop) addTargetLabel(sourceDepartureTime);
        for (const Edge edge : data.reverseTransferGraph.edgesFrom(targetStop)) {
            const Vertex stop = data.reverseTransferGraph.get(ToVertex, edge);
            if (stop == sourceStop) addTargetLabel(sourceDepartureTime + data.reverseTransferGraph.get(TravelTime, edge));
            transferToTarget[stop] = data.reverseTransferGraph.get(TravelTime, edge);
        }
        lastSource = sourceStop;
        lastTarget = targetStop;
        profiler.donePhase(PHASE_SCAN_INITIAL);
    }

    inline void evaluateInitialTransfers() noexcept {
        profiler.startPhase();
        reachedRoutes.clear();
        for (const RAPTOR::RouteSegment& route : data.routesContainingStop(sourceStop)) {
            reachedRoutes.insert(route.routeId);
        }
        for (const Edge edge : data.transferGraph.edgesFrom(sourceStop)) {
            const Vertex stop = data.transferGraph.get(ToVertex, edge);
            for (const RAPTOR::RouteSegment& route : data.routesContainingStop(StopId(stop))) {
                reachedRoutes.insert(route.routeId);
            }
        }
        reachedRoutes.sort();
        auto& routesToLoopOver = reachedRoutes.getValues();
        for (size_t i(0); i < routesToLoopOver.size(); ++i) {
            const RouteId route = routesToLoopOver[i];

#ifdef ENABLE_PREFETCH
            if (i + 16 < routesToLoopOver.size()) {
                __builtin_prefetch(&data.routeLabels[routesToLoopOver[i + 16]]);
                __builtin_prefetch(&data.firstTripOfRoute[routesToLoopOver[i + 16]]);
                __builtin_prefetch(data.stopArrayOfRoute(routesToLoopOver[i + 16]));
            }
#endif
            const RouteLabel& label = data.routeLabels[route];
            const StopIndex endIndex = label.end();
            const TripId firstTrip = data.firstTripOfRoute[route];
            const StopId* stops = data.stopArrayOfRoute(route);
            TripId tripIndex = noTripId;
            for (StopIndex stopIndex(0); stopIndex < endIndex; stopIndex++) {
                const int timeFromSource = transferFromSource[stops[stopIndex]];
                if (timeFromSource == INFTY) continue;
                const int stopDepartureTime = sourceDepartureTime + timeFromSource;
                const uint32_t labelIndex = stopIndex * label.numberOfTrips;
                if (tripIndex >= label.numberOfTrips) {
                    tripIndex = std::lower_bound(TripId(0), TripId(label.numberOfTrips), stopDepartureTime,
                                                 [&](const TripId trip, const int time) {
                                                     return label.departureTimes[labelIndex + trip] < time;
                                                 });
                    if (tripIndex >= label.numberOfTrips) continue;
                } else {
                    if (label.departureTimes[labelIndex + tripIndex - 1] < stopDepartureTime) continue;
                    --tripIndex;
                    while ((tripIndex > 0) && (label.departureTimes[labelIndex + tripIndex - 1] >= stopDepartureTime)) {
                        --tripIndex;
                    }
                }
                const TripId thisTrip(firstTrip + tripIndex);
                enqueue(firstTrip + tripIndex, StopIndex(stopIndex + 1), data.firstStopEventOfTrip[thisTrip],
                        static_cast<std::uint32_t>(-1));
                if (tripIndex == 0) break;
            }
        }
        profiler.donePhase(PHASE_EVALUATE_INITIAL);
    }

    inline void scanTrips(const uint8_t MAX_ROUNDS = 16) noexcept {
        profiler.startPhase();
        uint8_t currentRoundNumber = 0;

        std::size_t roundBegin = 0;
        std::size_t roundEnd = 0;

        const EventLookup* RESTRICT eventLookupPtr = data.eventLookup.data();
        const std::uint32_t* RESTRICT eventArrTimesPtr = data.eventArrTimes.data();
        const uint16_t* RESTRICT cellIdPtr = cellIdOfEvent.data();
        const auto* RESTRICT edgeRangeLookupPtr = edgeRangeLookup.nextEvent.data();

        while (!tmpQueue.empty() && currentRoundNumber < MAX_ROUNDS) {
            ++currentRoundNumber;

            profiler.countMetric(METRIC_ROUNDS);
            targetLabels.emplace_back(targetLabels.back());

            // loop over simple queue and split trip segments
            const std::size_t tmpQueueSize = tmpQueue.size();
            for (std::size_t i = 0; i < tmpQueueSize; ++i) {
#ifdef ENABLE_PREFETCH
                if (i + 8 < tmpQueueSize) {
                    __builtin_prefetch(&edgeRangeLookupPtr[tmpQueue[i + 8].begin()]);
                }
#endif

                const auto& tripSegm = tmpQueue[i];
                StopEventId runner(tripSegm.begin());
                const StopEventId originalBoardingEvent = tripSegm.begin();

                while (runner < tripSegm.end()) {
                    int lcl = static_cast<int>(std::min(std::bit_width<uint16_t>(cellIdPtr[runner] ^ sourceCellId),
                                                        std::bit_width<uint16_t>(cellIdPtr[runner] ^ targetCellId)));

                    int lowerLcl = std::max(lcl - 1, 0);
                    const StopEventId nextStopEventOutside = edgeRangeLookupPtr[runner][lowerLcl];

                    const StopEventId endOfConsecutiveLCL = std::min(nextStopEventOutside, tripSegm.end());

                    queue.emplace(runner, endOfConsecutiveLCL, tripSegm.parent(), originalBoardingEvent, lcl);
                    if (cellIdPtr[runner] == targetCellId) [[unlikely]] {
                        const std::size_t idx = queue.size() - 1;
                        targetCellQueue.emplace(runner, endOfConsecutiveLCL, idx);
                    }

                    runner = endOfConsecutiveLCL;
                }
            }

            // try to update arrival times by only processing level zero trip segments
            const std::size_t targetCellQueueSize = targetCellQueue.size();
            for (std::size_t i = 0; i < targetCellQueueSize; ++i) {
                const QueueElementTargetCell& label = targetCellQueue[i];
                profiler.countMetric(METRIC_SCANNED_LEVEL_ZERO_TRIPS);

#ifdef ENABLE_PREFETCH
                if (i + 16 < targetCellQueueSize) {
                    __builtin_prefetch(&eventLookupPtr[targetCellQueue[i + 16].begin]);
                }
#endif

                for (StopEventId j = label.begin; j < label.end;) {
                    profiler.countMetric(METRIC_SCANNED_LEVEL_ZERO_STOPS);
                    if (eventLookupPtr[j].arrTime >= static_cast<uint32_t>(minArrivalTime)) break;
                    const int timeToTarget = transferToTarget[eventLookupPtr[j].stop];
                    if (timeToTarget != INFTY) {
                        addTargetLabel(eventLookupPtr[j].arrTime + timeToTarget, label.originalId);
                    }
                    j++;
                }
            }

            roundBegin = roundEnd;
            roundEnd = queue.size();

            tmpQueue.clear();
            targetCellQueue.clear();

            if (currentRoundNumber == MAX_ROUNDS) break;

            for (size_t i = roundBegin; i < roundEnd; i++) {
#ifdef ENABLE_PREFETCH
                if (i + 16 < roundEnd) {
                    __builtin_prefetch(&eventArrTimesPtr[queue[i + 16].begin()]);
                }
#endif

                profiler.countMetric(METRIC_SCANNED_STOPS);
                TripLabel& label = queue[i];
                bool tooLate = (eventArrTimesPtr[label.begin()] >= static_cast<uint32_t>(minArrivalTime));
                label.setEnd(tooLate ? label.begin() : label.end());
            }

            for (size_t i = roundBegin; i < roundEnd; i++) {
#ifdef ENABLE_PREFETCH
                if (i + 16 < roundEnd) {
                    __builtin_prefetch(&transfers.beginOut[queue[i + 16].lcl()][queue[i + 16].begin()]);
                    __builtin_prefetch(&transfers.beginOut[queue[i + 16].lcl()][queue[i + 16].end()]);
                }
#endif

                profiler.countMetric(METRIC_SCANNED_TRIPS);
                const TripLabel& label = queue[i];

                if (label.begin() == label.end()) [[unlikely]] {
                    continue;
                }

                AssertMsg(label.lcl() < transfers.beginOut.size(),
                          "Label.lcl (" << (int)label.lcl() << ") is out of bounds!");

                const auto& outgoingTransfers = transfers.beginOut[label.lcl()];

                const Edge beginEdgeRange = outgoingTransfers[label.begin()];
                const Edge endEdgeRange = outgoingTransfers[label.end()];

                const EdgeLabel* RESTRICT edgeLabelsPtr = transfers.labels[label.lcl()].data();

                for (std::size_t edge = beginEdgeRange; edge < endEdgeRange; ++edge) {
#ifdef ENABLE_PREFETCH
                    if (edge + 16 < endEdgeRange) {
                        __builtin_prefetch(&edgeLabelsPtr[edge + 16]);
                    }
#endif

                    profiler.countMetric(METRIC_RELAXED_TRANSFERS);
                    const EdgeLabel edgeLabel = edgeLabelsPtr[edge];
                    enqueue(edgeLabel.getTrip(), edgeLabel.getStopIndex(), edgeLabel.getFirstEvent(), i);
                }
            }
        }
        profiler.donePhase(PHASE_SCAN_TRIPS);
    }

    inline void enqueue(const TripId trip, const StopIndex index, const StopEventId firstEvent,
                        const std::uint32_t parent) noexcept {
        profiler.countMetric(METRIC_ENQUEUES);
        const StopIndex endOfTripSeg = StopIndex(reachedIndex(trip));

        if (endOfTripSeg <= index) [[likely]] {
            return;
        }

        reachedIndex.update(trip, index);

        const StopEventId beginStopEventId = StopEventId(firstEvent + index);
        const StopEventId endStopEventId = StopEventId(firstEvent + endOfTripSeg);

        AssertMsg(beginStopEventId < endStopEventId, "Begin should be < End!");

        tmpQueue.emplace(beginStopEventId, endStopEventId, parent);
    }

    inline void addTargetLabel(const int newArrivalTime, const uint32_t parent = -1) noexcept {
        profiler.countMetric(METRIC_ADD_JOURNEYS);
        if (newArrivalTime < targetLabels.back().arrivalTime) {
            targetLabels.back() = TargetLabel(newArrivalTime, parent);
            minArrivalTime = newArrivalTime;
        }
    }

public:
    inline void printMemoryConsumption() const noexcept {
        std::cout << "DataStructure,Bytes,KB,MB" << std::endl;

        long long total = 0;
        auto row = [&](const std::string& name, long long bytes, const bool addToTotal = true) {
            if (addToTotal) total += bytes;
            const double kb = bytes / 1024.0;
            const double mb = kb / 1024.0;
            std::cout << name << "," << bytes << "," << kb << "," << mb << std::endl;
        };

        row("data.transferGraph", data.transferGraph.memoryUsageInBytes());
        row("data.reverseTransferGraph", data.reverseTransferGraph.memoryUsageInBytes());
        row("data.eventLookup", Vector::memoryUsageInBytes(data.eventLookup));
        row("data.eventArrTimes", Vector::memoryUsageInBytes(data.eventArrTimes));
        row("data.eventDepTimes", Vector::memoryUsageInBytes(data.eventDepTimes));
        row("data.tripOfStopEvent", Vector::memoryUsageInBytes(data.tripOfStopEvent));
        row("data.routeOfTrip", Vector::memoryUsageInBytes(data.routeOfTrip));
        row("data.firstStopEventOfTrip", Vector::memoryUsageInBytes(data.firstStopEventOfTrip));
        row("data.firstTripOfRoute", Vector::memoryUsageInBytes(data.firstTripOfRoute));
        row("data.firstRouteSegmentOfStop", Vector::memoryUsageInBytes(data.firstRouteSegmentOfStop));
        row("data.routeSegments", Vector::memoryUsageInBytes(data.routeSegments));
        row("data.firstStopIdOfRoute", Vector::memoryUsageInBytes(data.firstStopIdOfRoute));
        row("data.routeStopSequences", Vector::memoryUsageInBytes(data.routeStopSequences));
        std::size_t routeLabelBytes = Vector::memoryUsageInBytes(data.routeLabels);
        for (const auto& rl : data.routeLabels) routeLabelBytes += Vector::memoryUsageInBytes(rl.departureTimes);
        row("data.routeLabels", routeLabelBytes);

        row("transfers.beginOut", Vector::memoryUsageInBytes(transfers.beginOut));
        row("transfers.labels", Vector::memoryUsageInBytes(transfers.labels));
        row("transfers.travelTime", Vector::memoryUsageInBytes(transfers.travelTime));

        row("cellIdOfStop", Vector::memoryUsageInBytes(cellIdOfStop));
        row("cellIdOfEvent", Vector::memoryUsageInBytes(cellIdOfEvent));
        row("transferFromSource", Vector::memoryUsageInBytes(transferFromSource));
        row("transferToTarget", Vector::memoryUsageInBytes(transferToTarget));
        row("reachedRoutes", reachedRoutes.memoryUsageInBytes());
        row("queue", queue.memoryUsageInBytes());
        row("tmpQueue", tmpQueue.memoryUsageInBytes());
        row("targetCellQueue", targetCellQueue.memoryUsageInBytes());
        row("reachedIndex", reachedIndex.memoryUsageInBytes());
        row("targetLabels", Vector::memoryUsageInBytes(targetLabels));
        row("edgeRangeLookup", Vector::memoryUsageInBytes(edgeRangeLookup.nextEvent));

        long long otherStuff = 4 * sizeof(StopId) + 2 * sizeof(uint16_t) + 2 * sizeof(int);
        row("rest", otherStuff);

        row("TOTAL", total, false);
    }

private:
    QueryData data;
    TransfersWithOverlays transfers;
    std::vector<uint16_t> cellIdOfStop;
    std::vector<uint16_t> cellIdOfEvent;

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
    ReachedIndex reachedIndex;

    std::vector<TargetLabel> targetLabels;
    int minArrivalTime;

    StopId sourceStop;
    StopId targetStop;
    int sourceDepartureTime;

    Profiler profiler;

    EdgeRangeLookup edgeRangeLookup;
};

}  // namespace TripBased
