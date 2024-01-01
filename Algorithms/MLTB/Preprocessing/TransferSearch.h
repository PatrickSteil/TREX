#pragma once

/* #include "../../../DataStructures/Container/Set.h" */
#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/MLTB/MLData.h"
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../TripBased/Query/Profiler.h"
#include "../../TripBased/Query/ReachedIndex.h"

namespace TripBased {

template <typename PROFILER = NoProfiler>
class TransferSearch {
public:
    using Profiler = PROFILER;
    using Type = TransferSearch<Profiler>;

private:
    struct TripLabel {
        TripLabel(const StopEventId begin = noStopEvent, const StopEventId end = noStopEvent,
            const u_int32_t parent = -1, const Edge parentTransfer = noEdge)
            : begin(begin)
            , end(end)
            , parent(parent)
            , parentTransfer(parentTransfer)
        {
        }
        StopEventId begin;
        StopEventId end;
        u_int32_t parent;
        Edge parentTransfer;
    };

    struct EdgeRange {
        EdgeRange()
            : begin(noEdge)
            , end(noEdge)
        {
        }

        Edge begin;
        Edge end;
    };

    struct EdgeLabel {
        EdgeLabel(const StopEventId stopEvent = noStopEvent, const TripId trip = noTripId,
            const StopEventId firstEvent = noStopEvent)
            : stopEvent(stopEvent)
            , trip(trip)
            , firstEvent(firstEvent)
        {
        }

        StopEventId stopEvent;
        TripId trip;
        StopEventId firstEvent;
    };

    struct RouteLabel {
        RouteLabel()
            : numberOfTrips(0)
        {
        }
        inline StopIndex end() const noexcept
        {
            return StopIndex(departureTimes.size() / numberOfTrips);
        }
        u_int32_t numberOfTrips;
        std::vector<int> departureTimes;
    };

public:
    TransferSearch(MLData& data)
        : data(data)
        , queue(data.numberOfStopEvents())
        , edgeRanges(data.numberOfStopEvents())
        , queueSize(0)
        , reachedIndex(data)
        , edgeLabels(data.stopEventGraph.numEdges())
        , routeLabels(data.numberOfRoutes())
        , localLevels(data.stopEventGraph.numEdges(), 0)
        , toBeUnpacked(data.numberOfStopEvents())
        , fromStopEventId(data.stopEventGraph.numEdges())
        , lastExtractedRun(data.numberOfStopEvents(), 0)
        , currentRun(0)
        , extractedPaths(0)
        , totalLengthPfExtractedPaths(0)
        , numAddedShortcuts(0)
    {
        for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
            edgeLabels[edge].stopEvent = StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
            edgeLabels[edge].trip = data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
            edgeLabels[edge].firstEvent = data.firstStopEventOfTrip[edgeLabels[edge].trip];

            fromStopEventId[edge] = StopEventId(from);
        }

        for (const RouteId route : data.raptorData.routes()) {
            const size_t numberOfStops = data.numberOfStopsInRoute(route);
            const size_t numberOfTrips = data.raptorData.numberOfTripsInRoute(route);
            const RAPTOR::StopEvent* stopEvents = data.raptorData.firstTripOfRoute(route);
            routeLabels[route].numberOfTrips = numberOfTrips;
            routeLabels[route].departureTimes.resize((numberOfStops - 1) * numberOfTrips);
            for (size_t trip = 0; trip < numberOfTrips; trip++) {
                for (size_t stopIndex = 0; stopIndex + 1 < numberOfStops; stopIndex++) {
                    routeLabels[route].departureTimes[(stopIndex * numberOfTrips) + trip] = stopEvents[(trip * numberOfStops) + stopIndex].departureTime;
                }
            }
        }
        profiler.registerPhases({ PHASE_SCAN_TRIPS });
        profiler.registerMetrics({ METRIC_ROUNDS, METRIC_SCANNED_TRIPS, METRIC_SCANNED_STOPS, METRIC_RELAXED_TRANSFERS,
            METRIC_ENQUEUES });
    }

    inline void run(const TripId trip, const StopIndex stopIndex, std::vector<int> currentLevels, std::vector<int> currentCellIds) noexcept
    {
        AssertMsg(data.isTrip(trip), "Trip is not valid!");
        AssertMsg(stopIndex < data.numberOfStopsInTrip(trip), "StopIndex is not valid!");

        profiler.start();
        clear();
        levels = currentLevels;
        cellIds = currentCellIds;
        minLevel = *std::min_element(levels.begin(), levels.end());

        enqueue(trip, stopIndex);
        scanTrips();
        unpack();
        profiler.done();
    }

    inline Profiler& getProfiler() noexcept
    {
        return profiler;
    }

    inline std::vector<uint8_t>& getLocalLevels() noexcept
    {
        return localLevels;
    }

private:
    inline void clear() noexcept
    {
        queueSize = 0;
        reachedIndex.clear();
        toBeUnpacked.clear();

        if (currentRun == 0) {
            lastExtractedRun.assign(data.numberOfStopEvents(), 0);
        }
        ++currentRun;
    }

    inline void scanTrips() noexcept
    {
        profiler.startPhase();
        u_int8_t currentRoundNumber = 0;
        size_t roundBegin = 0;
        size_t roundEnd = queueSize;
        while (roundBegin < roundEnd && currentRoundNumber < 15) {
            ++currentRoundNumber;
            profiler.countMetric(METRIC_ROUNDS);

            for (size_t i = roundBegin; i < roundEnd; i++) {
                const TripLabel& label = queue[i];
                profiler.countMetric(METRIC_SCANNED_TRIPS);
                for (StopEventId j = label.begin; j < label.end; j++) {
                    profiler.countMetric(METRIC_SCANNED_STOPS);
                    // check if stop of j is outside this cell => add to 'toBeUnpacked'
                    StopId currentStop = data.getStopOfStopEvent(j);
                    if (!isStopInCell(currentStop)) {
                        toBeUnpacked.insert(i, j);
                    }
                }
            }

            for (size_t i = roundBegin; i < roundEnd; i++) {
                TripLabel& label = queue[i];
                edgeRanges[i].begin = data.stopEventGraph.beginEdgeFrom(Vertex(label.begin));
                edgeRanges[i].end = data.stopEventGraph.beginEdgeFrom(Vertex(label.end));
            }

            for (size_t i = roundBegin; i < roundEnd; i++) {
                const EdgeRange& label = edgeRanges[i];

                for (Edge edge = label.begin; edge < label.end; edge++) {
                    profiler.countMetric(METRIC_RELAXED_TRANSFERS);
                    enqueue(edge, i);
                }
            }
            roundBegin = roundEnd;
            roundEnd = queueSize;
        }
        profiler.donePhase(PHASE_SCAN_TRIPS);
    }

    inline bool isStopInCell(StopId stop) const
    {
        AssertMsg(data.isStop(stop), "Stop is not a valid stop!");
        return data.stopInCell(stop, levels, cellIds);
    }

    inline void enqueue(const TripId trip, const StopIndex index) noexcept
    {
        profiler.countMetric(METRIC_ENQUEUES);
        if (reachedIndex.alreadyReached(trip, index))
            return;
        const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
        queue[queueSize] = TripLabel(StopEventId(firstEvent + index), StopEventId(firstEvent + reachedIndex(trip)));
        queueSize++;
        AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
        reachedIndex.update(trip, index);
    }

    inline void enqueue(const Edge edge, const size_t parent) noexcept
    {
        profiler.countMetric(METRIC_ENQUEUES);
        const EdgeLabel& label = edgeLabels[edge];

        // break if a) already reached OR b) the stop if this transfer is not in the same cell
        if (reachedIndex.alreadyReached(label.trip, label.stopEvent - label.firstEvent) || !isStopInCell(data.getStop(label.trip, StopIndex(label.stopEvent - label.firstEvent - 1))))
            return;

        if (minLevel > localLevels[edge])
            return;

        queue[queueSize] = TripLabel(
            label.stopEvent,
            StopEventId(label.firstEvent + reachedIndex(label.trip)),
            parent,
            edge);

        queueSize++;
        AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
        reachedIndex.update(label.trip, StopIndex(label.stopEvent - label.firstEvent));
    }

    // all marked events which we want to marks as local for the next level
    inline void unpack()
    {
        // we need to loop over the collected queue elements,
        // and for each element, we stored the latest ('das hinterste') event which we want to mark as local
        for (const size_t index : toBeUnpacked.getKeys()) {
            unpackStopEvent(index);

            // STATS
            ++extractedPaths;
        }
    }

    // unpacks a reached stop event
    inline void unpackStopEvent(size_t index)
    {
        AssertMsg(index < queueSize, "Index is out of bounds!");
        AssertMsg(toBeUnpacked[index] < data.numberOfStopEvents(), "StopEvent out of range!");

        // did we already extract it? => skip
        if (lastExtractedRun[toBeUnpacked[index]] == currentRun)
            return;

        TripLabel label = queue[index];
        AssertMsg(label.begin <= toBeUnpacked[index] && toBeUnpacked[index] < label.end, "Out of range!");

        // toBeUnpacked[index] *needs* to be between begin and end, hence the && j < label.end check
        for (StopEventId j(label.begin); j <= toBeUnpacked[index] && j < label.end; ++j) {
            data.getLocalLevelOfEvent(j) = minLevel + 1;

            if (lastExtractedRun[j] == currentRun)
                return;

            lastExtractedRun[j] = currentRun;
        }
        Edge currentEdge = label.parentTransfer;

        // STATS
        uint8_t numCurrentEdges(0);

        while (currentEdge != noEdge) {
            localLevels[currentEdge] = minLevel + 1;

            index = label.parent;
            label = queue[index];

            // we don't want to mark all events on the queue segment, but only those before the transfer
            // a queue segment looks like this:
            // [start event, (...), event from which the transfer led to the "unpacked event" (call it e), (...), the last event]
            // the currentEdge starts at e, but we want to mark all events from [start event, e]
            StopEventId e = fromStopEventId[currentEdge];

            AssertMsg(label.begin <= e && e < label.end, "FromVertex is out of bounds! FromStopEvent: " << e << ", but label: " << label.begin << " -> " << label.end);

            for (StopEventId j(label.begin); j <= e && j < label.end; ++j) {
                data.getLocalLevelOfEvent(j) = minLevel + 1;

                if (lastExtractedRun[j] == currentRun)
                    return;

                lastExtractedRun[j] = currentRun;
            }

            currentEdge = label.parentTransfer;

            // STATS
            ++totalLengthPfExtractedPaths;
            ++numCurrentEdges;
        }

        AssertMsg(index == 0, "Index is not 0!");

        // STATS
        // only add shortcut if you skip at least 2 transfers
        numAddedShortcuts += (numCurrentEdges > 1);
    }

public:
    inline double getAvgPathLengthPerLevel() noexcept
    {
        return (double)totalLengthPfExtractedPaths / (double)extractedPaths;
    }

    inline uint64_t getNumberOfAddedShortcuts() noexcept
    {
        return numAddedShortcuts;
    }

    inline void resetStats() noexcept
    {
        // STATS
        totalLengthPfExtractedPaths = 0;
        extractedPaths = 0;
        numAddedShortcuts = 0;
    }

private:
    MLData& data;

    std::vector<TripLabel> queue;
    std::vector<EdgeRange> edgeRanges;

    size_t queueSize;
    ReachedIndex reachedIndex;

    std::vector<EdgeLabel> edgeLabels;
    std::vector<RouteLabel> routeLabels;
    std::vector<uint8_t> localLevels;

    std::vector<int> levels;
    std::vector<int> cellIds;
    int minLevel;

    Profiler profiler;

    // [roundIndex] = biggest StopEvent that is outside of the cell
    IndexedMap<StopEventId, false, size_t> toBeUnpacked;

    // same as FromVertex, but for the stopEventGraph, it is not defined
    // we need to extract quickly the event from which the transfer was possible
    std::vector<StopEventId> fromStopEventId;

    // like a timestamp, used to check in which run the stop event has already been extracted
    std::vector<size_t> lastExtractedRun;
    size_t currentRun;

    // stats
    uint64_t extractedPaths;
    uint64_t totalLengthPfExtractedPaths;
    uint64_t numAddedShortcuts;
};

} // namespace TripBased
