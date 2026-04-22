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
#include <execution>

#include "../../../DataStructures/TripBased/Data.h"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/MultiThreading.h"

namespace TripBased {

// Encoding: bits [31..8] = TripId, bits [7..0] = StopIndex
// TripId <= 5,000,000 < 2^23, StopIndex fits in uint8_t -> safe in uint32_t
inline constexpr std::uint32_t encodeStopEvent(const TripId trip, const StopIndex index) noexcept {
    return (static_cast<std::uint32_t>(trip) << 8) | static_cast<std::uint32_t>(static_cast<std::uint8_t>(index));
}
inline constexpr TripId decodeTrip(const std::uint32_t encoded) noexcept { return TripId(encoded >> 8); }
inline constexpr StopIndex decodeStopIndex(const std::uint32_t encoded) noexcept { return StopIndex(encoded & 0xFFu); }

class StopEventGraphBuilder {
    struct StopLabel {
    public:
        StopLabel() : arrivalTime(INFTY), timestamp(0) {}

        inline bool update(const int newTimestamp, const int newArrivalTime) noexcept {
            if (timestamp != newTimestamp) {
                timestamp = newTimestamp;
                arrivalTime = newArrivalTime;
                return true;
            }
            if (arrivalTime > newArrivalTime) {
                arrivalTime = newArrivalTime;
                return true;
            }
            return false;
        }

        int arrivalTime;
        int timestamp;
    };

    struct RouteTransfer {
        RouteTransfer(const RouteId toRoute, const StopIndex fromIndex, const StopIndex toIndex, const int transferTime)
            : toRoute(toRoute), fromIndex(fromIndex), toIndex(toIndex), transferTime(transferTime) {}

        RouteId toRoute;
        StopIndex fromIndex;
        StopIndex toIndex;
        int transferTime;

        inline std::tuple<RouteId, StopIndex, StopIndex> getTuple() const noexcept {
            return std::make_tuple(toRoute, -fromIndex, toIndex);
        }

        inline bool operator<(const RouteTransfer& other) const noexcept { return getTuple() < other.getTuple(); }
    };

    // Adjacency list indexed by StopEventId; each entry is an encoded (TripId, StopIndex) pair.
    using DynamicGraph = std::vector<std::vector<std::uint32_t>>;

public:
    StopEventGraphBuilder(const TripBased::Data& data)
        : data(data),
          labels(data.numberOfStops()),
          timestamp(0),
          generatedTransfers(data.numberOfStopEvents()),
          keptTransfers(data.numberOfStopEvents()) {}

public:
    inline void generateRouteBasedTransfers(const RouteId fromRoute, const int timeWindowBegin,
                                            const int timeWindowEnd) noexcept {
        if (numGeneratedEdges > 1000000) {
            for (auto& v : generatedTransfers) v.clear();
            numGeneratedEdges = 0;
        }

        const std::vector<RouteTransfer> routeTransfers = generateRouteTransfers(fromRoute);

        const Range<TripId> allTrips = data.tripsOfRoute(fromRoute);
        if (allTrips.empty()) return;

        const StopIndex firstStop(0);
        const StopIndex lastStop(data.numberOfStopsInRoute(fromRoute) - 1);
        const TripId firstTrip = data.getEarliestTrip(fromRoute, firstStop, timeWindowBegin);
        const TripId lastTrip = data.getLatestTrip(fromRoute, lastStop, timeWindowEnd);

        if (firstTrip == noTripId || lastTrip == noTripId) return;
        if (firstTrip > lastTrip) return;

        for (TripId fromTrip(firstTrip); fromTrip <= lastTrip; ++fromTrip) {
            RouteId toRoute = noRouteId;
            std::vector<TripId> earliestTrip;
            for (const RouteTransfer& routeTransfer : routeTransfers) {
                if (routeTransfer.toRoute != toRoute) {
                    toRoute = routeTransfer.toRoute;
                    std::vector<TripId>(data.numberOfStopsInRoute(toRoute), noTripId).swap(earliestTrip);
                }
                const StopEventId fromEvent = data.getStopEventId(fromTrip, routeTransfer.fromIndex);
                const int arrivalTime = data.raptorData.stopEvents[fromEvent].arrivalTime + routeTransfer.transferTime;
                const TripId toTrip = data.getEarliestTrip(toRoute, routeTransfer.toIndex, arrivalTime);
                if (toTrip >= earliestTrip[routeTransfer.toIndex]) continue;
                if ((toRoute == fromRoute) && (toTrip >= fromTrip) &&
                    (routeTransfer.toIndex >= routeTransfer.fromIndex))
                    continue;
                if (isUTurn(fromTrip, routeTransfer.fromIndex, toTrip, routeTransfer.toIndex)) continue;
                for (StopIndex i = routeTransfer.toIndex; i < data.numberOfStopsInRoute(toRoute); i++) {
                    earliestTrip[i] = std::min(earliestTrip[i], toTrip);
                }
                generatedTransfers[fromEvent].push_back(encodeStopEvent(toTrip, routeTransfer.toIndex));
                ++numGeneratedEdges;
            }
        }
    }

    inline void generateRouteBasedTransfers(const RouteId fromRoute) noexcept {
        if (numGeneratedEdges > 1000000) {
            for (auto& v : generatedTransfers) v.clear();
            numGeneratedEdges = 0;
        }
        const std::vector<RouteTransfer> routeTransfers = generateRouteTransfers(fromRoute);
        for (const TripId fromTrip : data.tripsOfRoute(fromRoute)) {
            RouteId toRoute = noRouteId;
            std::vector<TripId> earliestTrip;
            for (const RouteTransfer& routeTransfer : routeTransfers) {
                if (routeTransfer.toRoute != toRoute) {
                    toRoute = routeTransfer.toRoute;
                    std::vector<TripId>(data.numberOfStopsInRoute(toRoute), noTripId).swap(earliestTrip);
                }
                const StopEventId fromEvent = data.getStopEventId(fromTrip, routeTransfer.fromIndex);
                const int arrivalTime = data.raptorData.stopEvents[fromEvent].arrivalTime + routeTransfer.transferTime;
                const TripId toTrip = data.getEarliestTrip(toRoute, routeTransfer.toIndex, arrivalTime);
                if (toTrip >= earliestTrip[routeTransfer.toIndex]) continue;
                if ((toRoute == fromRoute) && (toTrip >= fromTrip) &&
                    (routeTransfer.toIndex >= routeTransfer.fromIndex))
                    continue;
                if (isUTurn(fromTrip, routeTransfer.fromIndex, toTrip, routeTransfer.toIndex)) continue;
                for (StopIndex i = routeTransfer.toIndex; i < data.numberOfStopsInRoute(toRoute); i++) {
                    earliestTrip[i] = std::min(earliestTrip[i], toTrip);
                }
                generatedTransfers[fromEvent].push_back(encodeStopEvent(toTrip, routeTransfer.toIndex));
                ++numGeneratedEdges;
            }
        }
    }

    inline void generateFullTransfers(const TripId trip) noexcept {
        if (numGeneratedEdges > 1000000) {
            for (auto& v : generatedTransfers) v.clear();
            numGeneratedEdges = 0;
        }
        const StopId* stops = data.stopArrayOfTrip(trip);
        for (StopIndex i = StopIndex(1); i < data.numberOfStopsInTrip(trip); i++) {
            const StopId stop = stops[i];
            const int arrivalTime = data.getStopEvent(trip, i).arrivalTime;
            findTransfers(trip, i, stop, arrivalTime);
            for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stop)) {
                const StopId toStop = StopId(data.raptorData.transferGraph.get(ToVertex, edge));
                const int transferTime = data.raptorData.transferGraph.get(TravelTime, edge);
                findTransfers(trip, i, toStop, arrivalTime + transferTime);
            }
        }
    }

    inline void reduceTransfers(const TripId trip) noexcept {
        timestamp++;
        const StopId* stops = data.stopArrayOfTrip(trip);
        for (StopIndex i = StopIndex(data.numberOfStopsInTrip(trip) - 1); i > 0; i--) {
            const int arrivalTime = data.getStopEvent(trip, i).arrivalTime;
            labels[stops[i]].update(timestamp, arrivalTime);
            for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stops[i])) {
                const StopId toStop = StopId(data.raptorData.transferGraph.get(ToVertex, edge));
                const int transferTime = data.raptorData.transferGraph.get(TravelTime, edge);
                labels[toStop].update(timestamp, arrivalTime + transferTime);
            }

            // Collect encoded (toTrip, toIndex) entries for this stop event
            const StopEventId fromEvent = data.getStopEventId(trip, i);
            std::vector<std::uint32_t>& outEdges = generatedTransfers[fromEvent];

            // Sort by arrival time of the target stop event
            std::stable_sort(outEdges.begin(), outEdges.end(), [&](const std::uint32_t a, const std::uint32_t b) {
                const TripId tripA = decodeTrip(a);
                const StopIndex idxA = decodeStopIndex(a);
                const TripId tripB = decodeTrip(b);
                const StopIndex idxB = decodeStopIndex(b);
                return data.getStopEvent(tripA, idxA).arrivalTime < data.getStopEvent(tripB, idxB).arrivalTime;
            });

            for (const std::uint32_t encoded : outEdges) {
                const TripId toTrip = decodeTrip(encoded);
                const StopIndex toIndex = decodeStopIndex(encoded);
                bool keep = false;
                const StopId* toStops = data.stopArrayOfTrip(toTrip) + toIndex;
                const size_t remainingStops = data.numberOfStopsInTrip(toTrip) - toIndex - 1;
                // We need the base StopEventId to walk forward; derive it once
                const StopEventId toEvent = data.getStopEventId(toTrip, toIndex);
                for (size_t j = remainingStops; j > 0; j--) {
                    const StopId destinationStop = toStops[j];
                    const int destinationArrivalTime = data.raptorData.stopEvents[toEvent + j].arrivalTime;

                    keep |= labels[destinationStop].update(timestamp, destinationArrivalTime);
                    for (const Edge edge : data.raptorData.transferGraph.edgesFrom(destinationStop)) {
                        const StopId arrivalStop = StopId(data.raptorData.transferGraph.get(ToVertex, edge));
                        const int arrivalTime =
                            destinationArrivalTime + data.raptorData.transferGraph.get(TravelTime, edge);
                        keep |= labels[arrivalStop].update(timestamp, arrivalTime);
                    }
                }
                if (keep) {
                    keptTransfers[fromEvent].push_back(encoded);
                    ++numKeptEdges;
                }
            }

            // Kept edges were appended in ascending arrival-time order; reverse to match original
            // descending-edge-id stable_sort (largest edge id first) — preserve insertion order instead.
            // Original code reversed by sorting edges descending; here we simply leave them as-is
            // since consumers iterate over the adjacency list rather than relying on edge IDs.
        }
    }

    inline void reduceTransfers(const RouteId route) noexcept {
        for (const TripId trip : data.tripsOfRoute(route)) {
            reduceTransfers(trip);
        }
    }

    // Build a SimpleEdgeList (or equivalent) from keptTransfers for merging into data.stopEventGraph.
    // Returns the kept adjacency list directly for callers that need it.
    inline const DynamicGraph& getStopEventGraph() const noexcept { return keptTransfers; }
    inline DynamicGraph& getStopEventGraph() noexcept { return keptTransfers; }
    inline const DynamicGraph& getGeneratedStopEventGraph() const noexcept { return generatedTransfers; }
    inline DynamicGraph& getGeneratedStopEventGraph() noexcept { return generatedTransfers; }
    inline size_t numKeptEdgesTotal() const noexcept { return numKeptEdges; }

    inline std::vector<RouteTransfer> generateRouteTransfers(const RouteId fromRoute) const noexcept {
        std::vector<RouteTransfer> routeTransfers;
        const StopId* stops = data.raptorData.stopArrayOfRoute(fromRoute);
        for (StopIndex i(data.numberOfStopsInRoute(fromRoute) - 1); i > 0; i--) {
            const StopId fromStop = stops[i];
            for (const RAPTOR::RouteSegment& toSegment : data.raptorData.routesContainingStop(fromStop)) {
                if (toSegment.routeId == fromRoute && toSegment.stopIndex == i) continue;
                routeTransfers.emplace_back(toSegment.routeId, i, toSegment.stopIndex, 0);
            }
            for (const Edge edge : data.raptorData.transferGraph.edgesFrom(fromStop)) {
                const StopId toStop = StopId(data.raptorData.transferGraph.get(ToVertex, edge));
                const int transferTime = data.raptorData.transferGraph.get(TravelTime, edge);
                for (const RAPTOR::RouteSegment& toRouteSegment : data.raptorData.routesContainingStop(toStop)) {
                    routeTransfers.emplace_back(toRouteSegment.routeId, i, toRouteSegment.stopIndex, transferTime);
                }
            }
        }
        std::sort(routeTransfers.begin(), routeTransfers.end());
        return routeTransfers;
    }

    inline void findTransfers(const TripId fromTrip, const StopIndex fromIndex, const StopId toStop,
                              const int toArrivalTime) noexcept {
        const RouteId fromRoute = data.routeOfTrip[fromTrip];
        for (const RAPTOR::RouteSegment& toSegment : data.raptorData.routesContainingStop(toStop)) {
            const TripId toTrip = data.getEarliestTrip(toSegment, toArrivalTime);
            if (toTrip == noTripId) continue;
            if ((toSegment.routeId == fromRoute) && (toTrip >= fromTrip) && (toSegment.stopIndex >= fromIndex))
                continue;
            if (isUTurn(fromTrip, fromIndex, toTrip, toSegment.stopIndex)) continue;
            const StopEventId fromEvent = data.getStopEventId(fromTrip, fromIndex);
            generatedTransfers[fromEvent].push_back(encodeStopEvent(toTrip, toSegment.stopIndex));
            ++numGeneratedEdges;
        }
    }

    inline bool isUTurn(const TripId fromTrip, const StopIndex fromIndex, const TripId toTrip,
                        const StopIndex toIndex) const noexcept {
        if (fromIndex < 2) return false;
        if (toIndex + 1 >= data.numberOfStopsInTrip(toTrip)) return false;
        if (data.getStop(fromTrip, StopIndex(fromIndex - 1)) != data.getStop(toTrip, StopIndex(toIndex + 1)))
            return false;
        if (data.getStopEvent(fromTrip, StopIndex(fromIndex - 1)).arrivalTime >
            data.getStopEvent(toTrip, StopIndex(toIndex + 1)).departureTime)
            return false;
        return true;
    }

    const TripBased::Data& data;
    std::vector<StopLabel> labels;
    int timestamp;
    size_t numGeneratedEdges{0};
    size_t numKeptEdges{0};
    DynamicGraph generatedTransfers;  // [StopEventId] -> list of encoded (TripId<<8|StopIndex)
    DynamicGraph keptTransfers;       // [StopEventId] -> list of encoded (TripId<<8|StopIndex)
};

inline void mergeAndBuildStopEventGraph(TripBased::Data& data, std::vector<TripBased::StopEventGraphBuilder>& builders,
                                        TransferGraphWithLocalLevel& stopEventGraph) {
    const size_t numVertices = stopEventGraph.numVertices();
    const size_t numBuilders = builders.size();

    std::vector<Edge> degree(numVertices, Edge(0));
    std::vector<size_t> vertexIdx(numVertices);
    std::iota(vertexIdx.begin(), vertexIdx.end(), size_t(0));

#pragma omp parallel for
    for (std::size_t b = 0; b < numBuilders; ++b) {
        const auto& bob = builders[b];
        for (std::size_t v = 0; v < numVertices; ++v) {
            assert(v < bob.getStopEventGraph().size());
            if (bob.getStopEventGraph()[v].size() == 0) {
                continue;
            }

            assert(v < degree.size());
            degree[v] = Edge(bob.getStopEventGraph()[v].size());
        }
    }

    std::vector<Edge> beginOut(numVertices + 1);
    beginOut[0] = Edge(0);

    std::exclusive_scan(std::execution::par, degree.begin(), degree.end(), beginOut.begin() + 1, Edge(0));

    const size_t numEdges = static_cast<size_t>(beginOut[numVertices]);

    std::vector<Vertex> toVertexVec(numEdges);
    std::vector<int> travelTimeVec(numEdges, 0);           // zero-initialised
    std::vector<std::uint8_t> localLevelVec(numEdges, 0);  // zero-initialised

#pragma omp parallel for
    for (std::size_t b = 0; b < numBuilders; ++b) {
        const auto& bob = builders[b];
        for (std::size_t v = 0; v < numVertices; ++v) {
            assert(v < bob.getStopEventGraph().size());
            if (bob.getStopEventGraph()[v].size() == 0) {
                continue;
            }

            auto pos = beginOut[v];

            for (auto encoded : bob.getStopEventGraph()[v]) {
                const TripId trip = decodeTrip(encoded);
                const StopIndex stopIndex = decodeStopIndex(encoded);
                Vertex toEvent = Vertex(data.getStopEventId(trip, stopIndex));
                assert(toEvent < numVertices);
                AssertMsg(pos < toVertexVec.size(),
                          "Position " << (int)pos << " is out of bounds! Size: " << toVertexVec.size());
                toVertexVec[pos++] = toEvent;
            }
        }
    }

#pragma omp parallel for
    for (size_t v = 0; v < numVertices; ++v) {
        const size_t begin = beginOut[v];
        const size_t end = beginOut[v + 1];

        if (end - begin <= 1) continue;

        std::sort(toVertexVec.begin() + begin, toVertexVec.begin() + end);
    }

    stopEventGraph.getBeginOut().swap(beginOut);
    stopEventGraph.get(ToVertex).swap(toVertexVec);
    stopEventGraph.get(TravelTime).swap(travelTimeVec);
    stopEventGraph.get(LocalLevel).swap(localLevelVec);

    AssertMsg(stopEventGraph.satisfiesInvariants(), "Invariants not satisfied after parallel merge!");
}

static void flushToEdgeList(const std::vector<std::vector<std::uint32_t>>& dg, SimpleEdgeList& out,
                            const TripBased::Data& data) {
    for (size_t from = 0; from < dg.size(); from++) {
        for (const std::uint32_t encoded : dg[from]) {
            const TripId toTrip = decodeTrip(encoded);
            const StopIndex toIndex = decodeStopIndex(encoded);
            out.addEdge(Vertex(from), Vertex(data.getStopEventId(toTrip, toIndex)));
        }
    }
}

inline void ComputeStopEventGraph(TripBased::Data& data) noexcept {
    Progress progress(data.numberOfTrips());
    StopEventGraphBuilder builder(data);
    for (const TripId trip : data.trips()) {
        builder.generateFullTransfers(trip);
        builder.reduceTransfers(trip);
        progress++;
    }
    SimpleEdgeList stopEventGraph;
    stopEventGraph.addVertices(data.numberOfStopEvents());
    flushToEdgeList(builder.getStopEventGraph(), stopEventGraph, data);
    Graph::move(std::move(stopEventGraph), data.stopEventGraph);
    data.stopEventGraph.sortEdges(ToVertex);
    progress.finished();
}

inline void ComputeStopEventGraph(TripBased::Data& data, const int numberOfThreads,
                                  const int pinMultiplier = 1) noexcept {
    Progress progress(data.numberOfTrips());
    SimpleEdgeList stopEventGraph;
    stopEventGraph.addVertices(data.numberOfStopEvents());

    const int numCores = numberOfCores();

    omp_set_num_threads(numberOfThreads);
#pragma omp parallel
    {
        int threadId = omp_get_thread_num();
        pinThreadToCoreId((threadId * pinMultiplier) % numCores);
        AssertMsg(omp_get_num_threads() == numberOfThreads,
                  "Number of threads is " << omp_get_num_threads() << ", but should be " << numberOfThreads << "!");

        StopEventGraphBuilder builder(data);
        const size_t numberOfTrips = data.numberOfTrips();

#pragma omp for schedule(dynamic, 1)
        for (size_t i = 0; i < numberOfTrips; i++) {
            const TripId trip = TripId(i);
            builder.generateFullTransfers(trip);
            builder.reduceTransfers(trip);
            progress++;
        }

#pragma omp critical
        {
            stopEventGraph.reserve(stopEventGraph.numVertices(),
                                   stopEventGraph.numEdges() + builder.numKeptEdgesTotal());
            flushToEdgeList(builder.getStopEventGraph(), stopEventGraph, data);
        }
    }

    Graph::move(std::move(stopEventGraph), data.stopEventGraph);
    data.stopEventGraph.sortEdges(ToVertex);
    progress.finished();
}

inline void ComputeStopEventGraphRouteBased(TripBased::Data& data) noexcept {
    Progress progress(data.numberOfRoutes());
    StopEventGraphBuilder builder(data);
    for (const RouteId route : data.routes()) {
        builder.generateRouteBasedTransfers(route);
        builder.reduceTransfers(route);
        progress++;
    }
    SimpleEdgeList stopEventGraph;
    stopEventGraph.addVertices(data.numberOfStopEvents());
    flushToEdgeList(builder.getStopEventGraph(), stopEventGraph, data);
    Graph::move(std::move(stopEventGraph), data.stopEventGraph);
    data.stopEventGraph.sortEdges(ToVertex);
    progress.finished();
}

inline void ComputeStopEventGraphRouteBased(TripBased::Data& data, const int numberOfThreads,
                                            const int pinMultiplier = 1) noexcept {
    const int numCores = numberOfCores();
    const size_t numberOfRoutes = data.numberOfRoutes();

    std::vector<StopEventGraphBuilder> builders(numberOfThreads, StopEventGraphBuilder(data));

    Progress progress(numberOfRoutes);

#pragma omp parallel num_threads(numberOfThreads)
    {
        const int threadId = omp_get_thread_num();
        pinThreadToCoreId((threadId * pinMultiplier) % numCores);
        auto& builder = builders[threadId];

#pragma omp for schedule(dynamic, 1)
        for (size_t i = 0; i < numberOfRoutes; i++) {
            builder.generateRouteBasedTransfers(RouteId(i));
            builder.reduceTransfers(RouteId(i));
            progress++;
        }
    }

    TransferGraphWithLocalLevel newGraph;
    newGraph.addVertices(data.numberOfStopEvents());

    mergeAndBuildStopEventGraph(data, builders, newGraph);

    Graph::move(std::move(newGraph), data.stopEventGraph);
    progress.finished();
}

inline void ComputeStopEventGraphRouteBasedTimeWindow(TripBased::Data& data, const int timeWindowBegin,
                                                      const int timeWindowEnd, const int numberOfThreads,
                                                      const int pinMultiplier = 1) noexcept {
    const int numCores = numberOfCores();
    const size_t numberOfRoutes = data.numberOfRoutes();

    std::vector<StopEventGraphBuilder> builders(numberOfThreads, StopEventGraphBuilder(data));

    Progress progress(numberOfRoutes);

#pragma omp parallel num_threads(numberOfThreads)
    {
        const int threadId = omp_get_thread_num();
        pinThreadToCoreId((threadId * pinMultiplier) % numCores);
        auto& builder = builders[threadId];

#pragma omp for schedule(dynamic, 1) nowait
        for (size_t i = 0; i < numberOfRoutes; i++) {
            const RouteId route = RouteId(i);
            builder.generateRouteBasedTransfers(route, timeWindowBegin, timeWindowEnd);
            builder.reduceTransfers(route);
            progress++;
        }
    }

    TransferGraphWithLocalLevel newGraph;
    newGraph.addVertices(data.numberOfStopEvents());

    mergeAndBuildStopEventGraph(data, builders, newGraph);
    newGraph.sortEdges(ToVertex);
    // we dont want to move, as it is partial
    /* data.stopEventGraph = std::move(newGraph); */

    progress.finished();
}

}  // namespace TripBased
