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

#include "../../../DataStructures/TripBased/Data.h"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/MultiThreading.h"

namespace TripBased {

class StopEventGraphBuilderEdgeList {
private:
    struct StopLabel {
    public:
        StopLabel() : arrivalTime(INFTY), timestamp(0) {}

        inline void checkTimestamp(const int newTimestamp) noexcept {
            arrivalTime = (timestamp != newTimestamp) ? INFTY : arrivalTime;
            timestamp = newTimestamp;
        }

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

public:
    StopEventGraphBuilderEdgeList(const TripBased::Data& data)
        : data(data), labels(data.numberOfStops()), timestamp(0) {
        keptTransfers.reserve(data.numberOfStopEvents(), 3 * data.numberOfStopEvents());
        keptTransfers.addVertices(data.numberOfStopEvents());

        size_t maxTripLen = 0;
        for (const TripId trip : data.trips()) {
            maxTripLen = std::max(maxTripLen, (size_t)data.numberOfStopsInTrip(trip));
        }
        edgesFrom.resize(maxTripLen);
    }

public:
    inline void generateFullTransfers(const TripId trip) noexcept {
        generatedTransfers.clear();

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

    // call this directly after generate full transfers
    inline void reduceTransfers(const TripId trip) noexcept {
        timestamp++;
        const StopId* stops = data.stopArrayOfTrip(trip);
        const std::size_t tripLen = data.numberOfStopsInTrip(trip);

        for (const auto& [fromIndex, to] : generatedTransfers) {
            edgesFrom[fromIndex].push_back(to);
        }
        generatedTransfers.clear();

        for (StopIndex i = StopIndex(tripLen - 1); i > 0; i--) {
            const int arrivalTime = data.getStopEvent(trip, i).arrivalTime;
            labels[stops[i]].update(timestamp, arrivalTime);
            for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stops[i])) {
                const StopId toStop = StopId(data.raptorData.transferGraph.get(ToVertex, edge));
                const int transferTime = data.raptorData.transferGraph.get(TravelTime, edge);
                labels[toStop].update(timestamp, arrivalTime + transferTime);
            }

            auto& candidates = edgesFrom[i];

            std::stable_sort(candidates.begin(), candidates.end(), [&](const StopEventId a, const StopEventId b) {
                return data.raptorData.stopEvents[a].arrivalTime < data.raptorData.stopEvents[b].arrivalTime;
            });

            const StopEventId fromEvent = data.getStopEventId(trip, i);
            for (const StopEventId toEvent : candidates) {
                bool keep = false;
                const StopIndex toIndex = data.indexOfStopEvent[toEvent];
                const TripId toTrip = data.tripOfStopEvent[toEvent];
                const StopId* toStops = data.stopArrayOfTrip(toTrip) + toIndex;
                for (size_t j = data.numberOfStopsInTrip(toTrip) - toIndex - 1; j > 0; j--) {
                    const StopId destinationStop = toStops[j];
                    const int destinationArrivalTime = data.raptorData.stopEvents[toEvent + j].arrivalTime;
                    keep |= labels[destinationStop].update(timestamp, destinationArrivalTime);

                    for (const Edge edge : data.raptorData.transferGraph.edgesFrom(destinationStop)) {
                        const StopId arrivalStop = StopId(data.raptorData.transferGraph.get(ToVertex, edge));
                        const int arrTime =
                            destinationArrivalTime + data.raptorData.transferGraph.get(TravelTime, edge);

                        keep |= labels[arrivalStop].update(timestamp, arrTime);
                    }
                }
                if (keep) {
                    keptTransfers.addEdge(Vertex(fromEvent), Vertex(toEvent));
                }
            }

            edgesFrom[i].clear();
        }
    }

    inline SimpleDynamicGraph& getStopEventGraph() noexcept { return keptTransfers; }
    inline const SimpleDynamicGraph& getStopEventGraph() const noexcept { return keptTransfers; }

private:
    inline void findTransfers(const TripId fromTrip, const StopIndex fromIndex, const StopId toStop,
                              const int toArrivalTime) noexcept {
        const RouteId fromRoute = data.routeOfTrip[fromTrip];
        for (const RAPTOR::RouteSegment& toSegment : data.raptorData.routesContainingStop(toStop)) {
            const TripId toTrip = data.getEarliestTrip(toSegment, toArrivalTime);
            if (toTrip == noTripId) continue;
            if ((toSegment.routeId == fromRoute) && (toTrip >= fromTrip) && (toSegment.stopIndex >= fromIndex))
                continue;
            if (isUTurn(fromTrip, fromIndex, toTrip, toSegment.stopIndex)) continue;
            generatedTransfers.emplace_back(fromIndex, data.getStopEventId(toTrip, toSegment.stopIndex));
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

private:
    const TripBased::Data& data;

    std::vector<std::pair<StopIndex, StopEventId>> generatedTransfers;
    std::vector<std::vector<StopEventId>> edgesFrom;
    SimpleDynamicGraph keptTransfers;

    std::vector<StopLabel> labels;
    int timestamp;
};
}  // namespace TripBased
