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

#include "../../../Helpers/Types.h"

namespace TripBased {
/*
struct EdgeLabel {
EdgeLabel(const StopEventId stopEvent = noStopEvent,
        const TripId trip = noTripId,
        const StopEventId firstEvent = noStopEvent)
  : stopEvent(stopEvent), trip(trip), firstEvent(firstEvent) {}

StopEventId stopEvent;
TripId trip;
StopEventId firstEvent;

StopEventId getStopEvent() const { return stopEvent; }
TripId getTrip() const { return trip; }
StopEventId getFirstEvent() const { return firstEvent; }

void setStopEvent(StopEventId id) { stopEvent = id; }
void setTrip(TripId id) { trip = id; }
void setFirstEvent(StopEventId id) { firstEvent = id; }
};
*/

struct EdgeLabel {
    uint64_t data;

    EdgeLabel(StopIndex stopIndex = noStopIndex, TripId trip = noTripId, StopEventId firstEvent = noStopEvent)
        : data(0) {
        setTrip(trip);
        setFirstEvent(firstEvent);
        setStopIndex(stopIndex);
    }

    void init(const StopEventId event, const TripId trip, const StopEventId firstEvent) {
        setTrip(trip);
        setFirstEvent(firstEvent);
        setStopIndex(StopIndex(event - firstEvent + 1));
    }

    StopIndex getStopIndex() const { return static_cast<StopIndex>(data & 0xFFULL); }
    void setStopIndex(StopIndex d) { data = (data & ~0xFFULL) | d; }

    TripId getTrip() const { return static_cast<TripId>((data >> 8) & 0x7FFFFFULL); }

    void setTrip(TripId id) {
        assert(id < (1u << 23) || id == noTripId);
        data = (data & ~(0x7FFFFFULL << 8)) | (static_cast<uint64_t>(id & 0x7FFFFF) << 8);
    }

    StopEventId getFirstEvent() const { return static_cast<StopEventId>((data >> 31) & 0x7FFFFFFULL); }

    void setFirstEvent(StopEventId id) {
        assert(id < (1u << 27) || id == noStopEvent);
        data = (data & ~(0x7FFFFFFULL << 31)) | (static_cast<uint64_t>(id & 0x7FFFFFFULL) << 31);
    }

    StopEventId getStopEvent() const { return StopEventId(getFirstEvent() + getStopIndex()); }

    uint8_t getRank() const { return static_cast<uint8_t>((data >> 58) & 0x1FULL); }

    void setRank(uint8_t value) {
        assert(value <= 16);
        data = (data & ~(0x1FULL << 58)) | (static_cast<uint64_t>(value & 0x1F) << 58);
    }
};

static_assert(sizeof(EdgeLabel) == 8, "EdgeLabel must be 8 bytes");
static_assert(alignof(EdgeLabel) == alignof(uint64_t), "Unexpected alignment");

struct EdgeLabelCellId {
    uint64_t data;
    uint16_t cellId;

    EdgeLabelCellId(StopIndex stopIndex = noStopIndex, TripId trip = noTripId, StopEventId firstEvent = noStopEvent,
                    uint16_t cell = 0)
        : data(0), cellId(cell) {
        setTrip(trip);
        setFirstEvent(firstEvent);
        setStopIndex(stopIndex);
    }

    StopIndex getStopIndex() const { return static_cast<StopIndex>(data & 0xFFULL); }
    void setStopIndex(StopIndex d) { data = (data & ~0xFFULL) | d; }

    TripId getTrip() const { return static_cast<TripId>((data >> 8) & 0x7FFFFFULL); }
    void setTrip(TripId id) {
        assert(id < (1u << 23) || id == noTripId);
        data = (data & ~(0x7FFFFFULL << 8)) | (static_cast<uint64_t>(id & 0x7FFFFF) << 8);
    }

    StopEventId getFirstEvent() const { return static_cast<StopEventId>((data >> 31) & 0x7FFFFFFULL); }
    void setFirstEvent(StopEventId id) {
        assert(id < (1u << 27) || id == noStopEvent);
        data = (data & ~(0x7FFFFFFULL << 31)) | (static_cast<uint64_t>(id & 0x7FFFFFFULL) << 31);
    }

    StopEventId getStopEvent() const { return StopEventId(getFirstEvent() + getStopIndex()); }

    uint8_t getRank() const { return static_cast<uint8_t>((data >> 58) & 0x1FULL); }

    void setRank(uint8_t value) {
        assert(value <= 16);
        data = (data & ~(0x1FULL << 58)) | (static_cast<uint64_t>(value & 0x1F) << 58);
    }

    uint16_t getCellId() const { return cellId; }
    void setCellId(uint16_t id) { cellId = id; }
};

struct RouteLabel {
    RouteLabel() : numberOfTrips(0) {}

    inline StopIndex end() const noexcept {
        return StopIndex(departureTimes.size() / numberOfTrips);
    }

    inline StopIndex getStopIndex(const size_t index) const noexcept {
        return StopIndex(static_cast<int>(index) / end());
    }

    inline size_t getTripOffset(const size_t index) const noexcept {
        return index % end();
    }

    u_int32_t numberOfTrips;
    std::vector<int> departureTimes;
};

struct EventLookup {
    StopId stop;
    uint32_t arrTime;

    EventLookup(const StopId stop = noStop, uint32_t arrTime = 0) : stop(stop), arrTime(arrTime) {}
};

class QueryData {
public:
    QueryData(const Data& data)
        : transferGraph(data.raptorData.transferGraph),
          reverseTransferGraph(data.raptorData.transferGraph),
          eventLookup(data.numberOfStopEvents()),
          eventArrTimes(data.numberOfStopEvents()),
          eventDepTimes(data.numberOfStopEvents()),
          tripOfStopEvent(data.tripOfStopEvent),
          routeOfTrip(data.routeOfTrip),
          firstStopEventOfTrip(data.firstStopEventOfTrip),
          firstTripOfRoute(data.firstTripOfRoute),
          firstRouteSegmentOfStop(data.raptorData.firstRouteSegmentOfStop),
          routeSegments(data.raptorData.routeSegments),
          firstStopIdOfRoute(data.firstStopIdOfTrip),
          routeStopSequences(data.raptorData.stopIds),
          routeLabels(data.numberOfRoutes()) {
        reverseTransferGraph.revert();

#pragma omp parallel for
        for (size_t event = 0; event < data.numberOfStopEvents(); ++event) {
            eventLookup[event] = EventLookup(data.arrivalEvents[event].stop, data.arrivalEvents[event].arrivalTime);
            eventArrTimes[event] = data.arrivalEvents[event].arrivalTime;
            eventDepTimes[event] = data.raptorData.stopEvents[event].departureTime;
        }

        for (const RouteId route : data.raptorData.routes()) {
            const size_t numberOfStops = data.numberOfStopsInRoute(route);
            const size_t numberOfTrips = data.raptorData.numberOfTripsInRoute(route);
            const RAPTOR::StopEvent* stopEvents = data.raptorData.firstTripOfRoute(route);
            routeLabels[route].numberOfTrips = numberOfTrips;
            routeLabels[route].departureTimes.resize((numberOfStops - 1) * numberOfTrips);
            for (size_t trip = 0; trip < numberOfTrips; trip++) {
                for (size_t stopIndex = 0; stopIndex + 1 < numberOfStops; stopIndex++) {
                    routeLabels[route].departureTimes[(stopIndex * numberOfTrips) + trip] =
                        stopEvents[(trip * numberOfStops) + stopIndex].departureTime;
                }
            }
        }
    }

    inline RouteId getRouteOfStopEvent(const StopEventId stopEvent) const noexcept {
        return routeOfTrip[tripOfStopEvent[stopEvent]];
    }

    inline SubRange<std::vector<RAPTOR::RouteSegment>> routesContainingStop(const StopId stop) const noexcept {
        return SubRange<std::vector<RAPTOR::RouteSegment>>(routeSegments, firstRouteSegmentOfStop, stop);
    }

    inline const StopId* stopArrayOfRoute(const RouteId route) const noexcept {
        return &(routeStopSequences[firstStopIdOfRoute[route]]);
    }

public:
    TransferGraph transferGraph;
    TransferGraph reverseTransferGraph;

    std::vector<EventLookup> eventLookup;  // Stop and arrival time
    std::vector<std::uint32_t> eventArrTimes;
    std::vector<std::uint32_t> eventDepTimes;

    std::vector<TripId> tripOfStopEvent;
    std::vector<RouteId> routeOfTrip;
    std::vector<StopEventId> firstStopEventOfTrip;
    std::vector<TripId> firstTripOfRoute;

    std::vector<size_t> firstRouteSegmentOfStop;
    std::vector<RAPTOR::RouteSegment> routeSegments;
    std::vector<size_t> firstStopIdOfRoute;
    std::vector<StopId> routeStopSequences;
    // Departure times of events, sorted by (stopIndex,trip)
    std::vector<RouteLabel> routeLabels;
};

struct Transfers {
    Transfers(const Data& data)
        : beginOut(data.stopEventGraph.getBeginOut()),
          labels(data.stopEventGraph.numEdges()),
          travelTime(data.stopEventGraph.get(TravelTime)) {
        for (const Edge edge : data.stopEventGraph.edges()) {
            const StopEventId event(data.stopEventGraph.get(ToVertex, edge));
            const TripId trip = data.tripOfStopEvent[event];
            const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
            labels[edge].init(event, trip, firstEvent);
        }
    }

    std::vector<Edge> beginOut;
    std::vector<EdgeLabel> labels;
    std::vector<int> travelTime;
};

}  // namespace TripBased
