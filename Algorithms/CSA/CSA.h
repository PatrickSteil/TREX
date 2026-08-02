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
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/CSA/Entities/Journey.h"
#include "../../Helpers/Assert.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/Types.h"
#include "../../Helpers/Vector/Vector.h"
#include "Profiler.h"
#include "TimeShift.h"

namespace CSA {
template <bool PATH_RETRIEVAL = true, typename PROFILER = NoProfiler,
          bool TRANSFERS_SECOND_CRIT = false>
class CSA {
 public:
  constexpr static bool PathRetrieval = PATH_RETRIEVAL;
  constexpr static bool TransfersSecondCrit = TRANSFERS_SECOND_CRIT;
  using Profiler = PROFILER;
  using Type = CSA<PathRetrieval, Profiler, TransfersSecondCrit>;
  using TripFlag = Meta::IF<PathRetrieval, std::size_t, bool>;

  static constexpr size_t PrefetchDistance = 8;

 private:
  struct ParentLabel {
    ParentLabel(const StopId parent = noStop,
                const bool reachedByTransfer = false,
                const TripId tripId = noTripId)
        : parent(parent),
          reachedByTransfer(reachedByTransfer),
          tripId(tripId) {}

    StopId parent;
    bool reachedByTransfer;
    union {
      TripId tripId;
      Edge transferId;
    };
  };

  // Colocates the two fields that are read together on every single
  // connection scan (`arrivalTime[stop]` and `minTransferTime[stop]`),
  // so both come from one cache line instead of two independent
  // random-indexed vectors.
  struct StopLabel {
    int32_t arrivalTime;
    int32_t minTransferTime;
  };

  // Colocates the trip-reached state with the transfer count. The
  // "reached" check no longer depends on comparing `reached` against
  // TripFlag() (which is ambiguous for size_t: a legitimate connection
  // index of 0 looked identical to "not reached"). Instead a dedicated
  // uint8_t sentinel marks reached/unreached, decoupled from the stored
  // index.
  struct TripLabel {
    static constexpr uint8_t NotReached = static_cast<uint8_t>(-1);

    TripFlag reached;  // connection id, only meaningful for PathRetrieval +
                       // when reached
    uint8_t transferCount =
        NotReached;  // NotReached sentinel, else 0..31 transfer count
  };

 public:
  CSA(const Data& data, const Profiler& profilerTemplate = Profiler())
      : data(data),
        sourceStop(noStop),
        targetStop(noStop),
        tripLabel(data.numberOfTrips()),
        stopLabel(data.numberOfStops()),
        parentLabel(PathRetrieval ? data.numberOfStops() : 0),
        profiler(profilerTemplate) {
    AssertMsg(Vector::isSorted(data.connections),
              "Connections must be sorted in ascending order!");
    for (const StopId stop : data.stops()) {
      stopLabel[stop].minTransferTime = data.minTransferTime(stop);
    }
    profiler.registerPhases(
        {PHASE_CLEAR, PHASE_INITIALIZATION, PHASE_CONNECTION_SCAN});
    profiler.registerMetrics({METRIC_CONNECTIONS, METRIC_EDGES,
                              METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
    profiler.initialize();
  }

  inline void run(const StopId source, const int departureTime,
                  const StopId target = noStop) noexcept {
    profiler.start();

    profiler.startPhase();
    AssertMsg(data.isStop(source),
              "Source stop " << source << " is not a valid stop!");
    clear();
    profiler.donePhase(PHASE_CLEAR);

    profiler.startPhase();
    sourceStop = source;
    targetStop = target;
    const int transformedDepartureTime = transformTime(departureTime);
    stopLabel[sourceStop].arrivalTime = transformedDepartureTime;
    relaxEdges(sourceStop, transformedDepartureTime);
    const std::size_t firstConnection =
        firstReachableConnection(transformedDepartureTime);
    profiler.donePhase(PHASE_INITIALIZATION);

    profiler.startPhase();
    scanConnections(firstConnection, std::size_t(data.connections.size()));
    profiler.donePhase(PHASE_CONNECTION_SCAN);

    profiler.done();
  }

  inline bool reachable(const StopId stop) const noexcept {
    return stopLabel[stop].arrivalTime < never;
  }

  inline int getEarliestArrivalTime(const StopId stop) const noexcept {
    if constexpr (TransfersSecondCrit) {
      return getExactArrivalTime(stopLabel[stop].arrivalTime);
    } else {
      return stopLabel[stop].arrivalTime;
    }
  }

  template <bool T = TransfersSecondCrit,
            typename = std::enable_if_t<T == TransfersSecondCrit && T>>
  inline int getTransferCount(const StopId stop) const noexcept {
    return getNumberOfTransfers(stopLabel[stop].arrivalTime);
  }

  template <bool T = PathRetrieval,
            typename = std::enable_if_t<T == PathRetrieval && T>>
  inline Journey getJourney() const noexcept {
    return getJourney(targetStop);
  }

  template <bool T = PathRetrieval,
            typename = std::enable_if_t<T == PathRetrieval && T>>
  inline Journey getJourney(StopId stop) const noexcept {
    Journey journey;
    if (!reachable(stop)) return journey;
    while (stop != sourceStop) {
      const ParentLabel& label = parentLabel[stop];
      const int exactArrival = getEarliestArrivalTime(stop);
      if (label.reachedByTransfer) {
        const int travelTime =
            data.transferGraph.get(TravelTime, label.transferId);
        journey.emplace_back(label.parent, stop, exactArrival - travelTime,
                             exactArrival, label.transferId);
      } else {
        journey.emplace_back(
            label.parent, stop,
            data.connections[tripLabel[label.tripId].reached].departureTime,
            exactArrival, label.tripId);
      }
      stop = label.parent;
    }
    Vector::reverse(journey);
    return journey;
  }

  inline std::vector<Vertex> getPath(const StopId stop) const noexcept {
    return journeyToPath(getJourney(stop));
  }

  inline std::vector<std::string> getRouteDescription(
      const StopId stop) const noexcept {
    return data.journeyToText(getJourney(stop));
  }

  inline const Profiler& getProfiler() const noexcept { return profiler; }

 private:
  inline void clear() {
    sourceStop = noStop;
    targetStop = noStop;
    for (StopLabel& label : stopLabel) label.arrivalTime = never;
    for (TripLabel& label : tripLabel)
      label.transferCount = TripLabel::NotReached;
    if constexpr (PathRetrieval) {
      Vector::fill(parentLabel, ParentLabel());
    }
  }

  inline int transformTime(const int time) const noexcept {
    if constexpr (!TransfersSecondCrit) {
      return time;
    } else {
      return shiftTime(time);
    }
  }

  inline std::size_t firstReachableConnection(
      const int departureTime) const noexcept {
    return std::size_t(Vector::lowerBound(
        data.connections, departureTime,
        [this](const Connection& connection, const int time) {
          return transformTime(connection.departureTime) < time;
        }));
  }

  inline void scanConnections(const std::size_t begin,
                              const std::size_t end) noexcept {
    for (std::size_t i = begin; i < end; i++) {
      prefetchUpcomingConnection(i, end);

      const Connection& connection = data.connections[i];
      if (targetStop != noStop && transformTime(connection.departureTime) >
                                      stopLabel[targetStop].arrivalTime)
          [[unlikely]]
        break;
      if (connectionIsReachable(connection, i)) {
        profiler.countMetric(METRIC_CONNECTIONS);
        arrivalByTrip(connection.arrivalStopId, connection.arrivalTime,
                      connection.tripId);
      }
    }
  }

  inline void prefetchUpcomingConnection(const std::size_t i,
                                         const std::size_t end) noexcept {
    const std::size_t prefetchIndex = std::size_t(i + PrefetchDistance);
    if (prefetchIndex >= end) return;
    const Connection& upcoming = data.connections[prefetchIndex];
    // Locality hint 3 (highest): the prefetched connection is consumed
    // within the next PrefetchDistance loop iterations, a short reuse
    // window, so it should land in L1 and stay there rather than being
    // evicted early.
    __builtin_prefetch(&tripLabel[upcoming.tripId], 0, 3);
    __builtin_prefetch(&stopLabel[upcoming.departureStopId], 0, 3);
  }

  inline bool connectionIsReachableFromStop(
      const Connection& connection) const noexcept {
    const StopLabel& label = stopLabel[connection.departureStopId];
    return label.arrivalTime <=
           transformTime(connection.departureTime) - label.minTransferTime;
  }

  inline bool connectionIsReachableFromTrip(
      const Connection& connection) const noexcept {
    return tripLabel[connection.tripId].transferCount != TripLabel::NotReached;
  }

  inline bool connectionIsReachable(const Connection& connection,
                                    const std::size_t id) noexcept {
    TripLabel& label = tripLabel[connection.tripId];
    if (label.transferCount != TripLabel::NotReached) return true;
    if (connectionIsReachableFromStop(connection)) {
      if constexpr (PathRetrieval) {
        label.reached = id;
      } else {
        suppressUnusedParameterWarning(id);
        label.reached = true;
      }
      if constexpr (TransfersSecondCrit) {
        const int transfersAtBoardingStop = getNumberOfTransfers(
            stopLabel[connection.departureStopId].arrivalTime);
        AssertMsg(transfersAtBoardingStop + 1 <= 31,
                  "Transfer count exceeds the 5 bits reserved for it by "
                  "the bit-packing trick (see TimeShift.h)!");
        label.transferCount = static_cast<uint8_t>(transfersAtBoardingStop + 1);
      } else {
        label.transferCount = 0;
      }
      return true;
    }
    return false;
  }

  inline void arrivalByTrip(const StopId stop, const int time,
                            const TripId trip) noexcept {
    const int candidateTime = arrivalTimeViaTrip(time, trip);
    if (stopLabel[stop].arrivalTime <= candidateTime) return;
    profiler.countMetric(METRIC_STOPS_BY_TRIP);
    stopLabel[stop].arrivalTime = candidateTime;
    if constexpr (PathRetrieval) {
      parentLabel[stop].parent =
          data.connections[tripLabel[trip].reached].departureStopId;
      parentLabel[stop].reachedByTransfer = false;
      parentLabel[stop].tripId = trip;
    }
    relaxEdges(stop, candidateTime);
  }

  inline int arrivalTimeViaTrip(const int connectionArrivalTime,
                                const TripId trip) const noexcept {
    if constexpr (!TransfersSecondCrit) {
      return connectionArrivalTime;
    } else {
      return shiftTime(connectionArrivalTime) +
             tripLabel[trip].transferCount * offset;
    }
  }

  inline void relaxEdges(const StopId stop, const int time) noexcept {
    const auto& edges = data.transferGraph.edgesFrom(stop);
    // TODO maybe unroll
    for (const Edge edge : edges) {
      profiler.countMetric(METRIC_EDGES);
      const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
      const int newArrivalTime =
          time + transformTime(data.transferGraph.get(TravelTime, edge));
      arrivalByTransfer(toStop, newArrivalTime, stop, edge);
    }
  }

  inline void arrivalByTransfer(const StopId stop, const int time,
                                const StopId parent, const Edge edge) noexcept {
    if (stopLabel[stop].arrivalTime <= time) return;
    profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
    stopLabel[stop].arrivalTime = time;
    if constexpr (PathRetrieval) {
      parentLabel[stop].parent = parent;
      parentLabel[stop].reachedByTransfer = true;
      parentLabel[stop].transferId = edge;
    }
  }

 private:
  const Data& data;

  StopId sourceStop;
  StopId targetStop;

  std::vector<TripLabel> tripLabel;
  std::vector<StopLabel> stopLabel;
  std::vector<ParentLabel> parentLabel;

  Profiler profiler;
};
}  // namespace CSA
