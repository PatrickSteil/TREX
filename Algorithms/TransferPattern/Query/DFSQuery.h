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

#include <vector>

#include "Profiler.h"
#include "TimestampedAlreadySeen.h"

static constexpr uint32_t MAX_LIMIT = (1u << 27) - 1; // 27-bit max
static constexpr uint32_t NO_CONNECTION = MAX_LIMIT;  // sentinel
static constexpr uint8_t MAX_TRIPS = 15;              // 4-bit max

struct PackedArrival {
  static constexpr uint32_t ARRIVAL_MASK = (1u << 27) - 1; // 27 bits
  static constexpr uint32_t TRIP_MASK = 0xF;               // 4 bits
  static constexpr uint32_t TRIP_SHIFT = 27;

  static inline uint32_t getArrival(uint32_t v) noexcept {
    return v & ARRIVAL_MASK;
  }

  static inline uint8_t getTrips(uint32_t v) noexcept {
    return (v >> TRIP_SHIFT) & TRIP_MASK;
  }

  static inline uint32_t setArrival(uint32_t v, uint32_t arr) noexcept {
    return (v & ~(ARRIVAL_MASK)) | (arr & ARRIVAL_MASK);
  }

  static inline uint32_t setTrips(uint32_t v, uint8_t trips) noexcept {
    return (v & ~(TRIP_MASK << TRIP_SHIFT)) |
           ((uint32_t(trips & TRIP_MASK)) << TRIP_SHIFT);
  }

  static inline uint32_t pack(uint32_t arr, uint8_t trips) noexcept {
    return (arr & ARRIVAL_MASK) | (uint32_t(trips & TRIP_MASK) << TRIP_SHIFT);
  }
};

// =========================================================
// TransferPattern::DFSQuery
// =========================================================
namespace TransferPattern {

template <typename PROFILER = NoProfiler> class DFSQuery {
public:
  using Profiler = PROFILER;
  using Self = DFSQuery<Profiler>;

  DFSQuery(Data &data)
      : data(data), sourceStop(noVertex), targetStop(noVertex),
        visited(data.maxNumVerticesAndNumEdgesInTP().first),
        packedArrival(data.maxNumVerticesAndNumEdgesInTP().first, MAX_LIMIT),
        targetArrivalTimes(16, MAX_LIMIT) {
    profiler.registerPhases({PHASE_CLEAR, PHASE_INIT_SOURCE_LABELS,
                             PHASE_EVAL_GRAPH, PHASE_EXTRACT_JOURNEYS});

    profiler.registerMetrics({METRIC_SETTLED_VERTICES,
                              METRIC_RELAXED_TRANSFER_EDGES,
                              METRIC_INCORPERATED_LABELS});
  }

  // ---------------------------------------------------------
  // Overload for Vertex input
  // ---------------------------------------------------------
  inline void run(Vertex source, int departureTime, Vertex target) {
    AssertMsg(data.raptorData.isStop(source),
              "Source " << (int)source << " is not a valid stop!");
    AssertMsg(data.raptorData.isStop(target),
              "Target " << (int)target << " is not a valid stop!");
    run(StopId(source), departureTime, StopId(target));
  }

  // ---------------------------------------------------------
  // Main run() method
  // ---------------------------------------------------------
  inline void run(StopId source, int departureTime, StopId target) {
    profiler.start();

    clear();
    sourceStop = source;
    targetStop = target;

    // initialize origin
    packedArrival[sourceStop] = PackedArrival::pack(departureTime, 0);

    const StaticDAGTransferPattern &tp = data.transferPatternOfStop[sourceStop];

    profiler.startPhase();
    dfs(tp, Vertex(targetStop), Vertex(targetStop));
    profiler.donePhase(PHASE_EVAL_GRAPH);

    profiler.done();
  }

  inline std::size_t getNumJourneysFound() const {
    std::size_t count = 0;
    uint32_t best = MAX_LIMIT;

    for (int i = 0; i < 16; ++i) {
      uint32_t t = targetArrivalTimes[i];
      count += (t < best);
      best = std::min(best, t);
    }
    return count;
  }

  void dfs(const StaticDAGTransferPattern &tp, Vertex from, Vertex to,
           int travelTime = 0) {
    profiler.countMetric(METRIC_SETTLED_VERTICES);
    // first visit => explore outgoing edges
    if (!visited.contains(from)) {
      visited.insert(from);

      for (const Edge edge : tp.edgesFrom(from)) {
        Vertex next = tp.get(ToVertex, edge);
        int edgeTravel = tp.get(TravelTime, edge);
        dfs(tp, next, from, edgeTravel);
      }
    }

    if (from == to) [[unlikely]]
      return;

    // process this (from -> to) connection
    uint32_t fromPacked = packedArrival[from];
    uint32_t fromArrival = PackedArrival::getArrival(fromPacked);
    uint8_t nrTrips = PackedArrival::getTrips(fromPacked);

    // compute new packed arrival for "to"
    uint32_t newPacked = MAX_LIMIT;

    Vertex stopFrom = tp.get(ViaVertex, from);
    Vertex stopTo = tp.get(ViaVertex, to);
    assert(stopFrom != stopTo);

    if (travelTime == -1) {
      // direct connection → increments trip count
      std::uint32_t newArrival =
          directConnectionIntersection(stopFrom, stopTo, fromArrival);

      ++nrTrips;
      newPacked = PackedArrival::pack(newArrival, nrTrips);
    } else {
      // simple transfer
      newPacked = PackedArrival::pack(travelTime, nrTrips);
    }

    packedArrival[to] = newPacked;

    // store plain time for target-stop arrivals
    if (stopTo == targetStop) [[unlikely]] {
      profiler.countMetric(METRIC_INCORPERATED_LABELS);
      uint32_t arr = PackedArrival::getArrival(newPacked);
      assert(nrTrips < targetArrivalTimes.size());
      targetArrivalTimes[nrTrips] = std::min(targetArrivalTimes[nrTrips], arr);
    }
  }

  inline std::uint32_t directConnectionIntersection(const Vertex from,
                                                    const Vertex to,
                                                    const int departureTime) {
    AssertMsg(data.raptorData.isStop(StopId(from)),
              "From " << from << " is not a valid stop");
    AssertMsg(data.raptorData.isStop(StopId(to)),
              "To " << to << "is not a valid stop");

    std::vector<RAPTOR::RouteSegment> &fromLookup =
        data.stopLookup[from].incidentLines;
    std::vector<RAPTOR::RouteSegment> &toLookup =
        data.stopLookup[to].incidentLines;

    AssertMsg(std::is_sorted(fromLookup.begin(), fromLookup.end()),
              "StopLookup from From is not sorted!");
    AssertMsg(std::is_sorted(toLookup.begin(), toLookup.end()),
              "StopLookup from From is not sorted!");

    size_t i(0);
    size_t j(0);

    size_t firstReachableTripIndex(-1);
    std::uint32_t currentArrivalTime(-1);
    std::uint32_t result(MAX_LIMIT);

    while (i < fromLookup.size() && j < toLookup.size()) {
      // if fromLookup[i] and toLookup[j] are on the same line ...
      // ... *and* fromLookup[i] is before toLookup[j] ...
      // ... then evaluate and get the intersection
      if (fromLookup[i].routeId == toLookup[j].routeId &&
          fromLookup[i].stopIndex < toLookup[j].stopIndex) {
        // get the first reachable trip departing >= departureTime
        firstReachableTripIndex = data.earliestTripIndexOfLineByStopIndex(
            fromLookup[i].stopIndex, fromLookup[i].routeId, departureTime);

        // is this firstReachableTripIndex valid? I.e., firstReachableTripIndex
        // != (-1)
        if (firstReachableTripIndex != (size_t)-1) {
          currentArrivalTime = data.getArrivalTime(fromLookup[i].routeId,
                                                   firstReachableTripIndex,
                                                   toLookup[j].stopIndex);
          result = std::min(currentArrivalTime, result);
        }
        ++i;
        ++j;
      } else {
        if (fromLookup[i] < toLookup[j]) {
          ++i;
        } else {
          ++j;
        }
      }
    }

    return result;
  }

  inline void clear() noexcept {
    profiler.startPhase();

    visited.clear();

    std::fill(targetArrivalTimes.begin(), targetArrivalTimes.end(), MAX_LIMIT);

    profiler.donePhase(PHASE_CLEAR);
  }

  inline Profiler &getProfiler() noexcept { return profiler; }

private:
  Data &data;

  Vertex sourceStop;
  Vertex targetStop;

  TimestampedAlreadySeen visited;

  std::vector<uint32_t> packedArrival;      // 27b time + 4b trip count
  std::vector<uint32_t> targetArrivalTimes; // only arrival times
  Profiler profiler;
};

} // namespace TransferPattern
