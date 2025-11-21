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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

#include "Profiler.h"
#include "TimestampedAlreadySeen.h"

// =========================================================
// Constants
// =========================================================
static constexpr uint32_t MAX_LIMIT =
    (1u << 27) - 1;  // 27-bit max (also sentinel)
static constexpr uint32_t NO_CONNECTION =
    MAX_LIMIT;                            // sentinel for no connection
static constexpr uint8_t MAX_TRIPS = 15;  // 4-bit max

// =========================================================
// Bit-packed arrival time + trip count
//  - Bits 0..26 : arrival time (27 bits)
//  - Bits 27..30: trip count (4 bits)
// =========================================================
struct PackedArrival {
  static constexpr uint32_t ARRIVAL_MASK = (1u << 27) - 1;  // 27 bits
  static constexpr uint32_t TRIP_MASK = 0xF;                // 4 bits
  static constexpr uint32_t TRIP_SHIFT = 27;

  static inline uint32_t getArrival(uint32_t v) noexcept {
    return v & ARRIVAL_MASK;
  }

  static inline uint8_t getTrips(uint32_t v) noexcept {
    return static_cast<uint8_t>((v >> TRIP_SHIFT) & TRIP_MASK);
  }

  // ensure arrival fits into ARRIVAL_MASK
  static inline constexpr uint32_t clampArrival(uint64_t arr) noexcept {
    return static_cast<uint32_t>((arr > ARRIVAL_MASK) ? ARRIVAL_MASK : arr);
  }

  // set arrival preserving trips
  static inline uint32_t setArrival(uint32_t v, uint32_t arr) noexcept {
    arr = arr & ARRIVAL_MASK;
    return (v & ~(ARRIVAL_MASK)) | (arr & ARRIVAL_MASK);
  }

  // set trips preserving arrival
  static inline uint32_t setTrips(uint32_t v, uint8_t trips) noexcept {
    return (v & ~(TRIP_MASK << TRIP_SHIFT)) |
           ((uint32_t(trips & TRIP_MASK)) << TRIP_SHIFT);
  }

  // pack arrival + trips, clamping arrival if necessary
  static inline uint32_t pack(uint32_t arr, uint8_t trips) noexcept {
    uint32_t clamped = arr & ARRIVAL_MASK;
    return (clamped) | (uint32_t(trips & TRIP_MASK) << TRIP_SHIFT);
  }

  // debug print
  static inline void print(uint32_t v) {
    uint32_t arrTime = getArrival(v);
    uint8_t nrTrips = getTrips(v);
    std::cout << "ArrTime " << arrTime << ", Nr Trips: " << int(nrTrips)
              << std::endl;
  }
};

// =========================================================
// TransferPattern::DFSQuery
// =========================================================
namespace TransferPattern {

template <typename PROFILER = NoProfiler>
class DFSQuery {
 public:
  using Profiler = PROFILER;
  using Self = DFSQuery<Profiler>;

  DFSQuery(Data &data)
      : data(data),
        sourceStop(noVertex),
        targetStop(noVertex),
        visited(data.maxNumVerticesAndNumEdgesInTP().first),
        packedArrival(data.maxNumVerticesAndNumEdgesInTP().first,
                      PackedArrival::pack(MAX_LIMIT, 0)),
        targetArrivalTimes(16, MAX_LIMIT) {
    profiler.registerPhases({PHASE_CLEAR, PHASE_INIT_SOURCE_LABELS,
                             PHASE_EVAL_GRAPH, PHASE_EXTRACT_JOURNEYS});

    profiler.registerMetrics({METRIC_SETTLED_VERTICES,
                              METRIC_RELAXED_TRANSFER_EDGES,
                              METRIC_INCORPERATED_LABELS});
  }

  inline void run(Vertex source, int departureTime, Vertex target) {
    AssertMsg(data.raptorData.isStop(source),
              "Source " << int(source) << " is not a valid stop!");
    AssertMsg(data.raptorData.isStop(target),
              "Target " << int(target) << " is not a valid stop!");
    run(StopId(source), departureTime, StopId(target));
  }

  inline void run(StopId source, int departureTime, StopId target) {
    profiler.start();

    clear();
    sourceStop = source;
    targetStop = target;

    if (sourceStop == targetStop) [[unlikely]] {
      targetArrivalTimes[0] = static_cast<uint32_t>(departureTime);
    } else {
      packedArrival[sourceStop] =
          PackedArrival::pack(static_cast<uint32_t>(departureTime), 0);

      const StaticDAGTransferPattern &tp =
          data.transferPatternOfStop[sourceStop];

      profiler.startPhase();
      dfs(tp, Vertex(targetStop), Vertex(targetStop));
      profiler.donePhase(PHASE_EVAL_GRAPH);
    }

    profiler.done();
  }

  inline std::size_t getNumJourneysFound() const {
    std::size_t count = 0;
    uint32_t best = MAX_LIMIT;

    for (int i = 0; i < 16; ++i) {
      uint32_t t = targetArrivalTimes[i];
      if (t < best) {
        ++count;
        best = t;
      }
    }
    return count;
  }

  // Saturating add: if a == NO_CONNECTION, keep NO_CONNECTION.
  // Otherwise compute a + b and clamp at MAX_LIMIT.
  static inline uint32_t saturatedAdd(uint32_t a, int b) noexcept {
    if (a == NO_CONNECTION) return NO_CONNECTION;
    if (b < 0) {
      // negative travel times are not expected; treat safely
      uint64_t ua = a;
      int64_t res = static_cast<int64_t>(ua) + static_cast<int64_t>(b);
      return (res < 0) ? 0u : static_cast<uint32_t>(res);
    }
    uint64_t sum = uint64_t(a) + uint64_t(b);
    return static_cast<uint32_t>((sum >= MAX_LIMIT) ? MAX_LIMIT : sum);
  }

  void dfs(const StaticDAGTransferPattern &tp, Vertex from, Vertex to,
           int travelTime = 0) {
    profiler.countMetric(METRIC_SETTLED_VERTICES);

    // explore children first (postorder)
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

    // get packed info for 'from'
    uint32_t fromPacked = packedArrival[from];
    uint32_t fromArrival = PackedArrival::getArrival(fromPacked);
    uint8_t nrTrips = PackedArrival::getTrips(fromPacked);

    uint32_t newPacked = PackedArrival::pack(MAX_LIMIT, nrTrips);  // default

    Vertex stopFrom = tp.get(ViaVertex, from);
    Vertex stopTo = tp.get(ViaVertex, to);

    if (travelTime == -1) {
      // direct connection: compute earliest arrival at stopTo when departing
      // from stopFrom not earlier than fromArrival
      uint32_t arrivalDC =
          directConnectionIntersection(stopFrom, stopTo, fromArrival);

      if (arrivalDC != NO_CONNECTION) {
        // increment trips but saturate to MAX_TRIPS
        uint8_t newTrips = (nrTrips < MAX_TRIPS) ? (nrTrips + 1) : MAX_TRIPS;
        newPacked = PackedArrival::pack(arrivalDC, newTrips);
      } else {
        // no direct connection; keep arrival sentinel, preserve trips
        newPacked = PackedArrival::pack(NO_CONNECTION, nrTrips);
      }
    } else {
      // simple transfer edge: arrival = fromArrival + travelTime (saturating)
      uint32_t arrival = saturatedAdd(fromArrival, travelTime);
      newPacked = PackedArrival::pack(arrival, nrTrips);
    }

    packedArrival[to] = newPacked;

    // store plain time for target-stop arrivals
    if (stopTo == targetStop) [[unlikely]] {
      profiler.countMetric(METRIC_INCORPERATED_LABELS);
      uint32_t arr = PackedArrival::getArrival(newPacked);
      // defensive: ensure index valid (nrTrips ∈ [0, MAX_TRIPS])
      const size_t idx =
          static_cast<size_t>(std::min<uint8_t>(nrTrips, MAX_TRIPS));
      targetArrivalTimes[idx] = std::min(targetArrivalTimes[idx], arr);
    }
  }

  // compute earliest arrival at 'to' when leaving 'from' not earlier than
  // departureTime. Returns NO_CONNECTION (MAX_LIMIT) if none available.
  inline uint32_t directConnectionIntersection(
      const Vertex from, const Vertex to,
      const uint32_t departureTime) noexcept {
    AssertMsg(data.raptorData.isStop(StopId(from)),
              "From " << int(from) << " is not a valid stop");
    AssertMsg(data.raptorData.isStop(StopId(to)),
              "To " << int(to) << " is not a valid stop");

    // references to incident lines for both stops (must be sorted by
    // routeId/stopIndex)
    const std::vector<RAPTOR::RouteSegment> &fromLookup =
        data.stopLookup[from].incidentLines;
    const std::vector<RAPTOR::RouteSegment> &toLookup =
        data.stopLookup[to].incidentLines;

    AssertMsg(std::is_sorted(fromLookup.begin(), fromLookup.end()),
              "StopLookup for from is not sorted!");
    AssertMsg(std::is_sorted(toLookup.begin(), toLookup.end()),
              "StopLookup for to is not sorted!");

    size_t i = 0, j = 0;
    uint32_t best = NO_CONNECTION;

    while (i < fromLookup.size() && j < toLookup.size()) {
      const auto &a = fromLookup[i];
      const auto &b = toLookup[j];

      if (a.routeId == b.routeId && a.stopIndex < b.stopIndex) {
        // find earliest trip on this route/stopIndex that departs >=
        // departureTime
        size_t tripIndex = data.earliestTripIndexOfLineByStopIndex(
            a.stopIndex, a.routeId, departureTime);

        if (tripIndex != static_cast<size_t>(-1)) {
          // fetch arrival time at the destination stopIndex for this trip
          uint32_t arrivalTime =
              data.getArrivalTime(a.routeId, tripIndex, b.stopIndex);

          // clamp arrivalTime (defensive) and update best
          if (arrivalTime != NO_CONNECTION) {
            uint32_t clamped =
                (arrivalTime > MAX_LIMIT) ? MAX_LIMIT : arrivalTime;
            if (clamped < best) best = clamped;
          }
        }

        ++i;
        ++j;
      } else {
        if (a < b) {
          ++i;
        } else {
          ++j;
        }
      }
    }

    return best;  // NO_CONNECTION if unchanged
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

  std::vector<uint32_t> packedArrival;       // 27b time + 4b trip count
  std::vector<uint32_t> targetArrivalTimes;  // only arrival times
  Profiler profiler;
};

}  // namespace TransferPattern
