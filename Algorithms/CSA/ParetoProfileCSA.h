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

#if !defined(__AVX2__)
#error \
    "ParetoProfileCSASIMD.h uses AVX2 intrinsics. Compile with -mavx2 (and " \
    "typically -march=native)."
#endif

#include <immintrin.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "../../DataStructures/CSA/Data.h"
#include "../../Helpers/Assert.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/Types.h"
#include "../../Helpers/Vector/Vector.h"
#include "Profiler.h"

namespace CSA {

namespace SIMD {

class alignas(32) Leg8 {
 public:
  static constexpr int LegMax = 8;

  inline Leg8() noexcept : vec(_mm256_set1_epi32(never)) {}
  inline explicit Leg8(const __m256i v) noexcept : vec(v) {}

  static inline Leg8 broadcast(const int value) noexcept {
    return Leg8(_mm256_set1_epi32(value));
  }

  static inline Leg8 infinity() noexcept { return broadcast(never); }

  static inline Leg8 load(const int* src) noexcept {
    return Leg8(_mm256_loadu_si256(reinterpret_cast<const __m256i*>(src)));
  }

  inline void store(int* dest) const noexcept {
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(dest), vec);
  }

  inline std::array<int, LegMax> toArray() const noexcept {
    alignas(32) std::array<int, LegMax> out;
    store(out.data());
    return out;
  }

  static inline Leg8 min(const Leg8& a, const Leg8& b) noexcept {
    return Leg8(_mm256_min_epi32(a.vec, b.vec));
  }

  inline Leg8 shift() const noexcept {
    static const __m256i permIdx = _mm256_setr_epi32(0, 0, 1, 2, 3, 4, 5, 6);
    const __m256i permuted = _mm256_permutevar8x32_epi32(vec, permIdx);
    return Leg8(
        _mm256_blend_epi32(permuted, _mm256_set1_epi32(never), 0b00000001));
  }

  inline Leg8 shiftAccumulateLast() const noexcept {
    const Leg8 shifted = shift();
    const int secondToLast = _mm256_extract_epi32(vec, LegMax - 2);
    const int last = _mm256_extract_epi32(vec, LegMax - 1);
    const int combined = std::min(secondToLast, last);
    return Leg8(_mm256_insert_epi32(shifted.vec, combined, LegMax - 1));
  }

  // True iff a[i] < b[i] in at least one lane i (a improves on b somewhere).
  static inline int lessThanMask(const Leg8& a, const Leg8& b) noexcept {
    const __m256i less = _mm256_cmpgt_epi32(b.vec, a.vec);
    return _mm256_movemask_ps(_mm256_castsi256_ps(less));
  }

  static inline bool dominatesAll(const Leg8& a, const Leg8& b) noexcept {
    const __m256i greater = _mm256_cmpgt_epi32(a.vec, b.vec);
    return _mm256_movemask_ps(_mm256_castsi256_ps(greater)) == 0;
  }

  inline bool improvesOn(const Leg8& other) const noexcept {
    return lessThanMask(*this, other) != 0;
  }

  __m256i vec;
};

}  // namespace SIMD

template <bool WITH_JOURNEYS>
struct JourneyLegsImpl {
  using LegVector = std::array<ConnectionId, SIMD::Leg8::LegMax>;

  LegVector enterLeg;
  LegVector exitLeg;

  JourneyLegsImpl() noexcept {
    enterLeg.fill(ConnectionId(-1));
    exitLeg.fill(ConnectionId(-1));
  }

  inline ConnectionId enter(const int leg) const noexcept {
    return enterLeg[leg];
  }
  inline ConnectionId exit(const int leg) const noexcept {
    return exitLeg[leg];
  }
  inline void setEnter(const int leg, const ConnectionId c) noexcept {
    enterLeg[leg] = c;
  }
  inline void setExit(const int leg, const ConnectionId c) noexcept {
    exitLeg[leg] = c;
  }
};

template <>
struct JourneyLegsImpl<false> {
  inline ConnectionId enter(const int) const noexcept {
    return ConnectionId(-1);
  }
  inline ConnectionId exit(const int) const noexcept {
    return ConnectionId(-1);
  }
  inline void setEnter(const int, const ConnectionId) noexcept {}
  inline void setExit(const int, const ConnectionId) noexcept {}
};

template <bool ACCUMULATE_UNBOUNDED_LEGS = true, bool WITH_JOURNEYS = true,
          typename PROFILER = NoProfiler>
class ParetoProfileCSASIMD {
 public:
  static constexpr bool AccumulateUnboundedLegs = ACCUMULATE_UNBOUNDED_LEGS;
  static constexpr bool WithJourneys = WITH_JOURNEYS;
  static constexpr int LegMax = SIMD::Leg8::LegMax;
  using Profiler = PROFILER;
  using Type =
      ParetoProfileCSASIMD<AccumulateUnboundedLegs, WithJourneys, Profiler>;
  using JourneyLegs = JourneyLegsImpl<WithJourneys>;

  struct Leg {
    Leg(const ConnectionId enter = ConnectionId(-1),
        const ConnectionId exit = ConnectionId(-1))
        : enter(enter), exit(exit) {}

    ConnectionId enter;
    ConnectionId exit;
  };

 private:
  struct ProfileElement {
    ProfileElement(const int departureTime = never,
                   const SIMD::Leg8 arrivalTimes = SIMD::Leg8::infinity(),
                   JourneyLegs legs = JourneyLegs())
        : departureTime(departureTime),
          arrivalTimes(arrivalTimes),
          legs(std::move(legs)) {}

    inline int getDepartureTime() const noexcept { return departureTime; }
    inline int getArrivalTime(const int legCount) const noexcept {
      return arrivalTimes.toArray()[legCount - 1];
    }

    int departureTime;
    SIMD::Leg8 arrivalTimes;
    JourneyLegs legs;
  };

  struct Profile {
    Profile() : elements(1, ProfileElement()) { elements.reserve(64); }

    inline int size() const noexcept {
      return static_cast<int>(elements.size());
    }

    inline int findIndex(const int departureTime) const noexcept {
      int i = size() - 1;
      while (elements[i].getDepartureTime() < departureTime) --i;
      return i;
    }

    inline const ProfileElement& evaluateElement(
        const int departureTime) const noexcept {
      return elements[findIndex(departureTime)];
    }

    inline SIMD::Leg8 evaluate(const int departureTime) const noexcept {
      return evaluateElement(departureTime).arrivalTimes;
    }

    inline bool isDominated(const int departureTime,
                            const SIMD::Leg8& candidate) const noexcept {
      return !candidate.improvesOn(evaluate(departureTime));
    }

    inline void incorporate(ProfileElement newElement) noexcept {
      const int newDepartureTime = newElement.getDepartureTime();

      int pos = size() - 1;
      while (elements[pos].getDepartureTime() < newDepartureTime) --pos;

      if (elements[pos].getDepartureTime() == newDepartureTime) {
        if (!newElement.arrivalTimes.improvesOn(elements[pos].arrivalTimes))
          return;
        const int improvedMask = SIMD::Leg8::lessThanMask(
            newElement.arrivalTimes, elements[pos].arrivalTimes);
        if constexpr (WithJourneys) {
          for (int leg = 0; leg < LegMax; ++leg) {
            if (improvedMask & (1 << leg)) {
              elements[pos].legs.setEnter(leg, newElement.legs.enter(leg));
              elements[pos].legs.setExit(leg, newElement.legs.exit(leg));
            }
          }
        }
        elements[pos].arrivalTimes = SIMD::Leg8::min(elements[pos].arrivalTimes,
                                                     newElement.arrivalTimes);
      } else {
        elements.insert(elements.begin() + pos + 1, std::move(newElement));
        ++pos;
      }

      const SIMD::Leg8 arrival = elements[pos].arrivalTimes;
      int write = pos + 1;
      for (int read = pos + 1; read < size(); ++read) {
        if (SIMD::Leg8::dominatesAll(arrival, elements[read].arrivalTimes))
          continue;
        if (write != read) elements[write] = std::move(elements[read]);
        ++write;
      }
      elements.resize(write);
    }

    void printElements() const {
      for (const ProfileElement& e : elements) {
        std::cout << String::secToString(e.getDepartureTime()) << " -> [ ";
        for (int legs = 1; legs <= LegMax; ++legs) {
          std::cout << String::secToString(e.getArrivalTime(legs));
          if (legs != LegMax) std::cout << ", ";
        }
        std::cout << " ]\n";
      }
    }

    std::vector<ProfileElement> elements;
  };

  struct TripLabel {
    SIMD::Leg8 arrivalTimes = SIMD::Leg8::infinity();
    JourneyLegs legs;
  };

 public:
  ParetoProfileCSASIMD(Data& data,
                       const Profiler& profilerTemplate = Profiler())
      : data(data),
        reverseTransferGraph(data.transferGraph),
        targetStop(noStop),
        tripLabel(data.numberOfTrips()),
        profiles(data.numberOfStops(), Profile()),
        distanceToTarget(data.numberOfStops(), INFTY),
        profiler(profilerTemplate) {
    static_assert(sizeof(int) == 4, "SIMD::Leg8 assumes 32-bit int lanes!");
    reverseTransferGraph.revert();
    AssertMsg(Vector::isSorted(data.connections),
              "Connections must be sorted in ascending order!");
    profiler.registerPhases(
        {PHASE_CLEAR, PHASE_INITIALIZATION, PHASE_CONNECTION_SCAN});
    profiler.registerMetrics({METRIC_CONNECTIONS, METRIC_EDGES,
                              METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
    profiler.initialize();
  }

  inline void run(const StopId target, int minDepartureTime = 0,
                  int maxDepartureTime = 24 * 60 * 60) noexcept {
    AssertMsg(data.isStop(target),
              "Target stop " << target << " is not a valid stop!");
    AssertMsg(minDepartureTime < maxDepartureTime,
              "minDepartureTime must be smaller than maxDepartureTime!");

    profiler.start();

    profiler.startPhase();
    resetDistancesToTarget(target);
    clear();
    targetStop = target;
    profiler.donePhase(PHASE_CLEAR);

    profiler.startPhase();
    const ConnectionId first = firstConnectionNotBefore(minDepartureTime);
    const ConnectionId last = firstConnectionNotBefore(maxDepartureTime + 1);
    profiler.donePhase(PHASE_INITIALIZATION);

    profiler.startPhase();
    scanConnections(first, last);
    profiler.donePhase(PHASE_CONNECTION_SCAN);

    profiler.done();
  }

  inline int earliestArrivalTime(const StopId stop, const int time,
                                 const int legs) const noexcept {
    AssertMsg(legs >= 1 && legs <= LegMax,
              "legs out of range [1, " << LegMax << "]!");
    return profiles[stop].evaluate(time).toArray()[legs - 1];
  }

  inline bool reachable(const StopId stop, const int time,
                        const int legs) const noexcept {
    return earliestArrivalTime(stop, time, legs) < never;
  }

  inline std::vector<Leg> getJourney(const StopId source, const int time,
                                     const int legs) const noexcept {
    static_assert(WithJourneys,
                  "getJourney() requires ParetoProfileCSASIMD<..., true, ...> "
                  "(WITH_JOURNEYS = true)");
    AssertMsg(legs >= 1 && legs <= LegMax,
              "legs out of range [1, " << LegMax << "]!");
    std::vector<Leg> journey;
    StopId stop = source;
    int departureTime = time;
    int legsLeft = legs;

    while (legsLeft > 0) {
      const ProfileElement& element =
          profiles[stop].evaluateElement(departureTime);
      const ConnectionId enter = element.legs.enter(legsLeft - 1);
      if (enter == ConnectionId(-1)) break;  // remainder is a plain footpath
      const ConnectionId exit = element.legs.exit(legsLeft - 1);
      journey.emplace_back(enter, exit);
      stop = data.connections[exit].arrivalStopId;
      departureTime = data.connections[exit].arrivalTime;
      --legsLeft;
    }
    return journey;
  }

  inline int numberOfJourneys(const StopId stop) const noexcept {
    return profiles[stop].size() - 1;
  }

  inline const Profiler& getProfiler() const noexcept { return profiler; }

  void printProfile(const StopId stop) const { profiles[stop].printElements(); }

 private:
  inline void resetDistancesToTarget(const StopId newTarget) noexcept {
    if (targetStop != noStop) {
      distanceToTarget[targetStop] = INFTY;
      for (const Edge edge : reverseTransferGraph.edgesFrom(targetStop))
        distanceToTarget[reverseTransferGraph.get(ToVertex, edge)] = INFTY;
    }
    distanceToTarget[newTarget] = 0;
    for (const Edge edge : reverseTransferGraph.edgesFrom(newTarget))
      distanceToTarget[reverseTransferGraph.get(ToVertex, edge)] =
          reverseTransferGraph.get(TravelTime, edge);
  }

  inline void clear() noexcept {
    targetStop = noStop;
    for (Profile& profile : profiles) profile = Profile();
    for (TripLabel& label : tripLabel) label = TripLabel();
  }

  inline ConnectionId firstConnectionNotBefore(
      const int departureTime) const noexcept {
    return ConnectionId(
        Vector::lowerBound(data.connections, departureTime,
                           [](const Connection& connection, const int time) {
                             return connection.departureTime < time;
                           }));
  }

  inline void scanConnections(const ConnectionId first,
                              const ConnectionId last) noexcept {
    AssertMsg(last <= data.connections.size(), "last out of bounds!");
    for (ConnectionId i = last; i > first; --i) {
      profiler.countMetric(METRIC_CONNECTIONS);
      scanConnection(data.connections[i - 1], ConnectionId(i - 1));
    }
  }

  inline void scanConnection(const Connection& c,
                             const ConnectionId idx) noexcept {
    TripLabel& trip = tripLabel[c.tripId];

    const SIMD::Leg8 tau1 = SIMD::Leg8::broadcast(
        c.arrivalTime + distanceToTarget[c.arrivalStopId]);
    const SIMD::Leg8 tau2 = trip.arrivalTimes;
    const SIMD::Leg8 continuation =
        profiles[c.arrivalStopId].evaluate(c.arrivalTime);
    const SIMD::Leg8 tau3 = AccumulateUnboundedLegs
                                ? continuation.shiftAccumulateLast()
                                : continuation.shift();

    const SIMD::Leg8 tauC = SIMD::Leg8::min(tau1, SIMD::Leg8::min(tau2, tau3));

    if constexpr (WithJourneys) {
      const int decreasedMask = SIMD::Leg8::lessThanMask(tauC, tau2);
      for (int leg = 0; leg < LegMax; ++leg)
        if (decreasedMask & (1 << leg)) trip.legs.setExit(leg, idx);
    }
    trip.arrivalTimes = tauC;

    Profile& departureProfile = profiles[c.departureStopId];
    const ProfileElement& previousBest =
        departureProfile.evaluateElement(c.departureTime);

    if (!tauC.improvesOn(previousBest.arrivalTimes)) return;

    const SIMD::Leg8 merged = SIMD::Leg8::min(previousBest.arrivalTimes, tauC);

    JourneyLegs legs = previousBest.legs;
    if constexpr (WithJourneys) {
      const int improvedMask =
          SIMD::Leg8::lessThanMask(tauC, previousBest.arrivalTimes);
      for (int leg = 0; leg < LegMax; ++leg) {
        if (improvedMask & (1 << leg)) {
          legs.setEnter(leg, idx);
          legs.setExit(leg, trip.legs.exit(leg));
        }
      }
    }

    profiler.countMetric(METRIC_STOPS_BY_TRIP);
    ProfileElement newElement(c.departureTime, merged, std::move(legs));
    departureProfile.incorporate(newElement);
    relaxIncomingEdges(c.departureStopId, newElement);
  }

  inline void relaxIncomingEdges(const StopId stop,
                                 const ProfileElement& element) noexcept {
    for (const Edge edge : reverseTransferGraph.edgesFrom(stop)) {
      profiler.countMetric(METRIC_EDGES);
      const StopId fromStop = StopId(reverseTransferGraph.get(ToVertex, edge));
      const int newDepartureTime = element.getDepartureTime() -
                                   reverseTransferGraph.get(TravelTime, edge);

      Profile& fromProfile = profiles[fromStop];
      if (fromProfile.isDominated(newDepartureTime, element.arrivalTimes))
        continue;

      profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
      fromProfile.incorporate(
          ProfileElement(newDepartureTime, element.arrivalTimes, element.legs));
    }
  }

 private:
  Data& data;
  TransferGraph reverseTransferGraph;

  StopId targetStop;

  std::vector<TripLabel> tripLabel;
  std::vector<Profile> profiles;
  std::vector<int> distanceToTarget;

  Profiler profiler;
};

}  // namespace CSA
