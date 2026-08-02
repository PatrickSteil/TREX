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
#include <iostream>
#include <string>
#include <vector>

#include "../../DataStructures/CSA/Data.h"
#include "../../Helpers/Assert.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/Types.h"
#include "Profiler.h"
#include "TimeShift.h"

namespace CSA {

template <bool TRANSFERS_SECOND_CRIT = true, typename PROFILER = NoProfiler,
          bool PATH_RETRIEVAL = true>
class ProfileCSA {
 public:
  constexpr static bool TransfersSecondCrit = TRANSFERS_SECOND_CRIT;
  constexpr static bool PathRetrieval = PATH_RETRIEVAL;
  using Profiler = PROFILER;
  using Type = ProfileCSA<TransfersSecondCrit, Profiler, PathRetrieval>;
  using TripFlag = bool;

 private:
  struct Leg {
    Leg(const ConnectionId enter = ConnectionId(-1),
        const ConnectionId exit = ConnectionId(-1))
        : enter(enter), exit(exit) {}

    ConnectionId enter;
    ConnectionId exit;
  };

  struct TripLabel {
    bool reached = false;
    int arrivalTime = never;
    ConnectionId exit = ConnectionId(-1);
  };

  struct StopLabel {
    int32_t arrivalTime;
    int32_t minTransferTime;
  };

  struct TripArrivalElement {
    TripArrivalElement(int arrivalTime = never,
                       ConnectionId exit = ConnectionId(-1))
        : arrivalTime(arrivalTime), exit(exit) {}

    int arrivalTime;
    ConnectionId exit;
  };

  struct ProfileElement {
    ProfileElement(int departureTime = never, int arrivalTime = never,
                   ConnectionId enter = ConnectionId(-1),
                   ConnectionId exit = ConnectionId(-1))
        : arrivalTime(arrivalTime),
          departureTime(departureTime),
          enter(enter),
          exit(exit) {}

    inline bool dominates(const ProfileElement& other) const {
      return dominates(other.getDepartureTime(), other.getArrivalTime());
    }

    inline bool dominates(int newDepartureTime, int newArrivalTime) const {
      return departureTime >= newDepartureTime && arrivalTime <= newArrivalTime;
    }

    inline int getArrivalTime() const { return arrivalTime; }
    inline int getDepartureTime() const { return departureTime; }
    inline int getEnter() const { return enter; }
    inline int getExit() const { return exit; }

    int arrivalTime;
    int departureTime;
    ConnectionId enter;
    ConnectionId exit;
  };

  struct Profile {
    Profile() : elements(1, ProfileElement()) { elements.reserve(64); };

    inline int size() const { return elements.size(); }

    inline bool isDominated(const ProfileElement& newProfileElement) const {
      const int i = findIndex(newProfileElement);
      return elements[i].getArrivalTime() <= newProfileElement.getArrivalTime();
    }

    inline void incorporate(ProfileElement newProfileElement) {
      int i = findIndex(newProfileElement);
      auto iter = elements.begin() + i + 1;

      iter = elements.insert(iter, newProfileElement);

      elements.erase(std::remove_if(std::next(iter), elements.end(),
                                    [&](auto& other) {
                                      return newProfileElement.dominates(other);
                                    }),
                     elements.end());
    }

    inline int findIndex(
        const ProfileElement& newProfileElement) const noexcept {
      int i(size() - 1);
      while (elements[i].getDepartureTime() <
             newProfileElement.getDepartureTime())
        --i;
      return i;
    }

    void printElements() const {
      if (TransfersSecondCrit) {
        for (auto e : elements)
          std::cout
              << String::secToString(getExactArrivalTime(e.getDepartureTime()))
              << "\t"
              << String::secToString(getExactArrivalTime(e.getArrivalTime()))
              << " @ " << getNumberOfTransfers(e.getArrivalTime()) << " [ "
              << e.enter << " -> " << e.exit << " ]\n";
      } else {
        for (auto e : elements)
          std::cout << String::secToString(e.getDepartureTime()) << "\t"
                    << String::secToString(e.getArrivalTime()) << " [ "
                    << e.enter << " : " << e.exit << "]\n";
      }
    }

    std::vector<ProfileElement> elements;
  };

 public:
  ProfileCSA(Data& data, const Profiler& profilerTemplate = Profiler())
      : data(data),
        reverseTransferGraph(data.transferGraph),
        sourceStop(noStop),
        targetStop(noStop),
        tripLabel(data.numberOfTrips()),
        profiles(data.numberOfStops(), Profile()),
        distanceToTarget(data.numberOfStops(), INFTY),
        stopLabel(data.numberOfStops()),
        sourceDominationIndex(0),
        profiler(profilerTemplate) {
    reverseTransferGraph.revert();
    if (TransfersSecondCrit) {
      for (auto& connection : data.connections) {
        connection.departureTime = shiftTime(connection.departureTime);
        connection.arrivalTime = shiftTime(connection.arrivalTime);
      }
    }
    AssertMsg(Vector::isSorted(data.connections),
              "Connections must be sorted in ascending order!");
    for (const StopId stop : data.stops()) {
      stopLabel[stop].minTransferTime = data.minTransferTime(stop);
    }
    profiler.registerPhases({PHASE_CLEAR, PHASE_INITIALIZATION,
                             PHASE_CONNECTION_SCAN, PHASE_REACHABLE_EA_QUERY});
    profiler.registerMetrics({METRIC_CONNECTIONS, METRIC_EDGES,
                              METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
    profiler.initialize();
  }

  inline void run(const StopId source, const StopId target,
                  int minDepartureTime = 0,
                  int maxDepartureTime = 86400) noexcept {
    AssertMsg(data.isStop(source),
              "Source stop " << source << " is not a valid stop!");
    AssertMsg(data.isStop(target),
              "Target stop " << target << " is not a valid stop!");
    AssertMsg(minDepartureTime >= 0 && minDepartureTime < 24 * 60 * 60,
              "Min Departure Time is not in the first day!");
    AssertMsg(maxDepartureTime > 0 && maxDepartureTime <= 24 * 60 * 60,
              "Max Departure Time is not in the first day!");
    AssertMsg(minDepartureTime < maxDepartureTime,
              "Min Departure Time should be smaller than Max Departure Time!");

    profiler.start();

    minDepartureTime = transformTime(minDepartureTime);
    maxDepartureTime = transformTime(maxDepartureTime);
    profiler.startPhase();
    resetDistancesToTarget(target);
    clear();
    profiler.donePhase(PHASE_CLEAR);

    profiler.startPhase();
    sourceStop = source;
    targetStop = target;
    ConnectionId earliestConnection(0);
    ConnectionId latestConnection(data.connections.size());

    earliestConnection = firstReachableConnection(minDepartureTime);
    latestConnection = firstReachableConnection(maxDepartureTime + 1);
    profiler.donePhase(PHASE_INITIALIZATION);

    profiler.startPhase();
    runOneEAQuery(earliestConnection, latestConnection, minDepartureTime);
    profiler.donePhase(PHASE_REACHABLE_EA_QUERY);

    profiler.startPhase();
    scanConnections(earliestConnection, latestConnection);
    profiler.donePhase(PHASE_CONNECTION_SCAN);

    profiler.done();
  }

  inline std::vector<Leg> getUsedConnections(StopId stop) {
    std::vector<Leg> journey;

    if (!reachable(stop)) return journey;
    if (distanceToTarget[stop] < INFTY) [[unlikely]]
      return journey;

    const ProfileElement& element = profiles[stop].elements.back();
    const int arrivalTimeAtTarget = getExactArrivalTime(element.arrivalTime);

    journey.push_back({element.enter, element.exit});
    stop = data.connections[element.exit].arrivalStopId;

    while (distanceToTarget[stop] == INFTY) {
      const Profile& profile = profiles[stop];

      int i(0);
      while (i < profile.size() &&
             getExactArrivalTime(profile.elements[i].arrivalTime) !=
                 arrivalTimeAtTarget)
        ++i;
      Assert(0 <= i && i < profile.size());
      const ProfileElement& e = profile.elements[i];

      journey.push_back({e.enter, e.exit});
      stop = data.connections[e.exit].arrivalStopId;
    }

    return journey;
  }

  inline bool reachable(const StopId stop) const noexcept {
    if (TransfersSecondCrit)
      return getExactArrivalTime(profiles[stop].elements.back().arrivalTime) <
             never;
    return profiles[stop].elements.back().arrivalTime < never;
  }

  inline const Profiler& getProfiler() const noexcept { return profiler; }

  void printProfile(const StopId stop) const { profiles[stop].printElements(); }

  int numberOfJourneys(const StopId stop) { return profiles[stop].size() - 1; }

 private:
  inline void resetDistancesToTarget(const StopId newTarget) {
    if (targetStop != noStop) {
      distanceToTarget[targetStop] = INFTY;
      for (auto edge : reverseTransferGraph.edgesFrom(targetStop)) {
        distanceToTarget[reverseTransferGraph.get(ToVertex, edge)] = INFTY;
      }
    }

    distanceToTarget[newTarget] = 0;
    for (auto edge : reverseTransferGraph.edgesFrom(newTarget)) {
      distanceToTarget[reverseTransferGraph.get(ToVertex, edge)] =
          transformTime(reverseTransferGraph.get(TravelTime, edge));
    }
  }

  inline void clear() {
    sourceStop = noStop;
    targetStop = noStop;
    Vector::fill(profiles, Profile());
    for (TripLabel& label : tripLabel) label = TripLabel();
    for (StopLabel& label : stopLabel) label.arrivalTime = never;
    sourceDominationIndex = 0;
  }

  inline ConnectionId firstReachableConnection(
      const int departureTime) const noexcept {
    return ConnectionId(
        Vector::lowerBound(data.connections, departureTime,
                           [](const Connection& connection, const int time) {
                             return connection.departureTime < time;
                           }));
  }

  inline void runOneEAQuery(const ConnectionId earliestConnection,
                            const ConnectionId latestConnection,
                            const int minDepartureTime) noexcept {
    stopLabel[sourceStop].arrivalTime = minDepartureTime;
    relaxOutgoingEdges(sourceStop, minDepartureTime);

    for (ConnectionId i(earliestConnection); i < latestConnection; ++i) {
      const Connection& connection = data.connections[i];
      if (i + PrefetchDistance < latestConnection) {
        const Connection& upcoming = data.connections[i + PrefetchDistance];
        // Locality hint 3: consumed within the next PrefetchDistance
        // iterations of a tight loop, so it should land and stay in L1.
        __builtin_prefetch(&stopLabel[upcoming.departureStopId], 0, 3);
      }
      if (connectionIsReachable(connection)) {
        arrivalByTrip(connection.arrivalStopId, connection.arrivalTime);
      }
    }
  }

  static constexpr size_t PrefetchDistance = 4;

  inline void scanConnections(const ConnectionId earliestConnection,
                              const ConnectionId latestConnection) noexcept {
    AssertMsg(earliestConnection < data.connections.size(),
              "earliestConnection out of bounds!\n");
    AssertMsg(latestConnection - 1 < data.connections.size(),
              "latestConnection out of bounds!\n");

    for (ConnectionId i(latestConnection); i > earliestConnection; --i) {
      const Connection& connection = data.connections[i - 1];

      prefetchUpcomingConnection(i, earliestConnection);

      if (!tripLabel[connection.tripId].reached) continue;

      const int tau1 =
          connection.arrivalTime + distanceToTarget[connection.arrivalStopId];
      const int tau2 = tripLabel[connection.tripId].arrivalTime;
      const int tau3 = earliestArrivalTimeInProfiles(connection.arrivalStopId,
                                                     connection.arrivalTime) +
                       (TransfersSecondCrit ? offset : 0);

      const int tauC = std::min(tau1, std::min(tau2, tau3));

      if (tauC == never) [[unlikely]]
        continue;

      if (tauC < tripLabel[connection.tripId].arrivalTime) {
        tripLabel[connection.tripId].arrivalTime = tauC;
        tripLabel[connection.tripId].exit = i;
      }

      profiler.countMetric(METRIC_CONNECTIONS);
      Assert(tripLabel[connection.tripId].exit != ConnectionId(-1));
      const ProfileElement currentProfile(connection.departureTime, tauC, i,
                                          tripLabel[connection.tripId].exit);
      Profile& profileArrivalStop = profiles[connection.arrivalStopId];

      if (!profileArrivalStop.isDominated(currentProfile) &&
          checkSourceDomination(currentProfile)) {
        profiler.countMetric(METRIC_STOPS_BY_TRIP);

        profileArrivalStop.incorporate(currentProfile);
        relaxIncommingEdges(connection.departureStopId, currentProfile);
      }
    }
  }

  inline void prefetchUpcomingConnection(
      const ConnectionId i, const ConnectionId earliestConnection) noexcept {
    if (i < earliestConnection + PrefetchDistance + 1) return;
    const ConnectionId prefetchIndex = ConnectionId(i - 1 - PrefetchDistance);
    const Connection& upcoming = data.connections[prefetchIndex];
    if (!tripLabel[upcoming.tripId].reached) return;
    __builtin_prefetch(&tripLabel[upcoming.tripId], 1, 3);
    __builtin_prefetch(&profiles[upcoming.arrivalStopId], 1, 1);
  }

  inline bool checkSourceDomination(
      const ProfileElement currentProfile) noexcept {
    Profile& sourceProfile = profiles[sourceStop];

    while (sourceDominationIndex < (int)sourceProfile.size() - 1 &&
           sourceProfile.elements[sourceDominationIndex].getDepartureTime() >=
               currentProfile.getDepartureTime())
      ++sourceDominationIndex;
    return sourceProfile.elements[sourceDominationIndex].getArrivalTime() >
           currentProfile.getArrivalTime();
  }

  inline int earliestArrivalTimeInProfiles(const StopId stop,
                                           const int arrivalTime) noexcept {
    const Profile& stopProfile = profiles[stop];

    int i(stopProfile.elements.size() - 1);

    while (stopProfile.elements[i].getDepartureTime() < arrivalTime) --i;
    return stopProfile.elements[i].getArrivalTime();
  }

  inline void relaxIncommingEdges(const StopId stop,
                                  ProfileElement currentProfile) noexcept {
    for (auto edge : reverseTransferGraph.edgesFrom(stop)) {
      profiler.countMetric(METRIC_EDGES);
      const StopId fromStop = StopId(reverseTransferGraph.get(ToVertex, edge));
      const int newDepartureTime =
          currentProfile.getDepartureTime() -
          transformTime(reverseTransferGraph.get(TravelTime, edge));
      ProfileElement newElement(newDepartureTime,
                                currentProfile.getArrivalTime(),
                                currentProfile.enter, currentProfile.exit);
      Profile& profileFromStop(profiles[fromStop]);

      if (profileFromStop.isDominated(newElement)) continue;
      profileFromStop.incorporate(newElement);
      profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
    }
  }

  inline void relaxOutgoingEdges(const StopId stop, const int time) noexcept {
    for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
      const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
      const int newArrivalTime =
          time + transformTime(data.transferGraph.get(TravelTime, edge));
      arrivalByTransfer(toStop, newArrivalTime);
    }
  }

  inline bool connectionIsReachableFromStop(
      const Connection& connection) const noexcept {
    const StopLabel& label = stopLabel[connection.departureStopId];
    return label.arrivalTime <=
           connection.departureTime - label.minTransferTime;
  }

  inline bool connectionIsReachableFromTrip(
      const Connection& connection) const noexcept {
    return tripLabel[connection.tripId].reached;
  }

  inline bool connectionIsReachable(const Connection& connection) noexcept {
    if (connectionIsReachableFromTrip(connection)) return true;
    if (connectionIsReachableFromStop(connection)) {
      tripLabel[connection.tripId].reached = true;
      return true;
    }
    return false;
  }

  inline void arrivalByTrip(const StopId stop, const int time) noexcept {
    if (stopLabel[stop].arrivalTime <= time) return;
    stopLabel[stop].arrivalTime = time;
    relaxOutgoingEdges(stop, time);
  }

  inline void arrivalByTransfer(const StopId stop, const int time) noexcept {
    if (stopLabel[stop].arrivalTime <= time) return;
    stopLabel[stop].arrivalTime = time;
  }

  inline int transformTime(int time) noexcept {
    if (!TransfersSecondCrit) return time;
    return shiftTime(time);
  }

 private:
  Data& data;

  TransferGraph reverseTransferGraph;
  StopId sourceStop;
  StopId targetStop;

  std::vector<TripLabel> tripLabel;
  std::vector<Profile> profiles;
  std::vector<int> distanceToTarget;
  std::vector<StopLabel> stopLabel;

  int sourceDominationIndex;
  Profiler profiler;
};
}  // namespace CSA
