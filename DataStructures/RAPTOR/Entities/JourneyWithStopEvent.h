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

#include <iostream>
#include <string>
#include <vector>

#include "../../../Helpers/Types.h"

namespace RAPTOR {

class JourneyLegWithStopEvent {
public:
  JourneyLegWithStopEvent(
      const Vertex from = noVertex, const Vertex to = noVertex,
      const size_t fromStopEventId = 0, const size_t toStopEventId = 0,
      const int departureTime = never, const int arrivalTime = never,
      const bool usesRoute = false, const RouteId routeId = noRouteId)
      : from(from), to(to), fromStopEventId(fromStopEventId),
        toStopEventId(toStopEventId), departureTime(departureTime),
        arrivalTime(arrivalTime), usesRoute(usesRoute), routeId(routeId) {}

  JourneyLegWithStopEvent(const Vertex from, const Vertex to,
                          const size_t fromStopEventId,
                          const size_t toStopEventId, const int departureTime,
                          const int arrivalTime, const Edge edge)
      : from(from), to(to), fromStopEventId(fromStopEventId),
        toStopEventId(toStopEventId), departureTime(departureTime),
        arrivalTime(arrivalTime), usesRoute(false), transferId(edge) {}

  JourneyLegWithStopEvent(const Vertex from = noVertex,
                          const Vertex to = noVertex,
                          const int departureTime = never,
                          const int arrivalTime = never,
                          const bool usesRoute = false,
                          const RouteId routeId = noRouteId)
      : from(from), to(to), fromStopEventId(0), toStopEventId(0),
        departureTime(departureTime), arrivalTime(arrivalTime),
        usesRoute(usesRoute), routeId(routeId) {}

  JourneyLegWithStopEvent(const Vertex from, const Vertex to,
                          const int departureTime, const int arrivalTime,
                          const Edge edge)
      : from(from), to(to), fromStopEventId(0), toStopEventId(0),
        departureTime(departureTime), arrivalTime(arrivalTime),
        usesRoute(false), transferId(edge) {}

  inline int transferTime() const noexcept {
    return usesRoute ? 0 : arrivalTime - departureTime;
  }

  inline friend std::ostream &
  operator<<(std::ostream &out, const JourneyLegWithStopEvent &leg) noexcept {
    return out << "from: " << leg.from << ", to: " << leg.to
               << ", dep-Time: " << leg.departureTime
               << ", arr-Time: " << leg.arrivalTime
               << (leg.usesRoute ? ", route: " : ", transfer: ") << leg.routeId
               << ", fromStopEventId: " << leg.fromStopEventId
               << ", toStopEventId: " << leg.toStopEventId;
  }

public:
  Vertex from;
  Vertex to;
  size_t fromStopEventId;
  size_t toStopEventId;
  int departureTime;
  int arrivalTime;
  bool usesRoute;
  union {
    RouteId routeId;
    Edge transferId;
  };
};

using JourneyWithStopEvent = std::vector<JourneyLegWithStopEvent>;

inline std::vector<Vertex>
journeyToPath(const JourneyWithStopEvent &journey) noexcept {
  std::vector<Vertex> path;
  for (const JourneyLegWithStopEvent &leg : journey) {
    path.emplace_back(leg.from);
  }
  path.emplace_back(journey.back().to);
  return path;
}

inline int totalTransferTime(const JourneyWithStopEvent &journey) noexcept {
  int transferTime = 0;
  for (const JourneyLegWithStopEvent &leg : journey) {
    transferTime += leg.transferTime();
  }
  return transferTime;
}

inline int
intermediateTransferTime(const JourneyWithStopEvent &journey) noexcept {
  int transferTime = 0;
  for (size_t i = 1; i < journey.size() - 1; i++) {
    transferTime += journey[i].transferTime();
  }
  return transferTime;
}

inline int initialTransferTime(const JourneyWithStopEvent &journey) noexcept {
  if (journey.empty())
    return 0;
  int transferTime = journey[0].transferTime();
  if (journey.size() > 1) {
    transferTime += journey.back().transferTime();
  }
  return transferTime;
}

inline size_t countTrips(const JourneyWithStopEvent &journey) noexcept {
  size_t numTrips = 0;
  for (const JourneyLegWithStopEvent &leg : journey) {
    if (leg.usesRoute && leg.routeId != noRouteId)
      numTrips++;
  }
  return numTrips;
}

inline size_t countEvents(const JourneyWithStopEvent &journey) noexcept {
  size_t numEvents = 0;
  for (const JourneyLegWithStopEvent &leg : journey) {
    if (leg.usesRoute && leg.routeId != noRouteId) {
      AssertMsg(leg.fromStopEventId < leg.toStopEventId,
                "From event should be < then to event!");
      numEvents += (leg.toStopEventId - leg.fromStopEventId);
    }
  }
  return numEvents;
}
} // namespace RAPTOR
