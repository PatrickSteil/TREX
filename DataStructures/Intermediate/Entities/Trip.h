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

#include "../../../Helpers/IO/Serialization.h"
#include "../../../Helpers/Types.h"
#include "StopEvent.h"

namespace Intermediate {

class Trip {
 public:
  static const std::string CSV_HEADER;

 public:
  Trip(const std::string& tripName = "", const std::string& routeName = "",
       const int type = -1, const std::string& routeShortName = "")
      : tripName(tripName),
        routeName(routeName),
        type(type),
        routeShortName(routeShortName) {}
  template <typename TRIP_TYPE>
  Trip(const TRIP_TYPE& t)
      : tripName(t.tripName), routeName(t.routeName), type(t.type) {}
  Trip(const Trip& t, const int timeOffset)
      : tripName(t.tripName),
        routeName(t.routeName),
        type(t.type),
        routeShortName(t.routeShortName) {
    for (const StopEvent se : t.stopEvents) {
      stopEvents.emplace_back(se, timeOffset);
    }
  }
  Trip(IO::Deserialization& deserialize) { this->deserialize(deserialize); }

  inline bool operator<(const Trip& t) const noexcept {
    if (less(stopEvents, t.stopEvents)) return true;
    if (!equals(stopEvents, t.stopEvents)) return false;
    for (size_t i = 0; i < stopEvents.size(); i++) {
      if (stopEvents[i].departureTime < t.stopEvents[i].departureTime)
        return true;
      if (stopEvents[i].departureTime > t.stopEvents[i].departureTime)
        return false;
      if (stopEvents[i].arrivalTime > t.stopEvents[i].arrivalTime) return true;
      if (stopEvents[i].arrivalTime < t.stopEvents[i].arrivalTime) return false;
    }
    return false;
  }

  inline bool operator==(const Trip& t) const noexcept {
    if (!equals(stopEvents, t.stopEvents)) return false;
    for (size_t i = 0; i < stopEvents.size(); i++) {
      if (stopEvents[i].departureTime != t.stopEvents[i].departureTime)
        return false;
      if (stopEvents[i].arrivalTime != t.stopEvents[i].arrivalTime)
        return false;
    }
    return true;
  }

  inline bool dominates(const Trip& t) const noexcept {
    if (!equals(stopEvents, t.stopEvents)) return false;
    for (size_t i = 0; i < stopEvents.size(); i++) {
      if (stopEvents[i].departureTime < t.stopEvents[i].departureTime)
        return false;
      if (stopEvents[i].arrivalTime > t.stopEvents[i].arrivalTime) return false;
    }
    return true;
  }

  friend std::ostream& operator<<(std::ostream& out, const Trip& t) {
    return out << "Trip{" << t.routeName << " (" << t.routeShortName << "), "
               << t.tripName << ", " << t.type << ", " << t.stopEvents.size()
               << "}";
  }

  inline void serialize(IO::Serialization& serialize) const noexcept {
    serialize(stopEvents, tripName, routeName, type, routeShortName);
  }

  inline void deserialize(IO::Deserialization& deserialize) noexcept {
    deserialize(stopEvents, tripName, routeName, type, routeShortName);
  }

  inline std::ostream& toCSV(std::ostream& out) const {
    return out << "\"" << tripName << "\",\"" << routeName << "\",\""
               << routeShortName << "\"," << type;
  }

  inline std::string toCSV() const {
    std::stringstream ss;
    toCSV(ss);
    return ss.str();
  }

  inline bool matches(const Trip& other) const noexcept {
    if (stopEvents.size() != other.stopEvents.size()) return false;
    for (size_t i = 0; i < stopEvents.size(); i++) {
      if (!stopEvents[i].matches(other.stopEvents[i])) return false;
    }
    return true;
  }

  inline void update(const Trip& other) noexcept {
    AssertMsg(matches(other), "Updating a trip with one that does not match!");
    for (size_t i = 0; i < stopEvents.size(); i++) {
      stopEvents[i].update(other.stopEvents[i]);
    }
    tripName = other.tripName;
    routeName = other.routeName;
    type = other.type;
    routeShortName = other.routeShortName;
  }

  inline void shiftInTime(const int amount) noexcept {
    for (StopEvent& stopEvent : stopEvents) {
      stopEvent.departureTime += amount;
      stopEvent.arrivalTime += amount;
    }
  }

  inline void applyStopPermutation(const Permutation& permutation) noexcept {
    for (StopEvent& stopEvent : stopEvents) {
      stopEvent.applyStopPermutation(permutation);
    }
  }

 public:
  std::vector<StopEvent> stopEvents{};
  std::string tripName{""};
  std::string routeName{""};
  int type{-1};
  std::string routeShortName{""};
};

const std::string Trip::CSV_HEADER = "name,route,routeShortName,vehicle";

inline bool sameRouteAttributes(const Trip& a, const Trip& b) noexcept {
  return a.routeName == b.routeName && a.type == b.type;
}

inline bool isFiFo(const Trip& a, const Trip& b) noexcept {
  AssertMsg(a.stopEvents.size() == b.stopEvents.size(),
            "FiFO property can only be tested for trips of equal size!");
  for (size_t i = 0; i < a.stopEvents.size(); i++) {
    if (a.stopEvents[i].arrivalTime > b.stopEvents[i].arrivalTime) return false;
    if (a.stopEvents[i].departureTime > b.stopEvents[i].departureTime)
      return false;
  }
  return true;
}

inline bool isOffset(const Trip& a, const Trip& b) noexcept {
  AssertMsg(a.stopEvents.size() == b.stopEvents.size(),
            "Shifted property can only be tested for trips of equal size!");
  if (a.stopEvents.size() == 0) return true;
  int offset =
      a.stopEvents.front().departureTime - b.stopEvents.front().departureTime;
  if (a.stopEvents.back().arrivalTime - b.stopEvents.back().arrivalTime !=
      offset)
    return false;
  for (size_t i = 1; i < a.stopEvents.size() - 1; i++) {
    if (a.stopEvents[i].arrivalTime - b.stopEvents[i].arrivalTime != offset)
      return false;
    if (a.stopEvents[i].departureTime - b.stopEvents[i].departureTime != offset)
      return false;
  }
  return true;
}

}  // namespace Intermediate
