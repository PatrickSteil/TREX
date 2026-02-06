#pragma once

#include "../../../Helpers/IO/Serialization.h"
#include "../../../Helpers/Types.h"
#include "Trip.h"

#include <algorithm>
#include <compare>
#include <vector>

namespace TimeTable {

class Route {
public:
  RouteId routeId;
  std::vector<Trip> trips;

  explicit Route(const RouteId tripId, std::vector<Trip> trips) noexcept
      : routeId(routeId), trips(std::move(trips)){};

  Route(IO::Deserialization &deserialize) { this->deserialize(deserialize); }

  const std::vector<Trip> &getTrips() const noexcept { return trips; }

  inline void serialize(IO::Serialization &serialize) const noexcept {
    serialize(routeId, trips);
  }

  inline void deserialize(IO::Deserialization &deserialize) noexcept {
    deserialize(routeId, trips);
  }
};
} // namespace TimeTable
