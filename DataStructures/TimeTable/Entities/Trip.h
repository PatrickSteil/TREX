#pragma once

#include "../../../Helpers/IO/Serialization.h"
#include "../../../Helpers/Types.h"

#include <algorithm>
#include <compare>
#include <vector>

namespace TimeTable {

class StopEvent {
public:
  StopId stop;
  Time arrivalTime;
  Time departureTime;
  int arrivalDelay;
  int departureDelay;

  StopEvent(const StopId stop = noStop, const Time arrivalTime = noTime,
            const Time departureTime = noTime, const int arrivalDelay = 0,
            const int departureDelay = 0)
      : stop(stop), arrivalTime(arrivalTime), departureTime(departureTime),
        arrivalDelay(arrivalDelay), departureDelay(departureDelay){};

  StopEvent(IO::Deserialization &deserialize) {
    this->deserialize(deserialize);
  }

  inline void serialize(IO::Serialization &serialize) const noexcept {
    serialize(stop, arrivalTime, departureTime, arrivalDelay, departureDelay);
  }

  inline void deserialize(IO::Deserialization &deserialize) noexcept {
    deserialize(stop, arrivalTime, departureTime, arrivalDelay, departureDelay);
  }

  void applyDelay(int newArrivalDelay, int newDepartureDelay) {
    arrivalDelay = newArrivalDelay;
    departureDelay = newDepartureDelay;
  }

  Time getLiveArrivalTime() const noexcept {
    return Time(arrivalTime + arrivalDelay);
  }

  Time getLiveDepartureTime() const noexcept {
    return Time(departureTime + departureDelay);
  }
};

enum TripStatus { SCHEDULED, ADDED, CANCELLED };

class Trip {
public:
  TripId tripId;
  std::vector<StopEvent> stopEvents;
  TripStatus status;

  Trip() {}
  Trip(const TripId tripId, std::vector<StopEvent> stopEvents,
       const TripStatus status = TripStatus::SCHEDULED) noexcept
      : tripId(tripId), stopEvents(std::move(stopEvents)), status(status){};

  Trip(IO::Deserialization &deserialize) { this->deserialize(deserialize); }

  inline void serialize(IO::Serialization &serialize) const noexcept {
    serialize(tripId, stopEvents);
  }

  inline void deserialize(IO::Deserialization &deserialize) noexcept {
    deserialize(tripId, stopEvents);
  }

  void applyDelay(int delay) {
    for (auto &event : stopEvents) {
      event.applyDelay(delay, delay);
    }
  }

  void setTripStatus(const TripStatus newStatus) { status = newStatus; }

  bool isCancelled() const { return status == TripStatus::CANCELLED; }
  bool isAdded() const { return status == TripStatus::ADDED; }

  const std::size_t numberOfEvents() const noexcept {
    return stopEvents.size();
  }

  const std::vector<StopEvent> &getStopEvents() const noexcept {
    return stopEvents;
  }

  std::strong_ordering operator<=>(const Trip &other) const noexcept {
    const auto n = std::min(stopEvents.size(), other.stopEvents.size());

    for (std::size_t i = 0; i < n; ++i) {
      if (auto c = stopEvents[i].stop <=> other.stopEvents[i].stop; c != 0)
        return c;
    }

    for (std::size_t i = 0; i < n; ++i) {
      if (auto c =
              stopEvents[i].arrivalTime <=> other.stopEvents[i].arrivalTime;
          c != 0)
        return c;
    }

    return stopEvents.size() <=> other.stopEvents.size();
  }

  bool operator==(const Trip &) const = default;
};
} // namespace TimeTable
