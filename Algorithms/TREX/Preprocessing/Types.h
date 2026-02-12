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
        const StopEventId firstEvent = noStopEvent,
        const uint16_t cellId = 0)
  : stopEvent(stopEvent), trip(trip), firstEvent(firstEvent),
    cellId(cellId) {}

StopEventId stopEvent;
TripId trip;
StopEventId firstEvent;
uint16_t cellId;

StopEventId getStopEvent() const { return stopEvent; }
TripId getTrip() const { return trip; }
StopEventId getFirstEvent() const { return firstEvent; }
uint16_t getCellId() const { return cellId; }

void setStopEvent(StopEventId id) { stopEvent = id; }
void setTrip(TripId id) { trip = id; }
void setCellId(uint16_t id) { cellId = id; }
void setFirstEvent(StopEventId id) { firstEvent = id; }
};

*/
#pragma pack(push, 1)
struct alignas(4) EdgeLabel {
  uint64_t lo; // 64-bit lower part
  uint32_t hi; // 32-bit upper part

  EdgeLabel(StopEventId stopEvent = noStopEvent, TripId trip = noTripId,
            StopEventId firstEvent = noStopEvent, uint16_t cellId = 0) {
    setStopEvent(stopEvent);
    setTrip(trip);
    setCellId(cellId);
    setFirstEvent(firstEvent);
  }

  StopEventId getStopEvent() const {
    return static_cast<StopEventId>(lo & 0x7FFFFFF); // mask 27 bits
  }
  void setStopEvent(StopEventId id) {
    assert(id < (1u << 27) || id == noStopEvent);

    lo = (lo & ~0x7FFFFFFULL) | (id & 0x7FFFFFFULL);
  }

  TripId getTrip() const {
    return static_cast<TripId>((lo >> 27) & 0x7FFFFF); // mask 23 bits
  }
  void setTrip(TripId id) {
    assert(id < (1u << 23) || id == noTripId);
    lo = (lo & ~(0x7FFFFFULL << 27)) |
         (static_cast<uint64_t>(id & 0x7FFFFF) << 27);
  }

  uint16_t getCellId() const {
    uint16_t lower = static_cast<uint16_t>((lo >> 50) & 0x3FFF); // 14 bits
    uint16_t upper = static_cast<uint16_t>(hi & 0x3);            // 2 bits
    return (upper << 14) | lower;
  }
  void setCellId(uint16_t id) {
    lo = (lo & ~(0x3FFFULL << 50)) | (static_cast<uint64_t>(id & 0x3FFF) << 50);
    hi = (hi & ~0x3) | ((id >> 14) & 0x3);
  }

  StopEventId getFirstEvent() const {
    return static_cast<StopEventId>((hi >> 2) & 0x7FFFFFF); // mask 27 bits
  }
  void setFirstEvent(StopEventId id) {
    assert(id < (1u << 27) || id == noStopEvent);
    hi = (hi & ~0x7FFFFFFC) | ((id & 0x7FFFFFF) << 2);
  }
};
#pragma pack(pop)

static_assert(sizeof(EdgeLabel) == 12, "EdgeLabel should be 12 bytes");
static_assert(alignof(EdgeLabel) == 4, "Alignment should be 4 bytes");

struct RouteLabel {
  RouteLabel() : numberOfTrips(0) {}
  inline StopIndex end() const noexcept {
    return StopIndex(departureTimes.size() / numberOfTrips);
  }
  u_int32_t numberOfTrips;
  std::vector<int> departureTimes;
};

} // namespace TripBased
