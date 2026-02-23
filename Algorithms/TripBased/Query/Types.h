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
        const StopEventId firstEvent = noStopEvent)
  : stopEvent(stopEvent), trip(trip), firstEvent(firstEvent) {}

StopEventId stopEvent;
TripId trip;
StopEventId firstEvent;

StopEventId getStopEvent() const { return stopEvent; }
TripId getTrip() const { return trip; }
StopEventId getFirstEvent() const { return firstEvent; }

void setStopEvent(StopEventId id) { stopEvent = id; }
void setTrip(TripId id) { trip = id; }
void setFirstEvent(StopEventId id) { firstEvent = id; }
};
*/

struct EdgeLabel {
  uint64_t data;

  EdgeLabel(StopIndex stopIndex = noStopIndex, TripId trip = noTripId,
            StopEventId firstEvent = noStopEvent)
      : data(0) {
    setTrip(trip);
    setFirstEvent(firstEvent);
    setStopIndex(stopIndex);
  }

  StopIndex getStopIndex() const {
    return static_cast<StopIndex>(data & 0xFFULL);
  }
  void setStopIndex(StopIndex d) { data = (data & ~0xFFULL) | d; }

  TripId getTrip() const {
    return static_cast<TripId>((data >> 8) & 0x7FFFFFULL);
  }

  void setTrip(TripId id) {
    assert(id < (1u << 23) || id == noTripId);
    data = (data & ~(0x7FFFFFULL << 8)) |
           (static_cast<uint64_t>(id & 0x7FFFFF) << 8);
  }

  StopEventId getFirstEvent() const {
    return static_cast<StopEventId>((data >> 31) & 0x7FFFFFFULL);
  }

  void setFirstEvent(StopEventId id) {
    assert(id < (1u << 27) || id == noStopEvent);
    data = (data & ~(0x7FFFFFFULL << 31)) |
           (static_cast<uint64_t>(id & 0x7FFFFFFULL) << 31);
  }

  StopEventId getStopEvent() const {
    return StopEventId(getFirstEvent() + getStopIndex());
  }

  uint8_t getRank() const {
    return static_cast<uint8_t>((data >> 58) & 0x1FULL);
  }

  void setRank(uint8_t value) {
    assert(value <= 16);
    data =
        (data & ~(0x1FULL << 58)) | (static_cast<uint64_t>(value & 0x1F) << 58);
  }
};

static_assert(sizeof(EdgeLabel) == 8, "EdgeLabel must be 8 bytes");
static_assert(alignof(EdgeLabel) == alignof(uint64_t), "Unexpected alignment");

struct EdgeLabelCellId {
  uint64_t data;
  uint16_t cellId;

  EdgeLabelCellId(StopIndex stopIndex = noStopIndex, TripId trip = noTripId,
                  StopEventId firstEvent = noStopEvent, uint16_t cell = 0)
      : data(0), cellId(cell) {
    setTrip(trip);
    setFirstEvent(firstEvent);
    setStopIndex(stopIndex);
  }

  StopIndex getStopIndex() const {
    return static_cast<StopIndex>(data & 0xFFULL);
  }
  void setStopIndex(StopIndex d) { data = (data & ~0xFFULL) | d; }

  TripId getTrip() const {
    return static_cast<TripId>((data >> 8) & 0x7FFFFFULL);
  }
  void setTrip(TripId id) {
    assert(id < (1u << 23) || id == noTripId);
    data = (data & ~(0x7FFFFFULL << 8)) |
           (static_cast<uint64_t>(id & 0x7FFFFF) << 8);
  }

  StopEventId getFirstEvent() const {
    return static_cast<StopEventId>((data >> 31) & 0x7FFFFFFULL);
  }
  void setFirstEvent(StopEventId id) {
    assert(id < (1u << 27) || id == noStopEvent);
    data = (data & ~(0x7FFFFFFULL << 31)) |
           (static_cast<uint64_t>(id & 0x7FFFFFFULL) << 31);
  }

  StopEventId getStopEvent() const {
    return StopEventId(getFirstEvent() + getStopIndex());
  }

  uint8_t getRank() const {
    return static_cast<uint8_t>((data >> 58) & 0x1FULL);
  }

  void setRank(uint8_t value) {
    assert(value <= 16);
    data =
        (data & ~(0x1FULL << 58)) | (static_cast<uint64_t>(value & 0x1F) << 58);
  }

  uint16_t getCellId() const { return cellId; }
  void setCellId(uint16_t id) { cellId = id; }
};

struct RouteLabel {
  RouteLabel() : numberOfTrips(0) {}
  inline StopIndex end() const noexcept {
    return StopIndex(departureTimes.size() / numberOfTrips);
  }
  u_int32_t numberOfTrips;
  std::vector<int> departureTimes;
};

} // namespace TripBased
