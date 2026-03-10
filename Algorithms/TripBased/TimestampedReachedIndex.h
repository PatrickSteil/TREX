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

#include "../../DataStructures/TripBased/Data.h"

static constexpr uint32_t LABEL_MASK = 0xFF;
static constexpr uint32_t DEFAULT_MASK = 0xFF00;
static constexpr uint32_t TS_SHIFT = 16;

namespace TripBased {

class TimestampedReachedIndex {
public:
  TimestampedReachedIndex(const Data &data)
      : data(data), state(data.numberOfTrips()), routeEnd(data.numberOfTrips()),
        timestamp(1) {

    for (TripId trip : data.trips()) {
      uint8_t stops = data.numberOfStopsInTrip(trip);

      if (stops > 255)
        warning("Trip ", trip, " has ", stops, " stops!");

      uint32_t packed =
          (uint32_t(timestamp) << 16) | (uint32_t(stops) << 8) | stops;

      state[trip] = packed;

      routeEnd[trip] = data.firstTripOfRoute[data.routeOfTrip[trip] + 1];
    }
  }

public:
  inline void clear() noexcept {
    ++timestamp;

    if (timestamp == 0) {
      uint32_t *__restrict ptr = state.data();
      const size_t n = state.size();

      for (size_t i = 0; i < n; ++i) {
        uint32_t s = ptr[i];
        uint8_t def = (s & DEFAULT_MASK) >> 8;
        ptr[i] = (uint32_t(def) << 8) | def;
      }

      timestamp = 1;
    }
  }

  inline StopIndex operator()(const TripId trip) noexcept {
    AssertMsg(trip < state.size(), "Trip " << trip << " is out of bounds!");
    return StopIndex(getLabel(trip));
  }

  inline bool alreadyReached(TripId trip, uint8_t index) noexcept {
    uint32_t s = state[trip];

    uint8_t label = ((s >> TS_SHIFT) == timestamp) ? (s & LABEL_MASK)
                                                   : ((s >> 8) & LABEL_MASK);

    return label <= index;
  }

  inline void update(TripId trip, StopIndex index) noexcept {
    uint32_t *__restrict statePtr = state.data();
    const uint32_t end = routeEnd[trip];
    const uint8_t newValue = index;

    for (uint32_t i = trip; i < end; ++i) {
      uint32_t s = statePtr[i];

      uint16_t ts = s >> TS_SHIFT;
      uint8_t label =
          (ts == timestamp) ? (s & LABEL_MASK) : ((s >> 8) & LABEL_MASK);

      if (label <= newValue)
        break;

      uint8_t def = (s >> 8) & LABEL_MASK;

      statePtr[i] =
          (uint32_t(timestamp) << TS_SHIFT) | (uint32_t(def) << 8) | newValue;
    }
  }

private:
  inline uint8_t &getLabel(const TripId trip) noexcept {
    if (timestamps[trip] != timestamp) {
      state[trip] = defaultLabels[trip];
      timestamps[trip] = timestamp;
    }
    return state[trip];
  }

  const Data &data;

  // [ timestamp (16) | default (8) | label (8) ]
  /*
  bits 0..7   = current label
  bits 8..15  = default label
  bits 16..31 = timestamp
  */
  std::vector<uint32_t> state;
  std::vector<uint32_t> routeEnd;
  uint16_t timestamp;
};

} // namespace TripBased
