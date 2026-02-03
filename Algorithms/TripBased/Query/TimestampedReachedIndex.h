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
#include <cstdint>
#include <vector>

#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

/**
 * PackedEntry Layout (32 bits):
 * [ Timestamp (16 bits) | Default Label (8 bits) | Current Label (8 bits) ]
 */
using PackedEntry = uint32_t;

static constexpr uint32_t TS_SHIFT = 16;
static constexpr uint32_t DEF_SHIFT = 8;
static constexpr uint32_t LABEL_MASK = 0xFFu;
static constexpr uint32_t TS_MASK = 0xFFFFu;

static inline uint32_t pack(uint16_t ts, uint8_t def, uint8_t cur) noexcept {
  return (static_cast<uint32_t>(ts) << TS_SHIFT) |
         (static_cast<uint32_t>(def) << DEF_SHIFT) | static_cast<uint32_t>(cur);
}

static inline uint16_t unpackTimestamp(uint32_t v) noexcept {
  return static_cast<uint16_t>(v >> TS_SHIFT);
}

static inline uint8_t unpackDefault(uint32_t v) noexcept {
  return static_cast<uint8_t>((v >> DEF_SHIFT) & LABEL_MASK);
}

static inline uint8_t unpackCurrent(uint32_t v) noexcept {
  return static_cast<uint8_t>(v & LABEL_MASK);
}

class TimestampedReachedIndex {
public:
  TimestampedReachedIndex(const Data &data)
      : data(data), entries(data.numberOfTrips()), timestamp(0) {

    for (TripId trip : data.trips()) {
      const auto stops = data.numberOfStopsInTrip(trip);
      const uint8_t def = static_cast<uint8_t>(stops);
      entries[trip] = pack(0, def, def);
    }
  }

public:
  inline void clear() noexcept {
    timestamp = (timestamp + 1) & TS_MASK;

    if (__builtin_expect(timestamp == 0, 0)) {
      for (TripId i(0); i < entries.size(); ++i) {
        uint8_t def = unpackDefault(entries[i]);
        entries[i] = pack(0, def, def);
      }
    }
  }

  inline StopIndex operator()(const TripId trip) noexcept {
    return StopIndex(getLabel(trip));
  }

  inline bool alreadyReached(const TripId trip, const uint8_t index) noexcept {
    return getLabel(trip) <= index;
  }

  inline void update(const TripId trip, const StopIndex index) noexcept {
    const TripId routeEnd = data.firstTripOfRoute[data.routeOfTrip[trip] + 1];

    for (TripId i = trip; i < routeEnd; i++) {
      uint8_t currentLabel = getLabel(i);
      if (currentLabel <= index)
        break;

      uint8_t def = unpackDefault(entries[i]);
      entries[i] = pack(timestamp, def, static_cast<uint8_t>(index));
    }
  }

  void prefetch(const TripId trip) const noexcept {
      __builtin_prefetch(&entries[trip]);
  }

private:
  inline uint8_t getLabel(const TripId trip) noexcept {
    PackedEntry v = entries[trip];
    const uint16_t ts = unpackTimestamp(v);

    if (__builtin_expect(ts != timestamp, 0)) {
      const uint8_t def = unpackDefault(v);
      const PackedEntry newVal = pack(timestamp, def, def);
      entries[trip] = newVal;
      return def;
    }

    return unpackCurrent(v);
  }

  const Data &data;
  std::vector<PackedEntry> entries;
  uint16_t timestamp;
};

} // namespace TripBased
