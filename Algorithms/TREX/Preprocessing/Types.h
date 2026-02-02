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
struct EdgeLabel {
  EdgeLabel(const StopEventId stopEvent = noStopEvent,
            const TripId trip = noTripId,
            const StopEventId firstEvent = noStopEvent)
      : stopEvent(stopEvent), trip(trip), firstEvent(firstEvent) {}

  StopEventId stopEvent;
  TripId trip;
  StopEventId firstEvent;
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
