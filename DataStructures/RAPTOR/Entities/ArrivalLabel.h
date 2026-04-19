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

namespace RAPTOR {

class ArrivalLabel {
 public:
  ArrivalLabel(const int arrivalTime = never, const size_t numberOfTrips = -1)
      : arrivalTime(arrivalTime), numberOfTrips(numberOfTrips) {}

  inline int travelTime(const int departureTime) const noexcept {
    return arrivalTime - departureTime;
  }

  inline bool operator<(const ArrivalLabel &other) const noexcept {
    return (arrivalTime < other.arrivalTime) ||
           ((arrivalTime == other.arrivalTime) &&
            (numberOfTrips < other.numberOfTrips));
  }

  inline bool operator==(const ArrivalLabel &other) const noexcept {
    return (arrivalTime == other.arrivalTime) &&
           (numberOfTrips == other.numberOfTrips);
  }

  inline bool operator!=(const ArrivalLabel &other) const noexcept {
    return !(*this == other);
  }

  inline bool dominates(const ArrivalLabel &other) const noexcept {
    return arrivalTime <= other.arrivalTime &&
           numberOfTrips <= other.numberOfTrips;
  }

  inline friend std::ostream &operator<<(std::ostream &out,
                                         const ArrivalLabel &label) noexcept {
    return out << "arrivalTime: " << label.arrivalTime
               << ", numberOfTrips: " << label.numberOfTrips;
  }

 public:
  int arrivalTime;
  size_t numberOfTrips;
};

}  // namespace RAPTOR
