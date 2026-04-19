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

namespace GTFS {

class StopTime {
public:
    StopTime(const std::string& tripId = "", const int arrivalTime = -1, const int departureTime = -2,
             const std::string& stopId = "", const int stopSequence = -1)
        : tripId(tripId),
          arrivalTime(arrivalTime),
          departureTime(departureTime),
          stopId(stopId),
          stopSequence(stopSequence) {}
    StopTime(IO::Deserialization& deserialize) { this->deserialize(deserialize); }

    inline bool validate() noexcept { return (!tripId.empty()) && (!stopId.empty()) && (arrivalTime <= departureTime); }

    inline bool operator<(const StopTime& s) const noexcept {
        return (tripId < s.tripId) ||
               ((tripId == s.tripId) && ((stopSequence < s.stopSequence) ||
                                         ((stopSequence == s.stopSequence) &&
                                          ((arrivalTime < s.arrivalTime) ||
                                           ((arrivalTime == s.arrivalTime) && ((departureTime < s.departureTime)))))));
    }

    friend std::ostream& operator<<(std::ostream& out, const StopTime& s) {
        return out << "StopTime{" << s.tripId << ", " << s.arrivalTime << ", " << s.departureTime << ", " << s.stopId
                   << ", " << s.stopSequence << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(tripId, arrivalTime, departureTime, stopId, stopSequence);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(tripId, arrivalTime, departureTime, stopId, stopSequence);
    }

public:
    std::string tripId{""};
    int arrivalTime{-1};
    int departureTime{-2};
    std::string stopId{""};
    int stopSequence{-1};
};

}  // namespace GTFS
