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
#include "../../../Helpers/String/String.h"

namespace GTFS {

class Frequency {
public:
    Frequency(const std::string& tripId = "", const int startTime = -1, const int endTime = -2,
              const int headwaySecs = 0, const bool exactTimes = true)
        : tripId(tripId), startTime(startTime), endTime(endTime), headwaySecs(headwaySecs), exactTimes(exactTimes) {}
    Frequency(const std::string& tripId, const std::string& startTime, const std::string& endTime,
              const int headwaySecs = 0, const bool exactTimes = true)
        : tripId(tripId),
          startTime(String::parseSeconds(startTime)),
          endTime(String::parseSeconds(endTime)),
          headwaySecs(headwaySecs),
          exactTimes(exactTimes) {}
    Frequency(IO::Deserialization& deserialize) { this->deserialize(deserialize); }

    inline bool validate() noexcept { return (!tripId.empty()) && (startTime <= endTime) && (headwaySecs > 0); }

    friend std::ostream& operator<<(std::ostream& out, const Frequency& f) {
        return out << "Frequency{" << f.tripId << ", " << f.startTime << ", " << f.endTime << ", " << f.headwaySecs
                   << ", " << f.exactTimes << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(tripId, startTime, endTime, headwaySecs, exactTimes);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(tripId, startTime, endTime, headwaySecs, exactTimes);
    }

public:
    std::string tripId{""};
    int startTime{-1};
    int endTime{-2};
    int headwaySecs{0};
    bool exactTimes{true};
};

}  // namespace GTFS
