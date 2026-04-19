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

#include "../../../Helpers/Types.h"

namespace CSA {

class JourneyLeg {
public:
    JourneyLeg(const Vertex from = noVertex, const Vertex to = noVertex, const int departureTime = never,
               const int arrivalTime = never, const Edge transferId = noEdge)
        : from(from),
          to(to),
          departureTime(departureTime),
          arrivalTime(arrivalTime),
          usesTrip(false),
          transferId(transferId) {}

    JourneyLeg(const Vertex from, const Vertex to, const int departureTime, const int arrivalTime, const TripId tripId)
        : from(from), to(to), departureTime(departureTime), arrivalTime(arrivalTime), usesTrip(true), tripId(tripId) {}

    inline friend std::ostream& operator<<(std::ostream& out, const JourneyLeg& leg) noexcept {
        return out << "from: " << leg.from << ", to: " << leg.to << ", dep-Time: " << leg.departureTime
                   << ", arr-Time: " << leg.arrivalTime << (leg.usesTrip ? ", trip: " : ", transfer: ") << leg.tripId;
    }

public:
    Vertex from;
    Vertex to;
    int departureTime;
    int arrivalTime;
    bool usesTrip;
    union {
        TripId tripId;
        Edge transferId;
    };
};

using Journey = std::vector<JourneyLeg>;

inline std::vector<Vertex> journeyToPath(const Journey& journey) noexcept {
    std::vector<Vertex> path;
    for (const JourneyLeg& leg : journey) {
        path.emplace_back(leg.from);
    }
    path.emplace_back(journey.back().to);
    return path;
}

}  // namespace CSA
