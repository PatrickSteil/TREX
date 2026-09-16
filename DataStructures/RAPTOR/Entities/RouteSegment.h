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
#include "../../../Helpers/Types.h"

namespace RAPTOR {

class RouteSegment {
public:
    RouteSegment(const RouteId routeId = noRouteId, const StopIndex stopIndex = noStopIndex)
        : routeId(routeId), stopIndex(stopIndex) {}
    RouteSegment(IO::Deserialization& deserialize) { this->deserialize(deserialize); }

    friend std::ostream& operator<<(std::ostream& out, const RouteSegment& r) {
        return out << "RouteSegment{" << r.routeId << ", " << r.stopIndex << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept { serialize(routeId, stopIndex); }

    inline void deserialize(IO::Deserialization& deserialize) noexcept { deserialize(routeId, stopIndex); }

    inline bool operator<(const RouteSegment& other) const noexcept {
        return routeId < other.routeId || (routeId == other.routeId && stopIndex < other.stopIndex);
    }

public:
    RouteId routeId;
    StopIndex stopIndex;
};

}  // namespace RAPTOR
