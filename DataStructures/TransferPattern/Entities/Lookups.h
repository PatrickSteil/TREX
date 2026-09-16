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

#include <vector>

#include "../../RAPTOR/Entities/RouteSegment.h"
#include "../../RAPTOR/Entities/StopEvent.h"

namespace TransferPattern {
struct HaltsOfStopInLine {
    HaltsOfStopInLine(std::vector<RAPTOR::StopEvent> halts = {}) : halts(halts) {}

    inline void serialize(IO::Serialization& serialize) const { serialize(halts); }

    inline void deserialize(IO::Deserialization& deserialize) { deserialize(halts); }

    std::vector<RAPTOR::StopEvent> halts;
};

struct LookupOfLine {
    LookupOfLine(std::vector<HaltsOfStopInLine> stopsAlongLine = {}) : stopsAlongLine(stopsAlongLine) {}

    inline void serialize(IO::Serialization& serialize) const { serialize(stopsAlongLine); }

    inline void deserialize(IO::Deserialization& deserialize) { deserialize(stopsAlongLine); }

    std::vector<HaltsOfStopInLine> stopsAlongLine;
};

struct LineAndStopIndex {
    LineAndStopIndex(RouteId routeId = noRouteId, StopIndex stopIndex = noStopIndex)
        : routeId(routeId), stopIndex(stopIndex) {}

    inline void serialize(IO::Serialization& serialize) { serialize(routeId, stopIndex); }

    inline void deserialize(IO::Deserialization& deserialize) { deserialize(routeId, stopIndex); }

    RouteId routeId;
    StopIndex stopIndex;

    bool operator<(const LineAndStopIndex& a) const {
        return std::tie(routeId, stopIndex) < std::tie(a.routeId, a.stopIndex);
    }

    bool beforeOnSameLine(const LineAndStopIndex& a) const { return routeId == a.routeId && stopIndex < a.stopIndex; }
};

struct StopLookup {
    StopLookup(std::vector<RAPTOR::RouteSegment> incidentLines = {}) : incidentLines(incidentLines) {}

    inline void serialize(IO::Serialization& serialize) const { serialize(incidentLines); }

    inline void deserialize(IO::Deserialization& deserialize) { deserialize(incidentLines); }

    std::vector<RAPTOR::RouteSegment> incidentLines;
};
}  // namespace TransferPattern
