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
#include "../../Geometry/Point.h"

namespace GTFS {

class Stop {
public:
    Stop(const std::string& stopId = "", const std::string& name = "",
         const Geometry::Point& coordinates = Geometry::Point())
        : stopId(stopId), name(name), coordinates(coordinates) {}
    Stop(IO::Deserialization& deserialize) { this->deserialize(deserialize); }

    inline bool validate() noexcept {
        if (name.empty()) name = "NOT_NAMED";
        return !stopId.empty();
    }

    friend std::ostream& operator<<(std::ostream& out, const Stop& s) {
        return out << "Stop{" << s.stopId << ", " << s.name << ", " << s.coordinates << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept { serialize(stopId, name, coordinates); }

    inline void deserialize(IO::Deserialization& deserialize) noexcept { deserialize(stopId, name, coordinates); }

public:
    std::string stopId{""};
    std::string name{""};
    Geometry::Point coordinates{};
};

}  // namespace GTFS
