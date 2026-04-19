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

class Agency {
public:
    Agency(const std::string& agencyId = "", const std::string& name = "", const std::string& timezone = "")
        : agencyId(agencyId), name(name), timezone(timezone) {}
    Agency(IO::Deserialization& deserialize) { this->deserialize(deserialize); }

    inline bool validate() noexcept {
        if (name.empty()) name = "NOT_NAMED";
        return !agencyId.empty();
    }

    friend std::ostream& operator<<(std::ostream& out, const Agency& a) {
        return out << "Agency{" << a.agencyId << ", " << a.name << ", " << a.timezone << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept { serialize(agencyId, name, timezone); }

    inline void deserialize(IO::Deserialization& deserialize) noexcept { deserialize(agencyId, name, timezone); }

public:
    std::string agencyId{""};
    std::string name{""};
    std::string timezone{""};
};

}  // namespace GTFS
