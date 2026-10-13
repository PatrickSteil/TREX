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

class Transfer {
public:
    Transfer(const std::string& fromStopId = "", const std::string& toStopId = "", const int minTransferTime = 0)
        : fromStopId(fromStopId), toStopId(toStopId), minTransferTime(minTransferTime) {}
    Transfer(IO::Deserialization& deserialize) { this->deserialize(deserialize); }

    inline bool validate() noexcept { return (!fromStopId.empty()) && (!toStopId.empty()) && (minTransferTime >= 0); }

    friend std::ostream& operator<<(std::ostream& out, const Transfer& t) {
        return out << "Transfer{" << t.fromStopId << ", " << t.toStopId << ", " << t.minTransferTime << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(fromStopId, toStopId, minTransferTime);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(fromStopId, toStopId, minTransferTime);
    }

public:
    std::string fromStopId{""};
    std::string toStopId{""};
    int minTransferTime{0};
};

}  // namespace GTFS
