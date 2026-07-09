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

#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "../../../Helpers/Calendar.h"
#include "../../../Helpers/IO/Serialization.h"

namespace GTFS {

class Calendar {
public:
    Calendar(const std::string& serviceId = "",
             const std::array<bool, 7>& operatesOnWeekday = {false, false, false, false, false, false, false},
             const int startDate = -1, const int endDate = -2)
        : serviceId(serviceId), operatesOnWeekday(operatesOnWeekday), startDate(startDate), endDate(endDate) {}
    Calendar(const std::string& serviceId, const std::array<bool, 7>& operatesOnWeekday, const std::string& startDate,
             const std::string& endDate)
        : serviceId(serviceId),
          operatesOnWeekday(operatesOnWeekday),
          startDate(stringToDay(startDate)),
          endDate(stringToDay(endDate)) {}
    Calendar(IO::Deserialization& deserialize) { this->deserialize(deserialize); }

    inline bool validate() noexcept { return (!serviceId.empty()) && (startDate <= endDate); }

    friend std::ostream& operator<<(std::ostream& out, const Calendar& c) {
        out << "Calendar{" << c.serviceId << ", ";
        for (const int day : week) out << c.operatesOnWeekday[day] << ", ";
        return out << c.startDate << ", " << c.endDate << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(serviceId, operatesOnWeekday, startDate, endDate);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(serviceId, operatesOnWeekday, startDate, endDate);
    }

public:
    std::string serviceId{""};
    std::array<bool, 7> operatesOnWeekday{{false, false, false, false, false, false, false}};
    int startDate{-1};
    int endDate{-2};
};

}  // namespace GTFS
