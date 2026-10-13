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

namespace GTFS {

namespace Type {
constexpr int Tram = 0;
constexpr int Subway = 1;
constexpr int Rail = 2;
constexpr int Bus = 3;
constexpr int Ferry = 4;
constexpr int CableCar = 5;
constexpr int Gondola = 6;
constexpr int Funicular = 7;
constexpr int Undefined = -1;
}  // namespace Type

const std::string TypeNames[] = {"Tram", "Subway", "Rail", "Bus", "Ferry", "Cable Car", "Gondola", "Funicular"};
const std::vector<int> Types = {3, 2, 4, 5, 6, 7, 1, 0};

inline std::string typeName(const int type) noexcept {
    if (type < 0 || type > 7) return "Undefined";
    return TypeNames[type];
}

}  // namespace GTFS
