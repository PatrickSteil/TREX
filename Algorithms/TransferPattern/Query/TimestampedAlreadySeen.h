/**********************************************************************************

 Copyright (c) 2023-2025 Patrick Steil

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

namespace TransferPattern {

class TimestampedAlreadySeen {
public:
    TimestampedAlreadySeen(const size_t numberOfElements = 0) : timestamps(numberOfElements, 0), currentTimestamp(0) {}

    inline void clear() {
        ++currentTimestamp;

        if (currentTimestamp == 0) {
            std::fill(timestamps.begin(), timestamps.end(), 0);
        }
    }

    inline bool contains(const size_t elementId) {
        AssertMsg(elementId < timestamps.size(), "Element Id out of bounds!");
        return (timestamps[elementId] == currentTimestamp);
    }

    inline void insert(const size_t elementId) {
        AssertMsg(elementId < timestamps.size(), "Element Id out of bounds!");
        timestamps[elementId] = currentTimestamp;
    }

private:
    std::vector<uint16_t> timestamps;

    uint16_t currentTimestamp;
};
}  // namespace TransferPattern
