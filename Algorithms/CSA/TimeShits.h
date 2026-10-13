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
namespace CSA {
// time has 32 bits [32 ... 14 | 13 ... 9 | 8 ... 1] - rounded | number of legs
// | lower exact arr time bits

const int offset = (1 << 8);

inline int getRoundedArrivalTime(int time) noexcept { return (((0xfFFFFE000) & (time)) >> 5); }

inline int getExactArrivalTime(int time) noexcept { return getRoundedArrivalTime(time) + (0xFF & time); }

inline int getNumberOfTransfers(int time) noexcept { return (((0b1111100000000) & (time)) >> 8); }

inline int shiftTime(int time) noexcept { return ((0xFF & (time)) + ((0xFFFFFF00 & (time)) << 5)); }

inline int increaseTransferCounter(int time) noexcept { return time + offset; }
}  // namespace CSA
