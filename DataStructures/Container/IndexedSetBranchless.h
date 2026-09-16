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

#include <cassert>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

template <typename key_t = std::uint32_t>
class IndexedSetBranchless {
public:
    explicit IndexedSetBranchless(std::size_t n) : stamps_(n + 1, 0), elems_(n), write_(0), epoch_(1) {}

    bool insert(std::size_t x, bool do_insert = true) {
        assert(x < stamps_.size());
        std::uint16_t prev = stamps_[x];
        std::uint16_t mask = static_cast<std::uint16_t>(do_insert) & static_cast<std::uint16_t>(prev != epoch_);
        stamps_[x] = prev + (epoch_ - prev) * mask;
        elems_[write_] = static_cast<key_t>(x);
        write_ += mask;
        return mask != 0;
    }

    bool contains(std::size_t x) const {
        assert(x < stamps_.size());
        return stamps_[x] == epoch_;
    }

    std::size_t capacity() const { return elems_.size(); }
    std::size_t size() const { return write_; }

    void clear() {
        ++epoch_;
        write_ = 0;
        if (epoch_ == 0) {
            std::fill(stamps_.begin(), stamps_.end(), 0);
            epoch_ = 1;
        }
    }

    using const_iterator = typename std::vector<key_t>::const_iterator;
    const_iterator begin() const { return elems_.begin(); }
    const_iterator end() const { return elems_.begin() + write_; }

private:
    std::vector<std::uint16_t> stamps_;
    std::vector<key_t> elems_;
    std::size_t write_;
    std::uint16_t epoch_;
};
