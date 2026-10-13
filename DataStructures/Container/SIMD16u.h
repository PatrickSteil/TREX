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

#include <immintrin.h>

#include <cstdint>
#include <iomanip>
#include <iostream>

union Holder {
    __m256i reg;
    std::uint16_t arr[16];
};

struct SIMD16u {
    Holder v;

    SIMD16u() noexcept = default;
    explicit SIMD16u(__m256i x) noexcept { v.reg = x; }
    SIMD16u(uint16_t scalar) noexcept { v.reg = _mm256_set1_epi16(scalar); }
    void fill(uint16_t scalar) noexcept { v.reg = _mm256_set1_epi16(scalar); }

    static SIMD16u load(const uint16_t* ptr) noexcept {
        return SIMD16u(_mm256_loadu_si256(reinterpret_cast<const __m256i*>(ptr)));
    }
    void store(uint16_t* ptr) const noexcept { _mm256_storeu_si256(reinterpret_cast<__m256i*>(ptr), v.reg); }

    uint16_t& operator[](std::size_t i) noexcept { return v.arr[i & 15]; }
    const uint16_t& operator[](std::size_t i) const noexcept { return v.arr[i & 15]; }

    SIMD16u operator+(const SIMD16u& o) const noexcept { return SIMD16u(_mm256_add_epi16(v.reg, o.v.reg)); }
    SIMD16u operator-(const SIMD16u& o) const noexcept { return SIMD16u(_mm256_sub_epi16(v.reg, o.v.reg)); }

    SIMD16u operator&(const SIMD16u& o) const noexcept { return SIMD16u(_mm256_and_si256(v.reg, o.v.reg)); }
    SIMD16u operator|(const SIMD16u& o) const noexcept { return SIMD16u(_mm256_or_si256(v.reg, o.v.reg)); }
    SIMD16u operator^(const SIMD16u& o) const noexcept { return SIMD16u(_mm256_xor_si256(v.reg, o.v.reg)); }

    SIMD16u sll(int bits) const noexcept { return SIMD16u(_mm256_slli_epi16(v.reg, bits)); }
    SIMD16u srl(int bits) const noexcept { return SIMD16u(_mm256_srli_epi16(v.reg, bits)); }

    SIMD16u cmpeq(const SIMD16u& o) const noexcept { return SIMD16u(_mm256_cmpeq_epi16(v.reg, o.v.reg)); }

    __m256i max(const SIMD16u& o) noexcept {
        __m256i m = _mm256_max_epu16(v.reg, o.v.reg);
        __m256i eq0 = _mm256_cmpeq_epi16(m, v.reg);
        v.reg = m;
        return eq0;
    }

    __m256i min(const SIMD16u& o) noexcept {
        __m256i m = _mm256_min_epu16(v.reg, o.v.reg);
        __m256i eq0 = _mm256_cmpeq_epi16(m, v.reg);
        v.reg = m;
        return eq0;
    }

    void blend(const SIMD16u& other, __m256i mask) noexcept { v.reg = _mm256_blendv_epi8(other.v.reg, v.reg, mask); }
};

void printSIMD(const char* name, const SIMD16u& x) {
    std::cout << std::setw(10) << name << ": [";
    for (int i = 0; i < 16; ++i) {
        std::cout << x.v.arr[i] << (i < 15 ? ", " : "");
    }
    std::cout << "]\n";
}
