#pragma once

#include <immintrin.h>

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>

constexpr std::uint32_t MAX_LIMIT = (1 << 31);

union Holder32 {
  __m256i reg;
  uint32_t arr[8];
};

struct SIMD32u {
  Holder32 v;

  explicit SIMD32u(__m256i x) noexcept { v.reg = x; }
  SIMD32u(uint32_t scalar = MAX_LIMIT) noexcept {
    v.reg = _mm256_set1_epi32(scalar);
  }
  void fill(uint32_t scalar = MAX_LIMIT) noexcept {
    v.reg = _mm256_set1_epi32(scalar);
  }

  void reset() noexcept { fill(MAX_LIMIT); }

  static SIMD32u load(const uint32_t *ptr) noexcept {
    return SIMD32u(_mm256_loadu_si256(reinterpret_cast<const __m256i *>(ptr)));
  }
  void store(uint32_t *ptr) const noexcept {
    _mm256_storeu_si256(reinterpret_cast<__m256i *>(ptr), v.reg);
  }

  std::size_t size() const noexcept { return 8; }

  // Random access
  uint32_t &operator[](std::size_t i) noexcept { return v.arr[i & 7]; }
  const uint32_t &operator[](std::size_t i) const noexcept {
    return v.arr[i & 7];
  }

  // Arithmetic
  SIMD32u operator+(const SIMD32u &o) const noexcept {
    return SIMD32u(_mm256_add_epi32(v.reg, o.v.reg));
  }
  SIMD32u operator-(const SIMD32u &o) const noexcept {
    return SIMD32u(_mm256_sub_epi32(v.reg, o.v.reg));
  }

  // Bitwise
  SIMD32u operator&(const SIMD32u &o) const noexcept {
    return SIMD32u(_mm256_and_si256(v.reg, o.v.reg));
  }
  SIMD32u operator|(const SIMD32u &o) const noexcept {
    return SIMD32u(_mm256_or_si256(v.reg, o.v.reg));
  }
  SIMD32u operator^(const SIMD32u &o) const noexcept {
    return SIMD32u(_mm256_xor_si256(v.reg, o.v.reg));
  }

  // Component-wise unsigned min (AVX2 trick)
  void min_with(const SIMD32u &o) noexcept {
    const __m256i offset = _mm256_set1_epi32(0x80000000);
    __m256i a = _mm256_sub_epi32(v.reg, offset);
    __m256i b = _mm256_sub_epi32(o.v.reg, offset);
    __m256i m = _mm256_min_epi32(a, b);
    v.reg = _mm256_add_epi32(m, offset);
  }

  // Shift right by 1 element (insert MAX_LIMIT on left)
  void shift_right1() noexcept {
    __m256i shuffled = _mm256_permutevar8x32_epi32(
        v.reg, _mm256_set_epi32(6, 5, 4, 3, 2, 1, 0, 7));  // rotate right
    // overwrite leftmost element with MAX_LIMIT
    shuffled = _mm256_blend_epi32(_mm256_set1_epi32(MAX_LIMIT), shuffled, 0x7F);
    v.reg = shuffled;
  }

  void print(const char *name) const {
    std::cout << std::setw(10) << name << ": [";
    for (int i = 0; i < 8; ++i) {
      std::cout << v.arr[i] << (i < 7 ? ", " : "");
    }
    std::cout << "]\n";
  }
};
