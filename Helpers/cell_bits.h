#pragma once

#include <cstdint>

uint32_t expandLoop(uint16_t x) {
  uint32_t y = 0;

  for (int i = 0; i < 16; ++i) {
    if (x & (1u << i))
      y |= (2u << (2 * i)); // 10
    else
      y |= (1u << (2 * i)); // 01
  }

  return y;
}

uint32_t expand(uint16_t x) {
  uint32_t v = x;

  v = (v | (v << 8)) & 0x00FF00FF;
  v = (v | (v << 4)) & 0x0F0F0F0F;
  v = (v | (v << 2)) & 0x33333333;
  v = (v | (v << 1)) & 0x55555555;

  return (~v & 0x55555555) | (v << 1);
}

// the latter is 3-4 times faster than the first one; it is boilder down to some
// bit operations the first one is compiler to SIMD, hence the additional
// overhead
