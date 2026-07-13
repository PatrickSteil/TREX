#pragma once

#include <ranges>

template <typename T> auto reverse_range(T begin, T end) {
  return std::views::iota(begin, end) | std::views::reverse;
}
