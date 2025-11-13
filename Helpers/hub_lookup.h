#pragma once

#include <array>
#include <cassert>
#include <cstdint>
#include <limits>

#include "flat_hash_map.hpp"

class HubLookup {
public:
  using Key = std::uint32_t;
  using Value = std::uint16_t;

  HubLookup() {
    array_.fill(empty_value_);
    map_.reserve(256);
  }

  void insert(Key key, Value value) {
    if (key <= 0xFFFF) [[likely]] {
      array_[key] = value;
    } else {
      map_[key] = value;
    }
  }

  Value find(Key key) {
    if (key <= 0xFFFF) {
      return array_[key];
    } else {
      auto it = map_.find(key);
      return it != map_.end() ? it->second : empty_value_;
    }
  }

  void clearMap() { map_.clear(); }

private:
  static constexpr Value empty_value_ = std::numeric_limits<Value>::max();
  alignas(8) std::array<Value, 65536> array_;
  ska::flat_hash_map<Key, Value> map_;
};
