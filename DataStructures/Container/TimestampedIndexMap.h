#pragma once
#include <cassert>
#include <cstdint>
#include <limits>
#include <vector>

template <typename VALUE, typename KEY_TYPE = std::size_t>
class TimestampedIndexedMap {
public:
  using Value = VALUE;
  using KeyType = KEY_TYPE;

  static constexpr Value DefaultValue = std::numeric_limits<Value>::max();

public:
  explicit TimestampedIndexedMap(std::size_t capacity)
      : index(capacity), stamp(capacity, 0), currentStamp(1) {}

  inline std::size_t size() const noexcept { return values.size(); }
  inline bool empty() const noexcept { return values.empty(); }
  inline std::size_t capacity() const noexcept { return index.size(); }

  inline const std::vector<KeyType> &getKeys() const noexcept { return keys; }
  inline const std::vector<Value> &getValues() const noexcept { return values; }

  inline bool contains(KeyType key) const noexcept {
    assert((std::size_t)key < index.size());
    return stamp[key] == currentStamp;
  }

  inline const Value &operator[](KeyType key) const noexcept {
    assert(contains(key));
    return values[index[key]];
  }

  inline Value getOrDefault(KeyType key) const noexcept {
    if ((std::size_t)key >= index.size())
      return DefaultValue;
    if (stamp[key] != currentStamp)
      return DefaultValue;
    return values[index[key]];
  }

  inline Value &getOrInit(KeyType key, Value init) noexcept {
    if (stamp[key] != currentStamp) {
      stamp[key] = currentStamp;
      index[key] = values.size();
      keys.push_back(key);
      values.push_back(init);
    }
    return values[index[key]];
  }

  inline bool insert(KeyType key, Value value) noexcept {
    assert((std::size_t)key < index.size());

    if (stamp[key] == currentStamp) {
      values[index[key]] = value;
      return false;
    }

    stamp[key] = currentStamp;
    index[key] = values.size();
    keys.push_back(key);
    values.push_back(value);
    return true;
  }

  inline Value &operator()(KeyType key) noexcept {
    assert((std::size_t)key < index.size());

    if (stamp[key] != currentStamp) {
      stamp[key] = currentStamp;
      index[key] = values.size();
      keys.push_back(key);
      values.emplace_back(DefaultValue);
    }
    return values[index[key]];
  }

  inline void clear() noexcept {
    currentStamp++;
    keys.clear();
    values.clear();

    if (currentStamp == 0) {
      std::fill(stamp.begin(), stamp.end(), 0);
      currentStamp = 1;
    }
  }

  inline void prefetch(KeyType key) const noexcept {
    __builtin_prefetch(&stamp[key], 0, 1);
    __builtin_prefetch(&index[key], 0, 1);
  }

private:
  std::vector<std::size_t> index; // key -> position
  std::vector<uint32_t> stamp;    // key -> round id
  uint32_t currentStamp;

  std::vector<KeyType> keys;
  std::vector<Value> values;
};
