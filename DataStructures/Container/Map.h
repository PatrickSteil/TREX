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

#include <map>

#include "../../Helpers/Ranges/SimultaneousRange.h"
#include "../../Helpers/Types.h"
#include "../../Helpers/Vector/Vector.h"

template <typename KEY, typename VALUE>
class Map : public std::map<KEY, VALUE> {
 public:
  using Key = KEY;
  using Value = VALUE;
  using Type = Map<Key, Value>;

 private:
  using Super = std::map<Key, Value>;

 public:
  inline bool contains(const Key& key) const noexcept {
    return Super::count(key) == 1;
  }

  inline void insert(const Key& key, const Value& value) noexcept {
    Super::operator[](key) = value;
  }
};

template <typename VALUE, bool RESIZEABLE = false, typename KEY_TYPE = size_t>
class IndexedMap {
 public:
  using Value = VALUE;
  static constexpr bool Resizeable = RESIZEABLE;
  using KeyType = KEY_TYPE;
  using Type = IndexedMap<Value, Resizeable, KeyType>;

  inline static constexpr size_t NotContained =
      std::numeric_limits<size_t>::max();
  using Iterator = typename std::vector<Value>::const_iterator;

 public:
  IndexedMap(const size_t initialCapacity)
      : indices(initialCapacity, NotContained) {}

  inline const std::vector<KeyType>& getKeys() const noexcept { return keys; }

  inline const std::vector<Value>& getValues() const noexcept { return values; }

  inline SimultaneousRange<std::vector<KeyType>, std::vector<Value>> map()
      const noexcept {
    return SimultaneousRange<std::vector<KeyType>, std::vector<Value>>(keys,
                                                                       values);
  }

  inline Iterator begin() const noexcept { return values.begin(); }

  inline Iterator end() const noexcept { return values.end(); }

  inline size_t size() const noexcept { return values.size(); }

  inline bool empty() const noexcept { return values.empty(); }

  inline size_t capacity() const noexcept { return indices.size(); }

  inline bool contains(const KeyType key) noexcept {
    if (Resizeable) {
      if ((size_t)key >= capacity()) indices.resize(key + 1, NotContained);
    } else {
      AssertMsg((size_t)key < capacity(), "Key " << key << " is out of range!");
    }
    return indices[key] != NotContained;
  }

  inline const Value& operator[](const KeyType key) const noexcept {
    AssertMsg((size_t)key < capacity(),
              "No value for key " << key << " contained!");
    return values[indices[key]];
  }

  inline Value& operator[](const KeyType key) noexcept {
    AssertMsg(contains(key), "No value for key " << key << " contained!");
    return values[indices[key]];
  }

  inline bool insert(const KeyType key, const Value& value = Value()) noexcept {
    if (contains(key)) {
      values[indices[key]] = value;
      return false;
    } else {
      indices[key] = keys.size();
      keys.emplace_back(key);
      values.emplace_back(value);
      return true;
    }
  }

  inline bool remove(const KeyType key) noexcept {
    if (!contains(key)) return false;
    keys[indices[key]] = keys.back();
    values[indices[key]] = values.back();
    indices[keys.back()] = indices[key];
    indices[key] = NotContained;
    keys.pop_back();
    values.pop_back();
    return true;
  }

  inline void clear() noexcept {
    for (const KeyType key : keys) {
      indices[key] = NotContained;
    }
    keys.clear();
    values.clear();
  }

  inline bool isSortedByKeys() const noexcept { return Vector::isSorted(keys); }

  inline void sortKeys() noexcept {
    std::sort(keys.begin(), keys.end());
    std::vector<Value> valuesCopy(values);
    for (size_t i = 0; i < keys.size(); i++) {
      values[i] = valuesCopy[indices[keys[i]]];
      indices[keys[i]] = i;
    }
  }

  inline void sortLastNKeys(const size_t n) noexcept {
    sortLastNKeys(n, [&](const KeyType& a, const KeyType& b) { return a < b; });
  }

  template <typename LESS>
  inline void sortLastNKeys(const size_t n, const LESS& less) noexcept {
    AssertMsg(n <= indices.size(),
              "n = " << n << " is greater than the number of keys = "
                     << indices.size());
    if (n == 0) return;
    std::sort(keys.end() - n, keys.end(), less);
    std::vector<Value> valuesCopy(values.end() - n, values.end());
    size_t firstIndex = keys.size() - n;
    for (size_t i = firstIndex; i < firstIndex + n; i++) {
      values[i] = valuesCopy[indices[keys[i]] - firstIndex];
      indices[keys[i]] = i;
    }
  }

 private:
  std::vector<size_t> indices;
  std::vector<KeyType> keys;
  std::vector<Value> values;
};
