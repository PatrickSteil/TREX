#include <array>
#include <bitset>
#include <cassert>
#include <cstddef>
#include <vector>

template <typename T, size_t MaxRounds = 16> class MultiQueue {
public:
  explicit MultiQueue(size_t capacityPerQueue) : capacity(capacityPerQueue) {
    for (size_t i = 0; i < MaxRounds; ++i) {
      queues[i].resize(capacity);
      begin[i] = 0;
      end[i] = 0;
    }
    nonEmpty.reset();
  }

  size_t size(int r) const noexcept {
    assert(r >= 0 && r < static_cast<int>(MaxRounds));
    return end[r] - begin[r];
  }

  size_t &start(int r) noexcept {
    assert(r >= 0 && r < static_cast<int>(MaxRounds));
    return begin[r];
  }

  size_t &finish(int r) noexcept {
    assert(r >= 0 && r < static_cast<int>(MaxRounds));
    return end[r];
  }

  std::vector<T> &queue(int r) noexcept {
    assert(r >= 0 && r < static_cast<int>(MaxRounds));
    return queues[r];
  }

  T &operator()(int r, size_t i) noexcept {
    assert(r >= 0 && r < static_cast<int>(MaxRounds));
    assert(i < end[r]);
    return queues[r][i];
  }

  const T &operator()(int r, size_t i) const noexcept {
    assert(r >= 0 && r < static_cast<int>(MaxRounds));
    assert(i < end[r]);
    return queues[r][i];
  }

  void push(int r, const T &value) noexcept {
    assert(r >= 0 && r < static_cast<int>(MaxRounds));
    assert(end[r] < capacity);
    queues[r][end[r]++] = value;
    nonEmpty.set(r);
  }

  void push(int r, T &&value) noexcept {
    assert(r >= 0 && r < static_cast<int>(MaxRounds));
    assert(end[r] < capacity);
    queues[r][end[r]++] = std::move(value);
    nonEmpty.set(r);
  }

  void clear_all() noexcept {
    for (size_t r = 0; r < MaxRounds; ++r) {
      begin[r] = 0;
      end[r] = 0;
    }
    nonEmpty.reset();
  }

  bool laterQueueNonEmpty(size_t r) const noexcept {
    assert(r < MaxRounds);
    return (nonEmpty >> r).any();
  }

private:
  size_t capacity;

  std::array<std::vector<T>, MaxRounds> queues;
  std::array<size_t, MaxRounds> begin;
  std::array<size_t, MaxRounds> end;

  std::bitset<MaxRounds> nonEmpty;
};
