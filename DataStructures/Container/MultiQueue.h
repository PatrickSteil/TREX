#include <array>
#include <bitset>
#include <cassert>
#include <cstddef>
#include <deque>
#include <stdexcept>

template <typename T, int N = 16> class MultiQueue {
public:
  using Queue = std::deque<T>;

  MultiQueue() = default;

  Queue &at(std::size_t i) {
    assert(i < N);
    return queues_[i];
  }

  const Queue &at(std::size_t i) const {
    assert(i < N);
    return queues_[i];
  }

  void push(std::size_t i, const T &v) {
    contains_.set(i, true);
    at(i).push_back(v);
  }

  void push(std::size_t i, T &&v) {
    contains_.set(i, true);
    at(i).push_back(std::move(v));
  }

  void pop(std::size_t i) {
    assert(i < N);
    auto &q = at(i);
    assert(!q.empty());
    q.pop_front();
  }

  T &front(std::size_t i) {
    assert(i < N);
    return at(i).front();
  }

  const T &front(std::size_t i) const {
    assert(i < N);
    return at(i).front();
  }

  bool empty(std::size_t i) const {
    assert(i < N);
    return at(i).empty();
  }

  std::size_t size(std::size_t i) const {
    assert(i < N);
    return at(i).size();
  }

  Queue &underlying(std::size_t i) {
    assert(i < N);
    return at(i);
  }

  const Queue &underlying(std::size_t i) const {
    assert(i < N);
    return at(i);
  }

  void clear(std::size_t i) {
    assert(i < N);
    at(i).clear();
    contains_.set(i, false);
  }

  void clear_all() {
    for (auto &q : queues_) {
      q.clear();
    }

    contains_.reset();
  }

  bool laterQueueHasElement(std::size_t i) {
    assert(i < N);
    std::bitset<N> shifted = contains_ >> i;
    return shifted.any();
  }

private:
  std::array<Queue, N> queues_;
  std::bitset<N> contains_;
};
