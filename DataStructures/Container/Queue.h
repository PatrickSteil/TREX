#pragma once
#include <cassert>
#include <cstddef>
#include <vector>

template <typename T> class PreallocatedQueue {
public:
  explicit PreallocatedQueue(std::size_t capacity)
      : buffer(capacity), read_idx(0), write_idx(0) {}

  std::size_t push(const T &value) noexcept {
    assert(write_idx < buffer.size());
    buffer[write_idx] = value;
    return write_idx++;
  }

  std::size_t push(T &&value) noexcept {
    assert(write_idx < buffer.size());
    buffer[write_idx] = std::move(value);
    return write_idx++;
  }

  template <typename... Args> std::size_t emplace(Args &&...args) noexcept {
    assert(write_idx < buffer.size());
    new (&buffer[write_idx]) T(std::forward<Args>(args)...);
    return write_idx++;
  }

  std::size_t pop() noexcept {
    assert(read_idx < write_idx);
    return read_idx++;
  }

  T &operator[](std::size_t idx) noexcept {
    assert(idx < write_idx);
    return buffer[idx];
  }

  const T &operator[](std::size_t idx) const noexcept {
    assert(idx < write_idx);
    return buffer[idx];
  }

  T &front() noexcept {
    assert(read_idx < write_idx);
    return buffer[read_idx];
  }

  const T &front() const noexcept {
    assert(read_idx < write_idx);
    return buffer[read_idx];
  }

  bool empty() const noexcept { return read_idx == write_idx; }

  std::size_t size() const noexcept { return write_idx - read_idx; }

  std::size_t capacity() const noexcept { return buffer.size(); }

  std::size_t read_index() const noexcept { return read_idx; }
  std::size_t write_index() const noexcept { return write_idx; }

  void clear() noexcept {
    read_idx = 0;
    write_idx = 0;
  }

private:
  std::vector<T> buffer;
  std::size_t read_idx;
  std::size_t write_idx;
};
