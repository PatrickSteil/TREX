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

#include <algorithm>
#include <vector>

#include "../Helpers/Assert.h"
#include "../Helpers/IO/Serialization.h"

class UnionFind {
public:
  UnionFind(const int n = 0) : parent(n, n), n(n) {}

  inline void clear() noexcept { parent.assign(n, n); }
  inline void reset(const int newN) noexcept {
    n = newN;
    clear();
  }

  inline int find(const int i) noexcept {
    if (parent[i] >= n) {
      return i;
    } else {
      parent[i] = find(parent[i]);
      return parent[i];
    }
  }
  inline int operator()(const int i) noexcept { return find(i); }

  inline void unite(const int i, const int j) noexcept {
    if (find(i) != find(j)) {
      link(find(i), find(j));
    }
  }
  inline void operator()(const int i, const int j) noexcept { unite(i, j); }

  inline std::vector<int> getParent() const { return parent; }

  inline void serialize(IO::Serialization &serialize) const noexcept {
    serialize(n, parent);
  }

  inline void deserialize(IO::Deserialization &deserialize) noexcept {
    deserialize(n, parent);
  }

protected:
  inline void link(const int i, const int j) noexcept {
    Assert(parent[i] >= n);
    Assert(parent[j] >= n);
    Assert(i != j);
    if (parent[i] < parent[j]) {
      parent[i] = j;
    } else if (parent[j] < parent[i]) {
      parent[j] = i;
    } else {
      parent[i] = j;
      parent[j]++;
    }
  }

  std::vector<int> parent;
  int n;
};
