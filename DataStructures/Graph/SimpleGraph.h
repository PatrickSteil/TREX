/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

template <typename VertexType = std::size_t> struct SimpleGraph {
  std::vector<std::size_t> adjArray;
  std::vector<VertexType> toVertex;

  SimpleGraph() : adjArray(1), toVertex(){};

  SimpleGraph(const SimpleGraph &other)
      : adjArray(other.adjArray), toVertex(other.toVertex){};

  SimpleGraph(SimpleGraph &&other) noexcept
      : adjArray(std::move(other.adjArray)),
        toVertex(std::move(other.toVertex)) {}

  bool isValid(const VertexType v) const { return v < numVertices(); }

  std::size_t numVertices() const { return adjArray.size() - 1; }
  std::size_t numEdges() const { return toVertex.size(); }

  void print() const {
    std::cout << "NumVertices: " << numVertices() << std::endl;
    std::cout << "NumEdges: " << numEdges() << std::endl;

    for (VertexType v = 0; v < numVertices(); ++v) {
      std::cout << "Edges from " << v << std::endl;

      for (std::size_t i = beginEdge(v); i < endEdge(v); ++i) {
        std::cout << toVertex[i] << " ";
      }
      std::cout << std::endl;
    }
  }

  std::size_t degree(const VertexType v) const {
    assert(isValid(v));
    return endEdge(v) - beginEdge(v);
  }

  std::size_t beginEdge(const VertexType v) const {
    assert(isValid(v) || v == numVertices());
    assert(v < adjArray.size());
    return adjArray[v];
  }

  std::size_t endEdge(const VertexType v) const {
    assert(isValid(v));
    assert(v + 1 < adjArray.size());
    return adjArray[v + 1];
  }

  void clear() {
    adjArray.clear();
    toVertex.clear();
  }

  void
  fromEdgeList(std::vector<std::tuple<VertexType, VertexType, uint16_t>> &edges,
               std::size_t numVertices) {
    clear();
    adjArray.resize(numVertices + 1, 0);

    assert(std::is_sorted(edges.begin(), edges.end()));
    /* std::sort(edges.begin(), edges.end()); */

    for (const auto &[u, v, _] : edges) {
      ++adjArray[u + 1];
    }

    for (std::size_t i = 1; i < adjArray.size(); ++i) {
      adjArray[i] += adjArray[i - 1];
    }

    toVertex.resize(edges.size());
    std::vector<std::size_t> offset = adjArray;

    for (const auto &[u, v, _] : edges) {
      toVertex[offset[u]++] = v;
    }
  }

  void showStats() const {
    if (numVertices() == 0) {
      std::cout << "Graph is empty.\n";
      return;
    }

    std::size_t minDegree = std::numeric_limits<std::size_t>::max();
    std::size_t maxDegree = 0;
    std::size_t totalDegree = 0;

    for (VertexType v(0); v < numVertices(); ++v) {
      std::size_t deg = degree(v);
      minDegree = std::min(minDegree, deg);
      maxDegree = std::max(maxDegree, deg);
      totalDegree += deg;
    }

    double avgDegree = static_cast<double>(totalDegree) / numVertices();

    std::cout << "Graph Statistics:\n";
    std::cout << "  Number of vertices: " << numVertices() << "\n";
    std::cout << "  Number of edges:    " << numEdges() << "\n";
    std::cout << "  Min degree:         " << minDegree << "\n";
    std::cout << "  Max degree:         " << maxDegree << "\n";
    std::cout << "  Average degree:     " << avgDegree << "\n";
  }

  void prefetchAdj(const VertexType v) {
    assert(isValid(v) || v == numVertices());
    __builtin_prefetch(&adjArray[v]);
  }
  void prefetchEdges(const VertexType v) {
    assert(isValid(v) || v == numVertices());
    __builtin_prefetch(&toVertex[adjArray[v]]);
  }
};
