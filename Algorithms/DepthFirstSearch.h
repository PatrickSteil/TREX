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

#include <iostream>
#include <string>
#include <vector>

#include "../DataStructures/Graph/Classes/GraphInterface.h"
#include "../Helpers/Assert.h"
#include "../Helpers/Vector/Permutation.h"

template <typename GRAPH, typename TRAVERSE_TREE_EDGE = NO_OPERATION,
          typename TRAVERSE_NON_TREE_EDGE = NO_OPERATION,
          typename BACKTRACK = NO_OPERATION, typename ROOT = NO_OPERATION,
          typename FINALIZE = NO_OPERATION>
class DepthFirstSearch {
 private:
  struct Item {
    Edge edge;
    Vertex from;
    bool backtrack;
  };

 public:
  DepthFirstSearch(
      const GRAPH &graph,
      const TRAVERSE_TREE_EDGE &traverseTreeEdge = TRAVERSE_TREE_EDGE(),
      const TRAVERSE_NON_TREE_EDGE &traverseNonTreeEdge =
          TRAVERSE_NON_TREE_EDGE(),
      const BACKTRACK &backtrack = BACKTRACK(), const ROOT &root = ROOT(),
      const FINALIZE &finalize = FINALIZE())
      : traverseTreeEdge(traverseTreeEdge),
        traverseNonTreeEdge(traverseNonTreeEdge),
        backtrack(backtrack),
        root(root),
        finalize(finalize),
        graph(graph) {}

  inline void initialize() {
    std::vector<bool>(graph.numVertices(), false).swap(settled);
    stack.clear();
  }

  inline void run() {
    initialize();
    for (Vertex vertex : graph.vertices()) {
      if (!settled[vertex]) {
        run(vertex);
      }
    }
  }

  inline void run(Vertex u) {
    root(u);
    settle(u);
    while (!stack.empty()) {
      Edge edge = topEdge();
      Vertex v = topVertex();
      if (topBacktrack()) {
        backtrack(edge, v);
        pop();
      } else {
        Vertex w = graph.get(ToVertex, edge);
        if (settled[w]) {
          traverseNonTreeEdge(edge, v);
          pop();
        } else {
          traverseTreeEdge(edge, v);
          settle(w);
        }
      }
    }
    finalize(u);
  }

  inline bool hasSettled(Vertex vertex) const { return settled[vertex]; }

 private:
  inline void settle(Vertex v) {
    Assert(!settled[v]);
    settled[v] = true;
    for (Edge edge : graph.edgesFrom(v)) {
      push(edge, v);
    }
  }

  inline void push(Edge edge, Vertex from) {
    stack.push_back(Item({edge, from, false}));
  }

  inline Edge topEdge() const { return stack.back().edge; }

  inline Vertex topVertex() const { return stack.back().from; }

  inline bool topBacktrack() {
    bool backtrack = stack.back().backtrack;
    stack.back().backtrack = true;
    return backtrack;
  }

  inline void pop() { stack.pop_back(); }

 private:
  TRAVERSE_TREE_EDGE traverseTreeEdge;
  TRAVERSE_NON_TREE_EDGE traverseNonTreeEdge;
  BACKTRACK backtrack;
  ROOT root;
  FINALIZE finalize;

  const GRAPH &graph;
  std::vector<bool> settled;
  std::vector<Item> stack;
};

template <typename GRAPH, typename OPERATION>
struct SimpleOperation {
  SimpleOperation(const GRAPH &graph, const OPERATION &operation)
      : operation(operation), graph(graph) {}
  inline void operator()(const Edge edge, const Vertex) {
    operation(graph.get(ToVertex, edge));
  }
  OPERATION operation;
  const GRAPH &graph;
};

template <typename GRAPH, typename TRAVERSE_TREE_EDGE = NO_OPERATION,
          typename TRAVERSE_NON_TREE_EDGE = NO_OPERATION,
          typename BACKTRACK = NO_OPERATION>
class DFS
    : public DepthFirstSearch<GRAPH, SimpleOperation<GRAPH, TRAVERSE_TREE_EDGE>,
                              SimpleOperation<GRAPH, TRAVERSE_NON_TREE_EDGE>,
                              SimpleOperation<GRAPH, BACKTRACK>,
                              TRAVERSE_TREE_EDGE, BACKTRACK> {
 public:
  using DFSType =
      DepthFirstSearch<GRAPH, SimpleOperation<GRAPH, TRAVERSE_TREE_EDGE>,
                       SimpleOperation<GRAPH, TRAVERSE_NON_TREE_EDGE>,
                       SimpleOperation<GRAPH, BACKTRACK>, TRAVERSE_TREE_EDGE,
                       BACKTRACK>;
  DFS(const GRAPH &graph,
      const TRAVERSE_TREE_EDGE &traverseTreeEdge = TRAVERSE_TREE_EDGE(),
      const TRAVERSE_NON_TREE_EDGE &traverseNonTreeEdge =
          TRAVERSE_NON_TREE_EDGE(),
      const BACKTRACK &backtrack = BACKTRACK())
      : DFSType(
            graph,
            SimpleOperation<GRAPH, TRAVERSE_TREE_EDGE>(graph, traverseTreeEdge),
            SimpleOperation<GRAPH, TRAVERSE_NON_TREE_EDGE>(graph,
                                                           traverseNonTreeEdge),
            SimpleOperation<GRAPH, BACKTRACK>(graph, backtrack),
            traverseTreeEdge, backtrack) {}
};

template <typename GRAPH, typename TRAVERSE_TREE_EDGE = NO_OPERATION,
          typename TRAVERSE_NON_TREE_EDGE = NO_OPERATION,
          typename BACKTRACK = NO_OPERATION, typename ROOT = NO_OPERATION,
          typename FINALIZE = NO_OPERATION>
inline void depthFirstSearch(
    const GRAPH &graph,
    const TRAVERSE_TREE_EDGE &traverseTreeEdge = TRAVERSE_TREE_EDGE(),
    const TRAVERSE_NON_TREE_EDGE &traverseNonTreeEdge =
        TRAVERSE_NON_TREE_EDGE(),
    const BACKTRACK &backtrack = BACKTRACK(), const ROOT &root = ROOT(),
    const FINALIZE &finalize = FINALIZE()) {
  DepthFirstSearch<GRAPH, TRAVERSE_TREE_EDGE, TRAVERSE_NON_TREE_EDGE, BACKTRACK,
                   ROOT, FINALIZE>
      dfs(graph, traverseTreeEdge, traverseNonTreeEdge, backtrack, root,
          finalize);
  dfs.run();
}

template <typename GRAPH, typename TRAVERSE_TREE_EDGE = NO_OPERATION,
          typename TRAVERSE_NON_TREE_EDGE = NO_OPERATION,
          typename BACKTRACK = NO_OPERATION>
inline void dfs(
    const GRAPH &graph,
    const TRAVERSE_TREE_EDGE &traverseTreeEdge = TRAVERSE_TREE_EDGE(),
    const TRAVERSE_NON_TREE_EDGE &traverseNonTreeEdge =
        TRAVERSE_NON_TREE_EDGE(),
    const BACKTRACK &backtrack = BACKTRACK()) {
  DFS<GRAPH, TRAVERSE_TREE_EDGE, TRAVERSE_NON_TREE_EDGE, BACKTRACK> dfs(
      graph, traverseTreeEdge, traverseNonTreeEdge, backtrack);
  dfs.run();
}

template <typename GRAPH>
inline Order getDFSVertexOrder(const GRAPH &graph) noexcept {
  Order order;
  depthFirstSearch(
      graph,
      [&](const Edge e, const Vertex) {
        const Vertex v = graph.get(ToVertex, e);
        order.emplace_back(v);
      },
      NoOperation, NoOperation, [&](const Vertex v) { order.emplace_back(v); },
      NoOperation);
  return order;
}

template <typename GRAPH>
inline Order getTopologicalOrder(const GRAPH &graph) {
  std::vector<Vertex> order;
  order.reserve(graph.numVertices());
  depthFirstSearch(
      graph, NoOperation, NoOperation,
      [&](const Edge e, const Vertex) {
        const Vertex v = graph.get(ToVertex, e);
        order.emplace_back(v);
      },
      NoOperation, [&](const Vertex v) { order.emplace_back(v); });

  std::reverse(order.begin(), order.end());

  Ensure(order.size() == graph.numVertices(), "Graph was not a DAG!");

  return Order{order};
}
