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

#include "../../Helpers/Assert.h"
#include "../../Helpers/Types.h"
#include "../Graph/Graph.h"

namespace Graph {

template <typename GRAPH>
static inline DynamicFlowGraph generateFlowGraph(const GRAPH& graph, const std::vector<int>& capacities,
                                                 const bool undirectedEdges = false) noexcept {
    DynamicFlowGraph flowGraph;
    flowGraph.addVertices(graph.numVertices());
    for (const auto [edge, from] : graph.edgesWithFromVertex()) {
        const Vertex to = graph.get(ToVertex, edge);
        if (from == to) continue;
        const Edge forwardEdge = flowGraph.findOrAddEdge(from, to);
        flowGraph.set(Capacity, forwardEdge, flowGraph.get(Capacity, forwardEdge) + capacities[edge]);
        if (undirectedEdges) {
            const Edge backwardEdge = flowGraph.findOrAddEdge(to, from);
            flowGraph.set(Capacity, backwardEdge, flowGraph.get(Capacity, backwardEdge) + capacities[edge]);
        } else {
            flowGraph.findOrAddEdge(to, from);
        }
    }
    return flowGraph;
}

template <typename GRAPH>
static inline DynamicFlowGraph generateFlowGraph(const GRAPH& graph, const bool undirectedEdges = false) noexcept {
    DynamicFlowGraph flowGraph;
    flowGraph.addVertices(graph.numVertices());
    for (const auto [edge, from] : graph.edgesWithFromVertex()) {
        const Vertex to = graph.get(ToVertex, edge);
        if (from == to) continue;
        flowGraph.set(Capacity, flowGraph.findOrAddEdge(from, to), 1);
        if (undirectedEdges) {
            flowGraph.set(Capacity, flowGraph.findOrAddEdge(to, from), 1);
        } else {
            flowGraph.findOrAddEdge(to, from);
        }
    }
    return flowGraph;
}

template <typename GRAPH>
inline DynamicFlowGraph generateVertexFlowGraph(const GRAPH& graph) noexcept {
    DynamicFlowGraph flowGraph;
    const Vertex offset = Vertex(graph.numVertices());
    flowGraph.addVertices(offset * 2);
    for (const Vertex from : graph.vertices()) {
        for (const Edge edge : graph.edgesFrom(from)) {
            const Vertex to = graph.get(ToVertex, edge);
            if (from == to) continue;
            flowGraph.set(Capacity, flowGraph.findOrAddEdge(from, to + offset), 0);
            flowGraph.set(Capacity, flowGraph.findOrAddEdge(to + offset, from), 1);
            flowGraph.set(Capacity, flowGraph.findOrAddEdge(to, from + offset), 0);
            flowGraph.set(Capacity, flowGraph.findOrAddEdge(from + offset, to), 1);
        }
        flowGraph.set(Capacity, flowGraph.findOrAddEdge(from, from + offset), 1);
        flowGraph.set(Capacity, flowGraph.findOrAddEdge(from + offset, from), 0);
    }
    return flowGraph;
}

}  // namespace Graph
