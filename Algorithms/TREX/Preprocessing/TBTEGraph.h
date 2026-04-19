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

#include <omp.h>

#include <vector>

#include "../../../Algorithms/DepthFirstSearch.h"
#include "../../../DataStructures/Container/SIMD16u.h"
#include "../../../DataStructures/Graph/Graph.h"
#include "../../../DataStructures/TREX/TREXData.h"
#include "../../../Helpers/Console/Progress.h"

namespace TripBased {

class TBTEGraph {
public:
    const TREXData& data;
    DynamicTBTEGraph graph;

    std::vector<std::uint8_t> rank;

    std::vector<SIMD16u> distances;
    std::vector<SIMD16u> parents;

    TBTEGraph(const TREXData& data) : data(data), rank(data.stopEventGraph.numEdges(), 0), distances(), parents() {}

    static std::uint8_t extractWeight(std::uint16_t packed) noexcept { return static_cast<std::uint8_t>(packed >> 15); }

    static std::uint16_t extractEdgeIndex(std::uint16_t packed) noexcept {
        return static_cast<std::uint16_t>(packed & 0x7FFF);
    }

    std::uint8_t getWeight(const Edge& e) const noexcept {
        auto packed = graph.get(TransferCost, e);
        return extractWeight(packed);
    }

    std::uint16_t getEdgeIndex(const Edge& e) const noexcept {
        auto packed = graph.get(TransferCost, e);
        return extractEdgeIndex(packed);
    }

    void buildTBTEGraph() {
        const std::size_t n = data.stopEventGraph.numVertices();
        std::vector<Vertex> splitVertex(data.stopEventGraph.numVertices());
        std::iota(splitVertex.begin(), splitVertex.end(), Vertex(n));

        EdgeListTBTEGraph builder;
        builder.addVertices(2 * n);
        builder.reserve(2 * n, n + data.numberOfTrips() + data.stopEventGraph.numEdges());

        {
            Progress progress(data.stopEventGraph.numEdges());
            for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
                const Vertex to = data.stopEventGraph.get(ToVertex, edge);

                AssertMsg(builder.isVertex(from), "From Vertex is not valid!");
                AssertMsg(builder.isVertex(to), "To  Vertex is not valid!");

                auto edgeHandle = builder.addEdge(from, splitVertex[to]);

                builder.set(TransferCost, edgeHandle, 1);
                builder.set(OriginalEdge, edgeHandle, edge);
                ++progress;
            }
            progress.finished();
        }

        {
            Progress progress(data.numberOfTrips());
            for (TripId trip = TripId(0); trip < data.numberOfTrips(); ++trip) {
                const StopEventId firstEvent(data.firstStopEventOfTrip[trip]);
                const StopEventId endEvent(data.firstStopEventOfTrip[trip + 1] - 1);

                for (StopEventId e = firstEvent; e < endEvent; ++e) {
                    builder.addEdge(splitVertex[e], Vertex(e + 1)).set(TransferCost, 0);
                }
                ++progress;
            }
            progress.finished();
        }

        {
            Progress progress(n);
            for (Vertex v(0); v < n; ++v) {
                builder.addEdge(v, splitVertex[v]).set(TransferCost, 0);
                ++progress;
            }
            progress.finished();
        }

        graph.clear();
        Graph::move(std::move(builder), graph);
        graph.sortEdges(ToVertex);

#pragma omp parallel for
        for (TripId trip = TripId(0); trip < data.numberOfTrips(); ++trip) {
            const StopEventId firstEvent(data.firstStopEventOfTrip[trip]);
            const StopId* stops = data.stopArrayOfTrip(trip);
            for (StopIndex i = StopIndex(0); i < data.numberOfStopsInTrip(trip); i++) {
                const StopId stop = stops[i];
                graph.set(CellId, Vertex(firstEvent + i), data.cellIds[stop]);
            }
        }

        // this sets the edgeIndex inside the TransferCost
#pragma omp parallel for
        for (Vertex u = Vertex(0); u < graph.numVertices(); ++u) {
            std::uint16_t edgeIdx = 0;

            for (auto edge : graph.edgesFrom(u)) {
                std::uint16_t w = (graph.get(TransferCost, edge) << 15) | edgeIdx;
                graph.set(TransferCost, edge, w);
                ++edgeIdx;
            }
        }

        Graph::printInfo(graph);

        AssertMsg(Graph::isAcyclic(graph), "Graph is not acyclic!");

        Order topoOrder = getTopologicalOrder(graph);

        graph.applyVertexOrder(topoOrder);

        // now the other datastructes
        rank.assign(data.stopEventGraph.numEdges(), 0);
        distances.resize(graph.numEdges());
        parents.resize(graph.numEdges());
    }
};
}  // namespace TripBased
