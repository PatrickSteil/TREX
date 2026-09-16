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

#include <deque>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "../../DataStructures/Attributes/AttributeNames.h"
#include "../../DataStructures/Container/Set.h"
#include "../../Helpers/Meta.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/Types.h"
#include "../../Helpers/Vector/Vector.h"

// This Variant of Dial's algorithm is tuned for 0-1 weighted edges. Technically
// it is called 0-1 BFS, but the overlying concept of "buckets" is due to Dial
// https://cp-algorithms.com/graph/01_bfs.html

template <typename GRAPH, bool DEBUG = false>
class DialDijkstra {
public:
    using Graph = GRAPH;
    static constexpr bool Debug = DEBUG;
    using Type = DialDijkstra<Graph, Debug>;

public:
    struct VertexLabel {
        VertexLabel() : distance(intMax), parent(noVertex), timeStamp(-1), isInQueue(false) {}
        inline void reset(int time) {
            distance = intMax;
            parent = noVertex;
            timeStamp = time;
            isInQueue = false;
        }

        int distance;
        Vertex parent;
        int timeStamp;
        bool isInQueue;
    };

public:
    DialDijkstra(const GRAPH& graph, const std::vector<int>& weight)
        : graph(graph), weight(weight), Q(), label(graph.numVertices()), timeStamp(0), settleCount(0) {}

    DialDijkstra(const GRAPH& graph) : DialDijkstra(graph, graph[TravelTime]) {}

    template <AttributeNameType ATTRIBUTE_NAME>
    DialDijkstra(const GRAPH& graph, const AttributeNameWrapper<ATTRIBUTE_NAME> weight)
        : DialDijkstra(graph, graph[weight]) {}

    DialDijkstra(const GRAPH&&, const std::vector<int>&) = delete;
    DialDijkstra(const GRAPH&, const std::vector<int>&&) = delete;
    DialDijkstra(const GRAPH&&) = delete;

    template <AttributeNameType ATTRIBUTE_NAME>
    DialDijkstra(const GRAPH&&, const AttributeNameWrapper<ATTRIBUTE_NAME>) = delete;

    template <typename SETTLE = NO_OPERATION, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION>
    inline void run(const Vertex source, const Vertex target = noVertex, const SETTLE& settle = NoOperation,
                    const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        clear();
        addSource(source);
        run(target, settle, stop, pruneEdge);
    }

    template <typename SETTLE = NO_OPERATION, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION>
    inline void run(const Vertex source, IndexedSet<false, Vertex>& targets, const SETTLE& settle = NoOperation,
                    const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        clear();
        addSource(source);
        run(
            noVertex,
            [&](const Vertex u) {
                settle(u);
                targets.remove(u);
            },
            [&]() { return stop() || targets.empty(); }, pruneEdge);
    }

    template <typename SOURCE_CONTAINER, typename SETTLE = NO_OPERATION, typename STOP = NO_OPERATION,
              typename PRUNE_EDGE = NO_OPERATION, typename = decltype(std::declval<SOURCE_CONTAINER>().begin())>
    inline void run(const SOURCE_CONTAINER& sources, const Vertex target = noVertex, const SETTLE& settle = NoOperation,
                    const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        clear();
        for (const Vertex source : sources) {
            addSource(source);
        }
        run(target, settle, stop, pruneEdge);
    }

    inline void clear() noexcept {
        if constexpr (Debug) {
            timer.restart();
            settleCount = 0;
        }
        Q.clear();
        timeStamp++;
    }

    inline void addSource(const Vertex source, const int distance = 0) noexcept {
        VertexLabel& sourceLabel = getLabel(source);
        sourceLabel.distance = distance;
        sourceLabel.isInQueue = true;
        Q.push_front(source);
    }

    inline void run() noexcept { run(noVertex, NoOperation, NoOperation, NoOperation); }

    template <typename SETTLE, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION,
              typename = decltype(std::declval<SETTLE>()(std::declval<Vertex>()))>
    inline void run(const Vertex target, const SETTLE& settle, const STOP& stop = NoOperation,
                    const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        while (!Q.empty()) {
            if (stop()) break;
            Vertex u = Q.front();
            Q.pop_front();

            VertexLabel& uLabel = label[u];
            uLabel.isInQueue = false;

            if (u == target) break;
            for (const Edge edge : graph.edgesFrom(u)) {
                const Vertex v = graph.get(ToVertex, edge);
                if (pruneEdge(u, edge)) continue;

                VertexLabel& vLabel = getLabel(v);
                const int distance = uLabel.distance + weight[edge];
                if (vLabel.distance > distance) {
                    vLabel.distance = distance;
                    vLabel.parent = u;

                    if (!vLabel.isInQueue) {
                        if (weight[edge] == 0) {
                            Q.push_front(v);
                        } else {
                            Q.push_back(v);
                        }
                        vLabel.isInQueue = true;
                    }
                }
            }
            settle(u);
            if constexpr (Debug) settleCount++;
        }
        if constexpr (Debug) {
            std::cout << "Settled Vertices = " << String::prettyInt(settleCount) << std::endl;
            std::cout << "Time = " << String::msToString(timer.elapsedMilliseconds()) << std::endl;
        }
    }

    inline bool reachable(const Vertex vertex) const noexcept { return label[vertex].timeStamp == timeStamp; }

    inline bool visited(const Vertex vertex) const noexcept { return label[vertex].timeStamp == timeStamp; }

    inline int getDistance(const Vertex vertex) const noexcept {
        if (visited(vertex)) return label[vertex].distance;
        return -1;
    }

    inline Vertex getParent(const Vertex vertex) const noexcept {
        if (visited(vertex)) return label[vertex].parent;
        return noVertex;
    }

    inline std::set<Vertex> getChildren(const Vertex vertex) const noexcept {
        if (visited(vertex)) {
            std::set<Vertex> children;
            for (Vertex child : graph.outgoingNeighbors(vertex)) {
                if (label[child].parent == vertex) {
                    children.insert(child);
                }
            }
            return children;
        }
        return std::set<Vertex>();
    }

    inline Vertex getQFront() const noexcept {
        if (Q.empty()) return noVertex;
        return Q.front();
    }

    inline std::vector<Vertex> getReversePath(const Vertex to) const noexcept {
        std::vector<Vertex> path;
        if (!visited(to)) return path;
        path.push_back(to);
        while (label[path.back()].parent != noVertex) {
            path.push_back(label[path.back()].parent);
        }
        return path;
    }

    inline std::vector<Vertex> getPath(const Vertex to) const noexcept { return Vector::reverse(getReversePath(to)); }

    inline int getSettleCount() const noexcept { return settleCount; }

private:
    inline VertexLabel& getLabel(const Vertex vertex) noexcept {
        VertexLabel& result = label[vertex];
        if (result.timeStamp != timeStamp) result.reset(timeStamp);
        return result;
    }

private:
    const GRAPH& graph;
    const std::vector<int>& weight;

    std::deque<Vertex> Q;

    std::vector<VertexLabel> label;
    int timeStamp;

    int settleCount;
    Timer timer;
};
