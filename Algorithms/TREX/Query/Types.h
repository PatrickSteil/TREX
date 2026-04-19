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

#include <execution>
#include <vector>

#include "../../TripBased/Query/Types.h"
#include "../../../DataStructures/Graph/SimpleGraph.h"
#include "../../../DataStructures/TREX/TREXData.h"

namespace TripBased {

struct TransfersWithoutOverlays {
    TransfersWithoutOverlays(const TREXData& data)
        : beginOut(data.stopEventGraph.getBeginOut()),
          labels(data.stopEventGraph.numEdges()),
          travelTime(data.stopEventGraph.get(TravelTime)) {
        std::vector<uint16_t> cellIdOfEvent(data.numberOfStopEvents(), 0);
#pragma omp parallel for
        for (size_t event = 0; event < data.numberOfStopEvents(); ++event) {
            const StopId stop = data.getStopOfStopEvent(StopEventId(event));
            AssertMsg(data.raptorData.isStop(stop), "Stop is not a stop!");
            cellIdOfEvent[event] = (uint16_t)data.getCellIdOfStop(stop);
        }

        // this is to test how well trip-ranked pruning works
        std::vector<uint8_t> rankOfRoute(data.numberOfRoutes(), 0);
        std::vector<uint8_t> rankOfTrip(data.numberOfTrips(), 0);
        std::vector<uint8_t> rankOfEvent(data.numberOfStopEvents(), 0);
        for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
            TripId trip = data.tripOfStopEvent[StopEventId(from)];
            AssertMsg(trip < rankOfTrip.size(), "Trip is out of bounds!");
            rankOfTrip[trip] = std::max(rankOfTrip[trip], data.stopEventGraph.get(LocalLevel, edge));
            RouteId route = data.routeOfTrip[trip];
            rankOfRoute[route] = std::max(rankOfRoute[route], data.stopEventGraph.get(LocalLevel, edge));
            rankOfEvent[from] = std::max(rankOfEvent[from], data.stopEventGraph.get(LocalLevel, edge));
        }

        for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
            const Vertex toVertex = data.stopEventGraph.get(ToVertex, edge);
            const TripId trip = data.tripOfStopEvent[toVertex];
            const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
            labels[edge].setTrip(trip);
            labels[edge].setFirstEvent(firstEvent);
            labels[edge].setStopIndex(StopIndex(toVertex - firstEvent + 1));
            labels[edge].setCellId(cellIdOfEvent[labels[edge].getStopEvent() - 1]);
            // set different ranks
            /* labels[edge].setRank( */
            /*     rankOfRoute[data.routeOfTrip[data.tripOfStopEvent[from]]]);
             */
            /* labels[edge].setRank(rankOfTrip[data.tripOfStopEvent[from]]);
             */
            /* labels[edge].setRank(rankOfEvent[from]); */
            labels[edge].setRank(data.stopEventGraph.get(LocalLevel, edge));
        }
    }

    std::vector<Edge> beginOut;
    std::vector<EdgeLabelCellId> labels;
    std::vector<int> travelTime;
};

struct TransfersWithOverlays {
    TransfersWithOverlays(const TREXData& data)
        : beginOut(data.numberOfLevels + 1),
          labels(data.numberOfLevels + 1),
          travelTime(data.stopEventGraph.get(TravelTime)) {
        // fill the overlayGraphs _per level_
        int numOverlayGraphs = data.numberOfLevels + 1;
        std::vector<SimpleGraph<std::uint32_t>> overlayGraphs;
        overlayGraphs.reserve(numOverlayGraphs);

        for (int i = 0; i < numOverlayGraphs; ++i) {
            overlayGraphs.emplace_back();
        }

        AssertMsg(overlayGraphs.size() == static_cast<std::size_t>(numOverlayGraphs),
                  "The number of overlay graphs is off!");

        std::vector<std::tuple<std::uint32_t, std::uint32_t, uint16_t>> edgesToInsert;
        edgesToInsert.reserve(data.stopEventGraph.numEdges());

        for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
            edgesToInsert.emplace_back((std::uint32_t)from, (std::uint32_t)data.stopEventGraph.get(ToVertex, edge),
                                       data.stopEventGraph.get(LocalLevel, edge));
        }

        for (int i = 0; i < numOverlayGraphs; ++i) {
            std::sort(std::execution::par, edgesToInsert.begin(), edgesToInsert.end());
            overlayGraphs[i].fromEdgeList(edgesToInsert, data.stopEventGraph.numVertices());
            labels[i].resize(edgesToInsert.size());

            beginOut[i].resize(data.stopEventGraph.numVertices() + 1);
            for (std::size_t event = 0; event <= data.stopEventGraph.numVertices(); event++) {
                beginOut[i][event] = Edge(overlayGraphs[i].beginEdge(Vertex(event)));
            }
            beginOut[i].back() = Edge(overlayGraphs[i].numEdges());

            for (std::size_t edge = 0; edge < edgesToInsert.size(); ++edge) {
                auto [from, to, rank] = edgesToInsert[edge];
                AssertMsg(to < data.numberOfStopEvents(), "To StopEventId is invalid!");
                AssertMsg(from < data.numberOfStopEvents(), "From StopEventId is invalid!");
                AssertMsg(rank < 16, "Rank is invalid!");

                const TripId trip = data.tripOfStopEvent[to];
                const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
                labels[i][edge].setTrip(trip);
                labels[i][edge].setFirstEvent(firstEvent);
                labels[i][edge].setStopIndex(StopIndex(to - firstEvent + 1));
            }

            edgesToInsert.erase(std::remove_if(edgesToInsert.begin(), edgesToInsert.end(),
                                               [i](const auto& e) { return std::get<2>(e) <= (uint16_t)i; }),
                                edgesToInsert.end());
        }

        AssertMsg(edgesToInsert.size() == 0, "The Edge Graph still has edges?");

        for (int i = 0; i < numOverlayGraphs; ++i) {
            std::cout << "Overlay Graph " << i << ": " << labels[i].size() << "\n";
        }
    }

    std::vector<std::vector<Edge>> beginOut;
    std::vector<std::vector<EdgeLabel>> labels;
    std::vector<int> travelTime;
};

}  // namespace TripBased
