#pragma once

#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/TimeTable/Data.h"
#include <numeric>
#include <vector>

namespace TimeTable {

FlatStopEventGraph FlattenTimeTable(const Data &data) {
  std::vector<std::size_t> nrEventsPerRoute(data.routes.size(), 0);
  std::vector<std::size_t> routeOffset(data.routes.size());

#pragma omp parallel for
  for (std::size_t i = 0; i < data.routes.size(); ++i) {
    const auto &route = data.routes[i];
    nrEventsPerRoute[i] =
        route.empty() ? 0 : route.size() * route.front().numberOfEvents();
  }

  // build prefix sum
  std::exclusive_scan(nrEventsPerRoute.begin(), nrEventsPerRoute.end(),
                      routeOffset.begin(), std::size_t{0});

  const std::size_t totalEvents = routeOffset.back() + nrEventsPerRoute.back();

  std::vector<std::size_t> oldMappedToNew(data.numberOfStopEventsBaseline(), 0);

#pragma omp parallel for
  for (std::size_t i = 0; i < data.routes.size(); ++i) {
    const auto &route = data.routes[i];
    std::size_t runner = 0;
    for (const auto &trip : route) {
      for (const auto &event : trip.getStopEvents()) {
        assert(event.stopEventId < oldMappedToNew.size());
        oldMappedToNew[event.stopEventId] = routeOffset[i] + runner;
        runner++;
      }
    }
  }

  // create graph and redirect edges
  EdgeListFlatStopEventGraph builder;
  builder.addVertices(totalEvents);
  builder.reserveEdges(data.edgeListStopEventGraph.numEdges());

  for (Edge edge : data.edgeListStopEventGraph.edges()) {
    Vertex newFrom = Vertex(
        oldMappedToNew[data.edgeListStopEventGraph.get(FromVertex, edge)]);
    Vertex toFrom =
        Vertex(oldMappedToNew[data.edgeListStopEventGraph.get(ToVertex, edge)]);

    assert(newFrom < totalEvents);
    assert(toFrom < totalEvents);
    builder.addEdge(newFrom, toFrom,
                    data.edgeListStopEventGraph.get(LocalLevel, edge));
  }

  FlatStopEventGraph flatGraph;
  Graph::move(std::move(builder), flatGraph);

  return flatGraph;

  // TODO
  // also return consecutive information about arrival times and stops per stop
  // event
}
} // namespace TimeTable
