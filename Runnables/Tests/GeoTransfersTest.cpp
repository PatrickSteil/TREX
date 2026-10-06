// Repro: makeDirectTransfersByGeoDistance must create each footpath once.
#include <iostream>

#include "../../DataStructures/Intermediate/Data.h"

int main() {
  using namespace Intermediate;
  Data data;
  const Geometry::Point p[3] = {
      Geometry::Point(Construct::LatLong, 49.000, 8.0),
      Geometry::Point(Construct::LatLong, 49.001, 8.0),   // ~111m from stop 0
      Geometry::Point(Construct::LatLong, 49.100, 8.0)};  // far away
  data.transferGraph.addVertices(3);
  for (int i = 0; i < 3; i++) {
    data.stops.emplace_back("s", p[i]);
    data.transferGraph.set(Coordinates, Vertex(i), p[i]);
  }
  for (int i = 0; i < 3; i++) {
    for (int j = i + 1; j < 3; j++) {
      data.trips.emplace_back("t", "r", 3);
      data.trips.back().stopEvents.emplace_back(StopId(i), 99, 100);
      data.trips.back().stopEvents.emplace_back(StopId(j), 200, 201);
    }
  }
  data.makeDirectTransfersByGeoDistance(20000, 4.5, false);
  const size_t edges = data.transferGraph.numEdges();
  std::cout << "edges=" << edges << " (expected 2)" << std::endl;
  return edges == 2 ? 0 : 1;
}
