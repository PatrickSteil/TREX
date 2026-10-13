// Repro: Intermediate::Data::validate() must not leave footpaths that point to
// vertices that are not stops (unused stops used to survive as plain vertices).
#include <iostream>

#include "../../DataStructures/Intermediate/Data.h"

int main() {
  using namespace Intermediate;
  Data data;
  const Geometry::Point p[3] = {
      Geometry::Point(Construct::LatLong, 49.00, 8.00),
      Geometry::Point(Construct::LatLong, 49.01, 8.00),
      Geometry::Point(Construct::LatLong, 49.00, 8.001)};
  for (int i = 0; i < 3; i++) data.stops.emplace_back("s", p[i]);
  data.transferGraph.addVertices(3);
  for (int i = 0; i < 3; i++) data.transferGraph.set(Coordinates, Vertex(i), p[i]);
  // Stop 2 is not served by any trip, but has a footpath to/from stop 0.
  data.transferGraph.addEdge(Vertex(0), Vertex(2)).set(TravelTime, 10);
  data.transferGraph.addEdge(Vertex(2), Vertex(0)).set(TravelTime, 10);
  data.trips.emplace_back("t", "r", 3);
  data.trips.back().stopEvents.emplace_back(StopId(0), 99, 100);
  data.trips.back().stopEvents.emplace_back(StopId(1), 200, 201);

  data.validate();

  int bad = 0;
  for (const Vertex v : data.transferGraph.vertices()) {
    if (!data.isStop(v)) continue;
    for (const Edge e : data.transferGraph.edgesFrom(v)) {
      if (!data.isStop(data.transferGraph.get(ToVertex, e))) ++bad;
    }
  }
  std::cout << "stops=" << data.numberOfStops()
            << " vertices=" << data.transferGraph.numVertices()
            << " footpaths to non-stops=" << bad << std::endl;
  return bad == 0 ? 0 : 1;
}
