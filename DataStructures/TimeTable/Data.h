#pragma once

#include "../../Algorithms/UnionFind.h"
#include "../../Helpers/Assert.h"
#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/Types.h"
#include "../Graph/Graph.h"
#include "../RAPTOR/Data.h"
#include "Entities/Trip.h"

#include <cassert>
#include <iostream>
#include <vector>

namespace TimeTable {

class Data {
public:
  RAPTOR::Data raptorData;
  std::vector<uint16_t> cellIds;

  std::vector<std::vector<Trip>> routes;

  UnionFind unionFind;
  StaticGraphWithWeightsAndCoordinates layoutGraph;

  StaticGraphWithReverseEdge fullStopEventGraph;
  EdgeListFlatStopEventGraph edgeListStopEventGraph;

  void buildFromRAPTOR() noexcept {
    cellIds.clear();
    routes.clear();
    unionFind.clear();

    cellIds.assign(raptorData.numberOfStops(), 0);
    routes.assign(raptorData.numberOfRoutes(), std::vector<Trip>{});
    unionFind.reset(raptorData.numberOfStops());

    TripId globalTripId(0);
    StopEventId stopEventIdBaseline(0);

    for (RouteId route : raptorData.routes()) {
      const std::size_t nrTrips = raptorData.numberOfTripsInRoute(route);
      const std::size_t nrStops = raptorData.numberOfStopsInRoute(route);
      const StopId *stopsOfRoute = raptorData.stopArrayOfRoute(route);

      assert(route < routes.size());

      routes[route].reserve(nrTrips);

      for (std::size_t tId = 0; tId < nrTrips; ++tId) {
        std::vector<StopEvent> curTrip;
        curTrip.reserve(nrStops);

        std::size_t stopIdx = 0;
        for (const RAPTOR::StopEvent &stopE :
             raptorData.stopEventsOfTrip(route, tId)) {
          curTrip.emplace_back(stopEventIdBaseline, stopsOfRoute[stopIdx],
                               Time(stopE.arrivalTime),
                               Time(stopE.departureTime));
          ++stopIdx;
          ++stopEventIdBaseline;
        }

        routes[route].emplace_back(globalTripId, curTrip);
        globalTripId++;
      }
    }
  }

  inline void createCompactLayoutGraph() {
    std::cout << "Computing the Compact Layout Graph!" << std::endl;

    unionFind.clear();
    std::vector<int> weightOfNodes(numberOfStops(), 1);

    for (const auto [edge, from] :
         raptorData.transferGraph.edgesWithFromVertex()) {
      Vertex toStop = raptorData.transferGraph.get(ToVertex, edge);
      if (unionFind(from) != unionFind(toStop)) {
        auto newWeight =
            weightOfNodes[unionFind(from)] + weightOfNodes[unionFind(toStop)];
        unionFind(from, toStop);
        weightOfNodes[unionFind(from)] = newWeight;
      }
    }

    DynamicGraphWithWeightsAndCoordinates dynamicLayoutGraph;
    dynamicLayoutGraph.clear();
    dynamicLayoutGraph.addVertices(numberOfStops());

    for (Vertex vertex : dynamicLayoutGraph.vertices()) {
      dynamicLayoutGraph.set(Weight, vertex, 0);
      if (unionFind(vertex) == (int)vertex) {
        dynamicLayoutGraph.set(Weight, vertex, weightOfNodes[vertex]);
      }
      dynamicLayoutGraph.set(Coordinates, vertex,
                             raptorData.stopData[vertex].coordinates);
    }

    Progress progress(raptorData.numberOfRoutes() +
                      raptorData.transferGraph.numEdges());

    for (const RouteId route : raptorData.routes()) {
      SubRange<std::vector<StopId>> stopsInCurrentRoute =
          raptorData.stopsOfRoute(route);
      size_t numberOfTrips = raptorData.numberOfTripsInRoute(route);

      for (size_t i(1); i < stopsInCurrentRoute.size(); ++i) {
        AssertMsg(dynamicLayoutGraph.isVertex(stopsInCurrentRoute[i]),
                  "Current Stop is not a valid vertex!\n");
        Vertex fromVertexUnion = Vertex(unionFind(stopsInCurrentRoute[i - 1]));
        Vertex toVertexUnion = Vertex(unionFind(stopsInCurrentRoute[i]));

        if (fromVertexUnion == toVertexUnion)
          continue;

        Edge edgeHeadTail =
            dynamicLayoutGraph.findEdge(fromVertexUnion, toVertexUnion);
        if (edgeHeadTail != noEdge) {
          dynamicLayoutGraph.set(Weight, edgeHeadTail,
                                 dynamicLayoutGraph.get(Weight, edgeHeadTail) +
                                     numberOfTrips);
          Edge edgeTailHead =
              dynamicLayoutGraph.findEdge(toVertexUnion, fromVertexUnion);
          AssertMsg(edgeTailHead != noEdge,
                    "A reverse edge is missing between "
                        << stopsInCurrentRoute[i - 1] << " and "
                        << stopsInCurrentRoute[i] << "\n");
          dynamicLayoutGraph.set(Weight, edgeTailHead,
                                 dynamicLayoutGraph.get(Weight, edgeTailHead) +
                                     numberOfTrips);

        } else {
          dynamicLayoutGraph.addEdge(fromVertexUnion, toVertexUnion)
              .set(Weight, 1);
          dynamicLayoutGraph.addEdge(toVertexUnion, fromVertexUnion)
              .set(Weight, 1);
        }
      }
      ++progress;
    }

    progress.finished();

    AssertMsg(!(dynamicLayoutGraph.edges().size() & 1),
              "The number of edges is uneven, thus we check that every edge "
              "has a reverse edge in the graph!\n");

    uint64_t totalEdgeWeight(0);
    for (Edge edge : dynamicLayoutGraph.edges()) {
      totalEdgeWeight += dynamicLayoutGraph.get(Weight, edge);
    }

    if (totalEdgeWeight > UINT32_MAX)
      std::cout << "** The total sum of all edge weights exceeds 32 bits **"
                << std::endl;

    layoutGraph.clear();
    Graph::move(std::move(dynamicLayoutGraph), layoutGraph);
    std::cout << "The Layout Graph looks like this:" << std::endl;
    layoutGraph.printAnalysis();
  }

  inline void applyGlobalIDs(const std::vector<uint64_t> &globalIds) noexcept {
    if (layoutGraph.numEdges() == 0 || layoutGraph.numVertices() == 0) {
      std::cout << "please built the layoutgraph and the union find "
                   "datastructures first!"
                << std::endl;
      return;
    }
    for (size_t i(0); i < numberOfStops(); ++i) {
      AssertMsg(static_cast<size_t>(unionFind(i)) < globalIds.size(),
                "unionFind is out of bounds!");
      AssertMsg(layoutGraph.get(Weight, Vertex(unionFind(i))) > 0,
                "The corresponding vertex weight is zero?");
      cellIds[i] = static_cast<uint16_t>(globalIds[unionFind(i)]);
    }
  }

  inline void readPartitionFile(const std::string &fileName) {
    std::vector<uint64_t> globalIds(numberOfStops(), 0);
    std::fstream file(fileName);

    if (file.is_open()) {
      uint64_t globalId(0);
      size_t index(0);

      while (file >> globalId) {
        globalIds[index] = globalId;
        ++index;
      }

      file.close();

      std::cout << "Read " << String::prettyInt(index) << " many IDs!"
                << std::endl;
    } else {
      std::cerr << "Unable to open the file: " << fileName << std::endl;
    }

    applyGlobalIDs(globalIds);
  }

  void printInfo() const {
    std::size_t totalTrips = 0;
    std::size_t totalStopEvents = 0;
    std::size_t maxTripsInRoute = 0;
    std::size_t maxStopsInTrip = 0;

    for (std::size_t route(0); route < routes.size(); ++route) {
      const auto &routeTrips = routes[route];
      totalTrips += routeTrips.size();
      maxTripsInRoute = std::max(maxTripsInRoute, routeTrips.size());

      for (const Trip &trip : routeTrips) {
        const auto &stops = trip.getStopEvents();
        totalStopEvents += stops.size();
        maxStopsInTrip = std::max(maxStopsInTrip, stops.size());
      }
    }

    std::cout << "TimeTable info\n";
    std::cout << "-----------------------------\n";
    std::cout << "Stops              : " << raptorData.numberOfStops() << "\n";
    std::cout << "Routes             : " << routes.size() << "\n";
    std::cout << "Trips              : " << totalTrips << "\n";
    std::cout << "Stop events        : " << totalStopEvents << "\n";
    std::cout << "Max trips / route  : " << maxTripsInRoute << "\n";
    std::cout << "Max stops / trip   : " << maxStopsInTrip << "\n";
  }

  std::size_t numberOfStops() const { return raptorData.numberOfStops(); }
  std::size_t numberOfRoutes() const { return raptorData.numberOfRoutes(); }
  std::size_t numberOfStopEventsBaseline() const {
    return raptorData.numberOfStopEvents();
  }

  // Constructors
  Data() {}

  Data(const std::string &fileName) { deserialize(fileName); }

  Data(const RAPTOR::Data &raptorData)
      : raptorData(raptorData), cellIds(), routes(), unionFind() {
    buildFromRAPTOR();
    createCompactLayoutGraph();
  }

  inline void serialize(const std::string &fileName) const noexcept {
    raptorData.serialize(fileName + ".raptor");
    IO::serialize(fileName, cellIds, routes, unionFind, layoutGraph);
    fullStopEventGraph.writeBinary(fileName + ".full.graph");
    edgeListStopEventGraph.writeBinary(fileName + ".flat.graph");
  }

  inline void deserialize(const std::string &fileName) noexcept {
    raptorData.deserialize(fileName + ".raptor");
    IO::deserialize(fileName, cellIds, routes, unionFind, layoutGraph);
    fullStopEventGraph.readBinary(fileName + ".full.graph");
    edgeListStopEventGraph.readBinary(fileName + ".flat.graph");
  }
};

} // namespace TimeTable
