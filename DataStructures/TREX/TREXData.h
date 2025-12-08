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

#include <cmath>
#include <numeric>
#include <string>

#include "../../Algorithms/UnionFind.h"
#include "../../Helpers/Assert.h"
#include "../../Helpers/Ranges/SubRange.h"
#include "../../Helpers/String/String.h"
#include "../RAPTOR/Data.h"
#include "../RAPTOR/Entities/RouteSegment.h"
#include "../TripBased/Data.h"

namespace TripBased {

class TREXData : public Data {
public:
  TREXData(const RAPTOR::Data &raptor, const int numLevels)
      : Data(raptor), numberOfLevels(numLevels), unionFind(numberOfStops()),
        layoutGraph(), localLevelOfEvent(raptor.numberOfStopEvents(), 0),
        cellIds(raptor.numberOfStops(), 0) {}

  TREXData(const std::string &fileName) { deserialize(fileName); }

public:
  inline void addInformationToStopEventGraph() noexcept {
    std::vector<uint8_t> zeroLevels(stopEventGraph.numEdges(), 0);
    stopEventGraph.get(LocalLevel).swap(zeroLevels);

    /* std::vector<uint8_t> initHops(stopEventGraph.numEdges(), 1); */
    /* stopEventGraph.get(Hop).swap(initHops); */
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

  inline void applyGlobalIDs(std::vector<uint64_t> &globalIds) noexcept {
    /*
    std::vector<Vertex> stopToVertexMapping(numberOfStops(), noVertex);

    for (Vertex v : layoutGraph.vertices()) {
        stopToVertexMapping[layoutGraph.get(Size, v)] = v;
    }
    */

    // now set the correct cellIds
    for (size_t i(0); i < numberOfStops(); ++i) {
      /*
      AssertMsg((size_t)unionFind(i) < stopToVertexMapping.size(),
          "unionFind[i] is out of bounds!");
      AssertMsg((size_t)stopToVertexMapping[unionFind(i)] < globalIds.size(),
          "stopToVertexMapping[unionFind[i]] is out of bounds!");
      cellIds[i] = globalIds[stopToVertexMapping[unionFind(i)]];
      */
      AssertMsg(static_cast<size_t>(unionFind(i)) < globalIds.size(),
                "unionFind is out of bounds!");
      AssertMsg(layoutGraph.get(Weight, Vertex(unionFind(i))) > 0,
                "The corresponding vertex weight is zero?");
      cellIds[i] = globalIds[unionFind(i)];
    }

    AssertMsg(assertNoCutTransfers(), "Footpath has been cut!");
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

    // size contains the original vertex
    // TODO remove size attribute
    DynamicGraphWithWeightsAndCoordinates dynamicLayoutGraph;
    dynamicLayoutGraph.clear();
    dynamicLayoutGraph.addVertices(numberOfStops());

    for (Vertex vertex : dynamicLayoutGraph.vertices()) {
      dynamicLayoutGraph.set(Weight, vertex, 0);
      /* dynamicLayoutGraph.set(Size, vertex, vertex); */
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
      /* dynamicLayoutGraph.set(Weight, edge,
       * std::log(dynamicLayoutGraph.get(Weight, edge))); */
      totalEdgeWeight += dynamicLayoutGraph.get(Weight, edge);
    }

    if (totalEdgeWeight > UINT32_MAX)
      std::cout << "** The total sum of all edge weights exceeds 32 bits **"
                << std::endl;

    /* dynamicLayoutGraph.deleteIsolatedVertices(); */
    /* dynamicLayoutGraph.packEdges(); */
    layoutGraph.clear();
    Graph::move(std::move(dynamicLayoutGraph), layoutGraph);
    std::cout << "The Layout Graph looks like this:" << std::endl;
    layoutGraph.printAnalysis();
  }

  inline void writeLayoutGraphToMETIS(const std::string fileName,
                                      const bool writeGRAPHML = true) {
    std::cout << "Write Layout Graph to file " << fileName << std::endl;
    std::cout << "[Num Vertices: " << layoutGraph.numVertices()
              << ", Num Edges: " << layoutGraph.numEdges() << "]" << std::endl;
    Progress progressWriting(layoutGraph.numVertices());

    unsigned long n = layoutGraph.numVertices();
    unsigned long m =
        layoutGraph.numEdges() >> 1; // halbieren weil METIS das braucht

    std::ofstream file(fileName + ".metis");

    // n [NUMBER of nodes]  m [NUMBER of edges]     f [int]
    // f values:
    /*
            f values:
    1 :     edge-weighted graph
    10:     node-weighted graph
    11:     edge & node - weighted graph
     */

    file << n << " " << m << " 11";

    for (Vertex vertex : layoutGraph.vertices()) {
      file << "\n" << layoutGraph.get(Weight, vertex) << " ";
      for (Edge edge : layoutGraph.edgesFrom(vertex)) {
        file << layoutGraph.get(ToVertex, edge).value() + 1 << " "
             << layoutGraph.get(Weight, edge) << " ";
      }
      progressWriting++;
    }

    file.close();
    progressWriting.finished();

    if (writeGRAPHML) {
      Graph::toGML(fileName, layoutGraph);
    }

    layoutGraph.writeBinary(fileName);
  }

  inline void writeLayoutGraphToHypMETIS(const std::string fileName) {
    std::cout << "Write Layout Graph to file " << fileName << std::endl;
    Progress progressWriting(layoutGraph.numVertices() +
                             layoutGraph.numEdges());

    unsigned long n = layoutGraph.numVertices();
    unsigned long m =
        layoutGraph.numEdges(); // halbieren weil METIS das braucht

    std::ofstream file(fileName + ".hypmetis");

    // see here https://course.ece.cmu.edu/~ee760/760docs/hMetisManual.pdf
    // n [NUMBER of nodes]  m [NUMBER of edges]     f [int]
    // f values:
    /*
            f values:
    1 :     edge-weighted graph
    10:     node-weighted graph
    11:     edge & node - weighted graph
     */

    file << m << " " << n << " 11";

    for (const auto [edge, from] : layoutGraph.edgesWithFromVertex()) {
      file << "\n"
           << layoutGraph.get(Weight, edge) << " " << (int)(from + 1) << " "
           << (layoutGraph.get(ToVertex, edge).value() + 1);
      progressWriting++;
    }

    for (Vertex vertex : layoutGraph.vertices()) {
      file << "\n" << layoutGraph.get(Weight, vertex) << " ";
      progressWriting++;
    }

    file.close();
    progressWriting.finished();
  }

  // Getter
  inline int getNumberOfLevels() const noexcept { return numberOfLevels; }

  inline int getNumberOfCellsPerLevel() const noexcept { return 2; }

  inline uint64_t getCellIdOfStop(const StopId &stop) const noexcept {
    AssertMsg(isStop(stop), "Stop is not a stop!");

    return cellIds[stop];
  }

  inline SubRange<std::vector<RAPTOR::RouteSegment>>
  routesContainingStop(const StopId stop) const noexcept {
    return raptorData.routesContainingStop(stop);
  }

  inline uint8_t &getLocalLevelOfEvent(StopEventId event) noexcept {
    AssertMsg(event < localLevelOfEvent.size(), "Event is out of bounds!");
    return localLevelOfEvent[event];
  }

  inline std::vector<StopEventId>
  getStopEventOfStopInRoute(const StopId stop, const RouteId route) noexcept {
    std::vector<StopEventId> result;
    result.reserve(raptorData.numberOfTripsInRoute(route));

    auto stopsOfRoute = raptorData.stopsOfRoute(route);
    Range<TripId> tripsOfRoute = Data::tripsOfRoute(route);

    for (size_t i(0); i < stopsOfRoute.size(); ++i) {
      if (stopsOfRoute[i] == stop) {
        for (TripId trip : tripsOfRoute) {
          result.push_back(getStopEventId(trip, StopIndex(i)));
        }
        return result;
      }
    }
    AssertMsg(false, "Wait, stop was not in the route??");
    return result;
  }

  inline void printInfo() const noexcept {
    Data::printInfo();
    std::cout << "   Number of Levels:         " << std::setw(12)
              << (int)numberOfLevels << std::endl;
    std::cout << "   Cells per Level:          " << std::setw(12) << "2"
              << std::endl;
  }

  // Serialization
  inline void serialize(const std::string &fileName) const noexcept {
    Data::serialize(fileName + ".trip");
    IO::serialize(fileName, numberOfLevels, unionFind, layoutGraph,
                  localLevelOfEvent, cellIds);
    stopEventGraph.writeBinary(fileName + ".trip.graph");
  }

  inline void deserialize(const std::string &fileName) noexcept {
    Data::deserialize(fileName + ".trip");
    IO::deserialize(fileName, numberOfLevels, unionFind, layoutGraph,
                    localLevelOfEvent, cellIds);
    stopEventGraph.readBinary(fileName + ".trip.graph");
  }

  inline void writePartitionToCSV(const std::string &fileName) noexcept {
    std::ofstream file(fileName);

    file << "StopID,CellId\n";

    for (StopId stop(0); stop < cellIds.size(); ++stop) {
      file << (int)stop << "," << (int)cellIds[stop] << "\n";
    }
    file.close();
  }

  inline void writeUnionFindToFile(const std::string &fileName) noexcept {
    std::ofstream file(fileName);

    file << "StopID,CorrespondingStopID\n";

    for (StopId stop(0); stop < numberOfStops(); ++stop) {
      assert(static_cast<size_t>(unionFind(stop)) < numberOfStops());
      file << (int)stop << "," << (int)unionFind(stop) << "\n";
    }
    file.close();
  }

  // Assert that no transfer is cut
  inline bool assertNoCutTransfers() noexcept {
    for (const auto [edge, from] :
         raptorData.transferGraph.edgesWithFromVertex()) {
      Vertex toVertex = raptorData.transferGraph.get(ToVertex, edge);
      if (getCellIdOfStop(StopId(from)) != getCellIdOfStop(StopId(toVertex))) {
        std::cout << "**** A cut between footpath from " << from << " and "
                  << toVertex
                  << "! The respective union find: " << unionFind(from)
                  << " and " << unionFind(toVertex) << std::endl;
        return false;
      }
    }
    return true;
  }

  inline bool isLevel(int level) const { return level < numberOfLevels; }

  inline void setNumberOfLevels(int level) noexcept { numberOfLevels = level; }

  inline void
  writeLocalLevelOfTripsToCSV(const std::string &fileName) const noexcept {
    std::vector<size_t> outgoingLocalLevelOfTrip(numberOfTrips(), 0);
    std::vector<size_t> incomingLocalLevelOfTrip(numberOfTrips(), 0);
    std::vector<std::vector<size_t>> numOfTransferPerLevel(numberOfTrips());

    for (TripId trip(0); trip < numberOfTrips(); ++trip) {
      numOfTransferPerLevel[trip].assign(numberOfLevels + 1, 0);

      Edge start =
          stopEventGraph.beginEdgeFrom(Vertex(firstStopEventOfTrip[trip]));
      Edge end =
          stopEventGraph.beginEdgeFrom(Vertex(firstStopEventOfTrip[trip + 1]));

      for (Edge e(start); e < end; ++e) {
        outgoingLocalLevelOfTrip[trip] =
            std::max(outgoingLocalLevelOfTrip[trip],
                     (size_t)stopEventGraph.get(LocalLevel, e));

        TripId toTrip =
            tripOfStopEvent[StopEventId(stopEventGraph.get(ToVertex, e))];
        incomingLocalLevelOfTrip[toTrip] =
            std::max(incomingLocalLevelOfTrip[toTrip],
                     (size_t)stopEventGraph.get(LocalLevel, e));

        ++numOfTransferPerLevel[trip][stopEventGraph.get(LocalLevel, e)];
      }
    }

    std::ofstream file(fileName);

    file << "TripID,Outgoing LocalLevel,Incoming LocalLevel";

    for (int l(0); l <= numberOfLevels; ++l) {
      file << ",Level " << l;
    }
    file << "\n";

    for (TripId trip(0); trip < numberOfTrips(); ++trip) {
      file << (int)trip << "," << outgoingLocalLevelOfTrip[trip] << ","
           << incomingLocalLevelOfTrip[trip];
      for (int l(0); l <= numberOfLevels; ++l) {
        file << "," << numOfTransferPerLevel[trip][l];
      }
      file << "\n";
    }
    file.close();
  }

  void exportStopFailureDistribution(const std::string &fileName) const {
    if (numberOfLevels <= 0) {
      std::cout << "Number Of Levels is not positive!" << std::endl;
      return;
    }

    if (numberOfLevels > 32) {
      std::cout << "Number Of Levels is too large!" << std::endl;
      return;
    }
    std::cout << "Load and export number of events per stop" << std::endl;
    std::vector<std::vector<size_t>> importancePerStop(
        numberOfStops(), std::vector<size_t>(numberOfLevels + 1, 0));

    for (StopEventId e(0); e < raptorData.numberOfStopEvents(); ++e) {
      uint8_t level = localLevelOfEvent[e];
      StopId s = getStopOfStopEvent(e);

      assert(s < raptorData.numberOfStopEvents());
      assert(level <= numberOfLevels);

      importancePerStop[s][level] += 1;
    }

    std::ofstream file(fileName);
    file << "StopId,StopName";

    for (int l = 0; l <= numberOfLevels; ++l) {
      file << ",EventsL" << l;
    }

    file << "\n";

    for (const StopId s : stops()) {
      file << (int)s << ",\"" << raptorData.stopData[s].name << "\"";

      for (auto nr : importancePerStop[s]) {
        file << "," << nr;
      }
      file << "\n";
    }

    file.close();
  }

public:
  int numberOfLevels;
  UnionFind unionFind;
  StaticGraphWithWeightsAndCoordinates layoutGraph;

  // we also keep track of the highest locallevel of an event
  std::vector<uint8_t> localLevelOfEvent;

  // for the 2' cell ids
  std::vector<uint16_t> cellIds;
};

} // namespace TripBased
