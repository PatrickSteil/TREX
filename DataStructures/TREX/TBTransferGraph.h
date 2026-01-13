#include <cassert>
#include <iomanip>
#include <iostream>
#include <stdint.h>
#include <vector>

#include "../../Helpers/Console/Progress.h"
#include "TREXData.h"

namespace TripBased {

struct TripAndStopIndex {
  uint32_t tripId : 24;
  uint8_t stopIndex;

  TripAndStopIndex() : tripId(0), stopIndex(0) {}

  TripAndStopIndex(uint32_t tripId, uint8_t stopIndex)
      : tripId(tripId), stopIndex(stopIndex) {
    assert(tripId < (1 << 24));
    assert(stopIndex < (1 << 8));
  }

  uint32_t getTripId() const { return tripId; }
  uint8_t getStopIndex() const { return stopIndex; }
};

static_assert(sizeof(TripAndStopIndex) == 4,
              "Size of TripAndStopIndex is not 32 bits!");

struct EdgeInformation {
  uint16_t cellId;
  uint8_t rank;
  uint8_t hop;

  EdgeInformation() : cellId(0), rank(0), hop(0) {}
  EdgeInformation(uint16_t cellId, uint8_t rank, uint8_t hop)
      : cellId(cellId), rank(rank), hop(hop) {}

  uint16_t getCellId() const { return cellId; }
  uint8_t getRank() const { return rank; }
  uint8_t getHop() const { return hop; }
};

class TBTransferGraph {
private:
  std::vector<std::vector<std::uint32_t>> adj;
  std::vector<std::vector<TripAndStopIndex>> toVertex;
  std::vector<std::vector<EdgeInformation>> edgeInfo;

  void buildFromData(const TREXData &data) {
    const std::uint32_t numTrips = data.numberOfTrips();

    adj.resize(numTrips);
    toVertex.resize(numTrips);
    edgeInfo.resize(numTrips);

    {
      Progress progress(data.numberOfTrips());
      for (TripId trip = TripId(0); trip < data.numberOfTrips(); ++trip) {
        const std::uint32_t numStops = data.numberOfStopsInTrip(trip);

        const StopEventId firstEvent(data.firstStopEventOfTrip[trip]);
        const StopEventId endEvent(data.firstStopEventOfTrip[trip + 1]);

        assert(numStops == endEvent - firstEvent);

        adj[trip].resize(numStops + 1);

        std::uint32_t runningSum = 0;
        for (Vertex from(firstEvent); from < Vertex(endEvent); ++from) {
          assert(from - firstEvent < adj[trip].size());

          adj[trip][from - firstEvent] = runningSum;
          runningSum += data.stopEventGraph.outDegree(from);
        }
        adj[trip].back() = runningSum;

        const std::uint32_t start =
            data.stopEventGraph.beginEdgeFrom(Vertex(firstEvent));
        const std::uint32_t end =
            data.stopEventGraph.beginEdgeFrom(Vertex(endEvent));

        const std::uint32_t numOutgoingEdges = end - start;
        toVertex[trip].reserve(numOutgoingEdges);
        edgeInfo[trip].reserve(numOutgoingEdges);

        for (Edge edge(start); edge < Edge(end); ++edge) {
          const StopEventId stopEvent(data.stopEventGraph.get(ToVertex, edge));
          const StopId stop = data.getStopOfStopEvent(stopEvent);

          const TripId toTrip(data.tripOfStopEvent[stopEvent]);
          const StopIndex toStopIndex(stopEvent -
                                      data.firstStopEventOfTrip[toTrip]);
          toVertex[trip].emplace_back(toTrip, toStopIndex);
          edgeInfo[trip].emplace_back(
              (uint16_t)data.getCellIdOfStop(stop),
              (uint8_t)data.stopEventGraph.get(LocalLevel, edge),
              (uint8_t)data.stopEventGraph.get(Hop, edge));
        }

        ++progress;
      }
      progress.finished();
    }
  }

public:
  TBTransferGraph(const std::uint32_t numTrips)
      : adj(numTrips), toVertex(numTrips), edgeInfo(numTrips) {}

  TBTransferGraph(const TREXData &data) { buildFromData(data); }

  bool isValid(const uint32_t tripId) const { return tripId < adj.size(); }

  bool isValid(const uint32_t tripId, const uint8_t stopIndex) const {
    return tripId < adj.size() && stopIndex + 1 < adj[tripId].size();
  }

  const std::uint32_t beginEdge(const uint32_t tripId,
                                const uint8_t stopIndex) const {
    assert(isValid(tripId, stopIndex));
    return adj[tripId][stopIndex];
  }

  const std::vector<TripAndStopIndex> &edges(const uint32_t tripId) const {
    assert(isValid(tripId));
    return toVertex[tripId];
  }

  const std::vector<EdgeInformation> &
  edgeInformation(const uint32_t tripId) const {
    assert(isValid(tripId));
    return edgeInfo[tripId];
  }

  void showStats() const {
    std::size_t numTrips = adj.size();
    std::size_t totalStops = 0;
    std::size_t totalEdges = 0;

    std::size_t maxEdgesPerTrip = 0;
    std::size_t maxStopsPerTrip = 0;

    for (std::size_t t = 0; t < numTrips; ++t) {
      const std::size_t stops = adj[t].empty() ? 0 : adj[t].size() - 1;
      const std::size_t edges = toVertex[t].size();

      totalStops += stops;
      totalEdges += edges;

      maxStopsPerTrip = std::max(maxStopsPerTrip, stops);
      maxEdgesPerTrip = std::max(maxEdgesPerTrip, edges);
    }

    const std::size_t adjBytes =
        sizeof(std::vector<std::size_t>) * adj.size() + [&]() {
          std::size_t sum = 0;
          for (const auto &v : adj)
            sum += v.capacity() * sizeof(std::size_t);
          return sum;
        }();

    const std::size_t toVertexBytes =
        sizeof(std::vector<TripAndStopIndex>) * toVertex.size() + [&]() {
          std::size_t sum = 0;
          for (const auto &v : toVertex)
            sum += v.capacity() * sizeof(TripAndStopIndex);
          return sum;
        }();

    const std::size_t edgeInfoBytes =
        sizeof(std::vector<EdgeInformation>) * edgeInfo.size() + [&]() {
          std::size_t sum = 0;
          for (const auto &v : edgeInfo)
            sum += v.capacity() * sizeof(EdgeInformation);
          return sum;
        }();

    const std::size_t totalBytes = adjBytes + toVertexBytes + edgeInfoBytes;

    auto mb = [](std::size_t b) {
      return static_cast<double>(b) / (1024.0 * 1024.0);
    };

    std::cout << "TBTransferGraph statistics\n";
    std::cout << "--------------------------\n";
    std::cout << "Trips:                  " << numTrips << "\n";
    std::cout << "StopEvents (total):     " << totalStops << "\n";
    std::cout << "Edges (total):          " << totalEdges << "\n";
    std::cout << "Avg edges / trip:       "
              << (numTrips ? double(totalEdges) / numTrips : 0.0) << "\n";
    std::cout << "Max edges / trip:       " << maxEdgesPerTrip << "\n";
    std::cout << "Max stopEvents / trip:  " << maxStopsPerTrip << "\n\n";

    std::cout << "Memory usage:\n";
    std::cout << "  adj:                  " << mb(adjBytes) << " MB\n";
    std::cout << "  toVertex:             " << mb(toVertexBytes) << " MB\n";
    std::cout << "  edgeInfo:             " << mb(edgeInfoBytes) << " MB\n";
    std::cout << "  -------------------------\n";
    std::cout << "  Total:                " << mb(totalBytes) << " MB\n";
  }
};
} // namespace TripBased
