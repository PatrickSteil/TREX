#pragma once

#include <cassert>
#include <iostream>
#include <vector>

#include "../../../DataStructures/Container/Queue.h"
#include "../../../DataStructures/TREX/TREXData.h"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/cell_bits.h"
#include "../../../Helpers/iota_ranger.h"

#include "BuilderIBEs.h"

// TODO
// - dont consider transfers that lead to trip segements that are in the same
// n'th queue
// - 'cut' the search at borderstops
// - check that A -> B .... C -> D and A -> E .... F -> D not set one and the

namespace TripBased {

struct IBEInfo {
  StopId stopId;
  TripId tripId;
  StopIndex stopIndex;
};

struct QueueElement {
  StopEventId begin; // inclusive
  StopEventId end;   // exclusive

  QueueElement() {}
  QueueElement(const StopEventId begin, const StopEventId end)
      : begin(begin), end(end) {}
};

struct RoundToken {
  // could be tighter, currently we limit the nr of rounds to 32
  std::uint32_t layer : 8;
  std::uint32_t searchId : 24;
};

// Ensure it stays exactly 32 bits for memory density
static_assert(sizeof(RoundToken) == sizeof(std::uint32_t),
              "RoundToken must be exactly 32 bits");

class BackwardForwardSearch {
private:
  static constexpr int MAX_ROUNDS = 32;
  const TREXData &data;
  const TransferGraph &revTransferGraph;
  std::vector<std::uint32_t> &flags;
  std::uint32_t rootCellId;

  PreallocatedQueue<QueueElement> queue;

  // Changed from std::uint32_t to our custom packed struct
  std::vector<RoundToken> round;

  std::uint32_t searchId;
  std::uint32_t currentRound;

private:
  void setFlag(const Edge edge, const std::uint16_t cell) {
    assert(edge < flags.size());
    flags[edge] |= expand(cell);
  }

  bool isVisitedInCurrentSearch(const StopEventId e) const {
    return round[e].searchId == searchId;
  }

  bool isInRootCell(const StopEventId e) const {
    const StopId currentStop = data.getStopOfStopEvent(e);
    return (data.getCellIdOfStop(currentStop) == rootCellId);
  }

  std::uint32_t getRoundLayer(const StopEventId e) const {
    // Returns 0 if unvisited in this search, otherwise returns the 1-based
    // layer index
    return isVisitedInCurrentSearch(e) ? (round[e].layer + 1) : 0;
  }

  // [left, right)
  void flagTripSegment(const StopEventId left, const StopEventId right) {
    for (StopEventId e = left; e < right; ++e) {
      for (const Edge edge : data.stopEventGraph.edgesFrom(Vertex(e))) {
        StopEventId toEvent{data.stopEventGraph.get(ToVertex, edge)};
        assert(toEvent < data.numberOfStopEvents());

        std::uint32_t toRound = getRoundLayer(toEvent);
        std::uint32_t eRound = getRoundLayer(e);

        if (toRound != 0 && toRound < eRound) {
          setFlag(edge, rootCellId);
          break;
        }
      }
    }
  }

  void enqueue(const StopEventId to) {
    assert(to < data.tripOfStopEvent.size());

    // 1) did we already see it?
    if (isVisitedInCurrentSearch(to))
      return;

    // 2) does it reach into the "root" cell?
    if (isInRootCell(to))
      return;

    const TripId trip = data.tripOfStopEvent[to];
    const StopEventId first = data.firstStopEventOfTrip[trip];

    StopEventId left = to;
    round[left] = RoundToken{.layer = currentRound, .searchId = searchId};

    while (left != first && !isVisitedInCurrentSearch(StopEventId(left - 1))) {
      left = StopEventId(left - 1);
      round[left] = RoundToken{.layer = currentRound, .searchId = searchId};
    }

    queue.emplace(left, StopEventId(to + 1));
  }

  void startBackward(const IBEInfo &ibe) {
    rootCellId = data.getCellIdOfStop(ibe.stopId);

    ++searchId;

    if (searchId >= (1U << 24)) [[unlikely]] {
      std::fill(round.begin(), round.end(), RoundToken{0, 0});
      searchId = 1;
    }

    queue.clear();
    currentRound = 0;

    StopEventId startingEvent =
        data.getStopEventId(ibe.tripId, StopIndex(ibe.stopIndex));
    enqueue(startingEvent);

    std::size_t left = 0;
    std::size_t right = queue.size();

    while (left < right) {
      ++currentRound;
      assert(currentRound < MAX_ROUNDS &&
             "Number of rounds exceeded safety layer limit!");

      for (std::size_t i = left; i < right; ++i) {
        const auto &element = queue[i];
        const auto begin =
            revTransferGraph.beginEdgeFrom(Vertex(element.begin));
        const auto end = revTransferGraph.beginEdgeFrom(Vertex(element.end));

        for (auto edge : reverse_range((uint64_t)begin, (uint64_t)end)) {
          StopEventId toEvent{revTransferGraph.get(ToVertex, Edge(edge))};
          enqueue(toEvent);
        }
      }

      for (std::size_t i = left; i < right; ++i) {
        const auto &element = queue[i];
        flagTripSegment(element.begin, element.end);
      }

      left = right;
      right = queue.size();
    }
  }

public:
  BackwardForwardSearch(const TREXData &data,
                        const TransferGraph &revTransferGraph,
                        std::vector<std::uint32_t> &flags)
      : data(data), revTransferGraph(revTransferGraph), flags(flags),
        rootCellId(0), round(data.numberOfStopEvents(), RoundToken{0, 0}),
        searchId(1), currentRound(0), queue(data.numberOfStopEvents()) {}

  void run(const std::vector<IBEInfo> &IBEs, const std::size_t left,
           const std::size_t right) {
    for (std::size_t i = left; i < right; ++i) {
      startBackward(IBEs[i]);
    }
  }
};

class BackwardForwardSweeper {
private:
  const TREXData &data;
  TransferGraph revTransferGraph;
  std::vector<std::uint32_t> flags;
  BackwardForwardSearch search;

  void flipTransferGraph() {
    Graph::copy(data.stopEventGraph, revTransferGraph);
    revTransferGraph.revert();

    Graph::printInfo(revTransferGraph);
  }

  std::size_t collectAllIBEs(std::vector<IBEInfo> &IBEs) {
    std::cout << "Collect all IBEs";
    Progress progress(data.numberOfStops());

    auto inSameCell = [&](const StopId a, const StopId b) {
      return (data.getCellIdOfStop(a) == data.getCellIdOfStop(b));
    };

    std::size_t nrOfBorderStops = 0;

    bool isBorder = false;

    for (StopId stop(0); stop < data.numberOfStops(); ++stop) {
      isBorder = false;

      for (const RAPTOR::RouteSegment &route :
           data.routesContainingStop(stop)) {
        if (route.stopIndex == 0)
          continue;

        RAPTOR::RouteSegment neighbourSeg(route.routeId,
                                          StopIndex(route.stopIndex - 1));
        StopId prevStop = data.raptorData.stopOfRouteSegment(neighbourSeg);

        if (!inSameCell(stop, prevStop)) {
          for (TripId trip : data.tripsOfRoute(route.routeId)) {
            IBEs.emplace_back(stop, trip, StopIndex(route.stopIndex - 1));
          }

          isBorder = true;
        }
      }
      nrOfBorderStops += (int)isBorder;
      ++progress;
    }

    std::sort(IBEs.begin(), IBEs.end(),
              [&](const auto &left, const auto &right) {
                StopEventId leftEvent =
                    data.getStopEventId(left.tripId, left.stopIndex);
                StopEventId rightEvent =
                    data.getStopEventId(right.tripId, right.stopIndex);

                return std::tie(left.stopId, data.departureTime(leftEvent),
                                leftEvent) <
                       std::tie(right.stopId, data.departureTime(rightEvent),
                                rightEvent);
              });

    progress.finished();
    return nrOfBorderStops;
  }

public:
  BackwardForwardSweeper(const TREXData &data)
      : data(data), revTransferGraph(),
        flags(data.stopEventGraph.numEdges(), 0),
        search(data, revTransferGraph, flags) {
    flipTransferGraph();
  }

  const std::vector<std::uint32_t> &getFlags() const { return flags; }

  void setCellOwnFlags() {
    std::cout << "Setting Cell-Own Flags";
    Progress progress(data.stopEventGraph.numEdges());
    for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
      const StopId fromStop = data.getStopOfStopEvent(StopEventId(from));

#ifndef DEBUG
      const StopId toStop = data.getStopOfStopEvent(StopEventId(from));
      assert(data.getCellIdOfStop(fromStop) == data.getCellIdOfStop(toStop));
#endif // !DEBUG

      assert(edge < flags.size());
      flags[edge] |= expand(data.getCellIdOfStop(fromStop));

      ++progress;
    }

    progress.finished();
  }

  void showStats() const {
    std::uint64_t totalTransfers = flags.size();
    std::uint64_t totalBitsSet = 0;
    std::size_t zeroFlagTransfers = 0;
    std::size_t fullyAmbiguousTransfers = 0;

    std::size_t minBitsSet = 32;
    std::size_t maxBitsSet = 0;
    std::uint64_t sumBitsSet = 0;

    for (const std::uint32_t value : flags) {
      const std::size_t bitsSet =
          static_cast<std::size_t>(__builtin_popcount(value));
      totalBitsSet += bitsSet;
      sumBitsSet += bitsSet;
      minBitsSet = std::min(minBitsSet, bitsSet);
      maxBitsSet = std::max(maxBitsSet, bitsSet);

      if (value == 0)
        ++zeroFlagTransfers;
    }

    const double avgBitsSet =
        totalTransfers == 0 ? 0.0
                            : static_cast<double>(sumBitsSet) / totalTransfers;

    std::cout << "===== Flag Stats =====\n";
    std::cout << "Total transfers (edges):        " << totalTransfers << "\n";
    std::cout << "Total bits set to true:         " << totalBitsSet << " / "
              << totalTransfers * 32 << "\n";
    std::cout << "Bits set per transfer: min "
              << (totalTransfers == 0 ? 0 : minBitsSet) << ", max "
              << maxBitsSet << ", avg " << std::fixed << std::setprecision(3)
              << avgBitsSet << "\n";

    std::cout << "\nTransfers with 0 bits set (never needed by any cell): "
              << zeroFlagTransfers << " / " << totalTransfers;
    if (totalTransfers > 0) {
      std::cout << " (" << std::fixed << std::setprecision(2)
                << (100.0 * zeroFlagTransfers / totalTransfers) << "%)";
    }
    std::cout << "\n=======================\n";
  }

  void debug() {
    std::vector<IBEInfo> IBEs = {
        IBEInfo{StopId(1066), TripId(31041), StopIndex(15)}};

    search.run(IBEs, 0, 1);
  }

  void run() {
    setCellOwnFlags();

    std::vector<IBEInfo> IBEs{};
    std::size_t nrOfBorderStops = collectAllIBEs(IBEs);

    if (IBEs.empty()) {
      std::cout << "Warning: No IBEs found!\n";
      return;
    }

    std::cout << "Running Backward Forward Search\n";

    Progress progress(nrOfBorderStops);

    // this set the range [, ) that is departing at the same stop
    std::size_t left = 0;
    for (std::size_t right = 0; right < IBEs.size(); ++right) {
      const bool isLast = (right + 1 == IBEs.size());
      const bool stopChanges =
          !isLast && IBEs[right + 1].stopId != IBEs[left].stopId;

      if (isLast || stopChanges) {
        search.run(IBEs, left, right + 1);
        left = right + 1;
        ++progress;
      }
    }

    progress.finished();
  }

  static void saveFlags(const std::string &fileName,
                        const std::vector<std::uint32_t> &flags) {
    IO::serialize(fileName, flags);
  }

  static std::vector<std::uint32_t> loadFlags(const std::string &fileName) {
    std::vector<std::uint32_t> flags;
    IO::deserialize(fileName, flags);

    return flags;
  }

  void ingestFlags(const std::string &fileName) {
    flags.clear();
    IO::deserialize(fileName, flags);
  }
};

} // namespace TripBased
