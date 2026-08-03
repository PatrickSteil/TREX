#pragma once

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "../../../DataStructures/Container/Queue.h"
#include "../../../DataStructures/TREX/TREXData.h"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/cell_bits.h"
#include "../../../Helpers/iota_ranger.h"

#include "BuilderIBEs.h"

namespace TripBased {
void setFlag(std::vector<std::uint64_t> &flags, const Edge edge,
             const std::uint16_t cell) {
  assert(edge < flags.size());
  assert(cell < 64);
  flags[edge] |= (1ULL << cell);
}

struct IBEInfo {
  StopId stopId;
  TripId tripId;
  StopIndex stopIndex;

  std::uint16_t stopCellId;
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

static_assert(sizeof(RoundToken) == sizeof(std::uint32_t),
              "RoundToken must be exactly 32 bits");

class BackwardForwardSearch {
private:
  static constexpr int MAX_ROUNDS = 32;
  const TREXData &data;
  const TransferGraph &revTransferGraph;
  std::vector<std::uint64_t> &flags;

  const std::vector<std::uint16_t> &stopEventCellId;
  std::uint16_t targetCellId;

  PreallocatedQueue<QueueElement> queue;

  std::vector<RoundToken> round;
  std::vector<std::uint32_t> batchVisited;
  std::uint32_t batchId;
  std::uint32_t searchId;
  std::uint32_t currentRound;

private:
  bool isVisitedInCurrentSearch(const StopEventId e) const {
    return round[e].searchId == searchId;
  }

  bool isVisitedInBatch(const StopEventId e) const {
    return batchVisited[e] == batchId;
  }

  std::uint32_t getRoundLayer(const StopEventId e) const {
    return isVisitedInCurrentSearch(e) ? round[e].layer : 0;
  }

  void flagSingleEvent(const StopEventId e) {
    const bool verbose = (data.tripOfStopEvent[e] == TripId(39076) ||
                          data.tripOfStopEvent[e] == TripId(66492));

    if (verbose)
      std::cout << "> flag event " << (int)e << std::endl;
    const std::uint32_t eRound = getRoundLayer(e);

    for (const Edge edge : data.stopEventGraph.edgesFrom(Vertex(e))) {
      StopEventId toEvent{data.stopEventGraph.get(ToVertex, edge)};
      assert(toEvent < data.numberOfStopEvents());
      const std::uint32_t toRound = getRoundLayer(toEvent);

      if (toRound != 0 && toRound < eRound) {
        if (verbose)
          std::cout << "> flag edge to " << (int)toEvent << " from " << (int)e
                    << std::endl;
        setFlag(flags, edge, targetCellId);
        break;
      }
    }
  }

  template <bool IN_FIRST_ROUND = false> void enqueue(const StopEventId to) {
    assert(to < data.tripOfStopEvent.size());

    const bool verbose = (data.tripOfStopEvent[to] == TripId(39076) ||
                          data.tripOfStopEvent[to] == TripId(66492));

    if (verbose)
      std::cout << "> enqueue " << (int)to << std::endl;

    if (isVisitedInBatch(to)) {
      if (verbose)
        std::cout << "hitting already seen, enqueing one element\n";
      round[to] = RoundToken{.layer = currentRound, .searchId = searchId};
      batchVisited[to] = batchId;
      // queue.emplace(to, to);
      flagSingleEvent(to);
      return;
    }

    const TripId trip = data.tripOfStopEvent[to];
    const StopEventId first = data.firstStopEventOfTrip[trip];

    if constexpr (!IN_FIRST_ROUND) {
      if (to == first) {
        return;
      }
    }

    assert(IN_FIRST_ROUND || to > first);

    StopEventId left = to;

    while (!isVisitedInBatch(left)) {
      round[left] = RoundToken{.layer = currentRound, .searchId = searchId};
      batchVisited[left] = batchId;
      if (left == first)
        break;
      left = StopEventId(left - 1);
    }
    if (verbose)
      std::cout << "> Left: " << left << ", right: " << to << "\n";
    queue.emplace(left, StopEventId(to));
  }

  void startBackward(const IBEInfo &ibe) {
    targetCellId = ibe.stopCellId;

    ++searchId;

    if (searchId >= (1U << 24)) [[unlikely]] {
      std::fill(round.begin(), round.end(), RoundToken{0, 0});
      searchId = 1;
    }

    queue.clear();
    currentRound = 1;

    StopEventId startingEvent =
        data.getStopEventId(ibe.tripId, StopIndex(ibe.stopIndex));
    enqueue<true>(startingEvent);

    std::size_t left = 0;
    std::size_t right = queue.size();

    while (left < right && (++currentRound < MAX_ROUNDS)) {
      assert(currentRound < MAX_ROUNDS);

      for (std::size_t i = left; i < right; ++i) {
        const auto &element = queue[i];

        const auto begin =
            revTransferGraph.beginEdgeFrom(Vertex(element.begin));
        const auto end = revTransferGraph.beginEdgeFrom(Vertex(element.end));

        // Note that this range from (right, left]
        for (auto edge : reverse_range((uint64_t)begin, (uint64_t)end)) {
          StopEventId toEvent{revTransferGraph.get(ToVertex, Edge(edge))};
          enqueue(toEvent);
        }
      }

      for (std::size_t i = left; i < right; ++i) {
        const auto &element = queue[i];
        for (StopEventId e = element.begin; e <= element.end; ++e) {
          flagSingleEvent(e);
        }
      }

      left = right;
      right = queue.size();
    }
  }

public:
  BackwardForwardSearch(const TREXData &data,
                        const TransferGraph &revTransferGraph,
                        std::vector<std::uint64_t> &flags,
                        const std::vector<std::uint16_t> &stopEventCellId)
      : data(data), revTransferGraph(revTransferGraph), flags(flags),
        stopEventCellId(stopEventCellId), targetCellId(0),
        round(data.numberOfStopEvents(), RoundToken{0, 0}),
        batchVisited(data.numberOfStopEvents(), 0), batchId(0), searchId(1),
        currentRound(0), queue(data.numberOfStopEvents()) {}

  void run(const std::vector<IBEInfo> &IBEs, const std::size_t left,
           const std::size_t right) {
    assert(left < right);
    assert(right <= IBEs.size());

    ++batchId;
    if (batchId == std::numeric_limits<std::uint32_t>::max()) [[unlikely]] {
      std::fill(batchVisited.begin(), batchVisited.end(), 0);
      batchId = 1;
    }

    for (std::size_t i = left; i < right; ++i) {
      startBackward(IBEs[i]);
    }
  }
};

class BackwardForwardSweeper {
private:
  const TREXData &data;
  TransferGraph revTransferGraph;
  std::vector<std::uint64_t> flags;

  std::vector<std::uint16_t> stopEventCellId;

  BackwardForwardSearch search;

  void flipTransferGraph() {
    Graph::copy(data.stopEventGraph, revTransferGraph);
    revTransferGraph.revert();

    Graph::printInfo(revTransferGraph);
  }

  void precomputeStopEventCellIds() {
    stopEventCellId.resize(data.numberOfStopEvents());
    for (StopEventId e(0); e < data.numberOfStopEvents(); ++e) {
      const StopId stop = data.getStopOfStopEvent(e);
      stopEventCellId[e] = data.getCellIdOfStop(stop);
    }
  }

  std::size_t collectAllIBEs(std::vector<IBEInfo> &IBEs) {
    std::cout << "Collect all IBEs (level 0)";
    Progress progress(data.numberOfStops());

    std::size_t nrOfBorderStops = 0;
    bool isBorder = false;

    // for (StopId stop(0); stop < data.numberOfStops(); ++stop) {
    for (StopId stop : {StopId(3301)}) {
      isBorder = false;
      const std::uint16_t stopCellId = data.getCellIdOfStop(stop);

      for (const RAPTOR::RouteSegment &route :
           data.routesContainingStop(stop)) {
        if (route.stopIndex == 0)
          continue;

        RAPTOR::RouteSegment neighbourSeg(route.routeId,
                                          StopIndex(route.stopIndex - 1));
        StopId prevStop = data.raptorData.stopOfRouteSegment(neighbourSeg);
        const std::uint16_t neighborCellId = data.getCellIdOfStop(prevStop);

        if (stopCellId != neighborCellId) {
          for (TripId trip : data.tripsOfRoute(route.routeId)) {
            IBEs.push_back(IBEInfo{stop, trip, StopIndex(route.stopIndex - 1),
                                   stopCellId});
          }
          isBorder = true;
        }
      }
      nrOfBorderStops += isBorder;
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

  void setOwnGroupFlags() {
    std::cout << "Setting own-group flags";
    Progress progress(data.stopEventGraph.numEdges());

    for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
      const std::uint16_t fromGroup = stopEventCellId[from];
      const std::uint16_t toGroup =
          stopEventCellId[data.stopEventGraph.get(ToVertex, edge)];
      assert(fromGroup == toGroup);
      setFlag(flags, edge, fromGroup);
      ++progress;
    }

    progress.finished();
  }

public:
  BackwardForwardSweeper(const TREXData &data)
      : data(data), revTransferGraph(),
        flags(data.stopEventGraph.numEdges(), 0),
        search(data, revTransferGraph, flags, stopEventCellId) {
    flipTransferGraph();
  }

  const std::vector<std::uint64_t> &getFlags() const { return flags; }

  void writeFlagsToCSV(const std::string &fileName) const {
    std::ofstream out(fileName);
    if (!out) {
      std::cerr << "Could not open file '" << fileName
                << "' for writing flags CSV.\n";
      return;
    }

    out << "fromStopEvent,toStopEvent,flags\n";

    for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
      const Vertex to = data.stopEventGraph.get(ToVertex, edge);

      out << static_cast<std::size_t>(from) << ','
          << static_cast<std::size_t>(to) << ',' << std::bitset<64>(flags[edge])
          << "\n";
    }

    out.close();
    std::cout << "Wrote flags CSV to '" << fileName << "' (" << flags.size()
              << " rows).\n";
  }

  void showPerEventPruningStats() const {
    const std::size_t numEvents = data.numberOfStopEvents();
    const std::size_t numCells = 32; // adjust if this changes

    std::size_t eventsConsidered = 0;
    double sumAvgFraction = 0.0;

    std::size_t eventsFullyDense = 0;  // avg popcount == numCells
    std::size_t eventsFullySparse = 0; // avg popcount == 1 (own-cell only)

    // Bucket by average fraction of cells needing this event's transfers, 10%
    // wide.
    std::array<std::size_t, 11> fractionBuckets{};

    for (StopEventId e(0); e < numEvents; ++e) {
      std::size_t outDegree = 0;
      std::size_t totalBits = 0;

      for (const Edge edge : data.stopEventGraph.edgesFrom(Vertex(e))) {
        ++outDegree;
        totalBits +=
            static_cast<std::size_t>(__builtin_popcountll(flags[edge]));
      }

      if (outDegree == 0)
        continue;

      ++eventsConsidered;
      const double avgBitsForEvent = static_cast<double>(totalBits) / outDegree;
      const double avgFraction = avgBitsForEvent / numCells;
      sumAvgFraction += avgFraction;

      if (avgBitsForEvent >= numCells - 0.001)
        ++eventsFullyDense;
      if (avgBitsForEvent <= 1.001)
        ++eventsFullySparse;

      std::size_t bucket = static_cast<std::size_t>(avgFraction * 10.0);
      if (bucket > 10)
        bucket = 10;
      ++fractionBuckets[bucket];
    }

    std::cout << "===== Per-Event Real Pruning Stats =====\n";
    std::cout << "Events considered: " << eventsConsidered << "\n";
    if (eventsConsidered > 0) {
      std::cout << "Avg fraction of cells needing an event's transfers: "
                << std::fixed << std::setprecision(3)
                << (sumAvgFraction / eventsConsidered) << "\n";
      std::cout
          << "Events where transfers are needed by ALL cells (no pruning): "
          << eventsFullyDense << " / " << eventsConsidered << " ("
          << std::setprecision(2)
          << (100.0 * eventsFullyDense / eventsConsidered) << "%)\n";
      std::cout << "Events where transfers are needed by ONLY own cell (max "
                   "pruning): "
                << eventsFullySparse << " / " << eventsConsidered << " ("
                << (100.0 * eventsFullySparse / eventsConsidered) << "%)\n";
    }

    std::cout << "\n----- Fraction-of-Cells-Needing-Transfer Distribution (per "
                 "event) -----\n";
    for (std::size_t i = 0; i < fractionBuckets.size(); ++i) {
      if (fractionBuckets[i] == 0)
        continue;
      const double pct = eventsConsidered == 0
                             ? 0.0
                             : 100.0 * fractionBuckets[i] / eventsConsidered;
      const std::size_t barLen = static_cast<std::size_t>(pct / 2.0);
      const std::string label = (i == 10)
                                    ? "100%"
                                    : (std::to_string(i * 10) + "-" +
                                       std::to_string((i + 1) * 10) + "%");
      std::cout << std::setw(9) << label << ": " << std::setw(10)
                << fractionBuckets[i] << " (" << std::fixed
                << std::setprecision(2) << std::setw(6) << pct << "%) "
                << std::string(barLen, '#') << "\n";
    }
    std::cout << "=========================================\n";
  }

  void showStats() const {
    std::uint64_t totalTransfers = flags.size();
    std::uint64_t totalBitsSet = 0;
    std::size_t zeroFlagTransfers = 0;

    std::size_t minBitsSet = 64;
    std::size_t maxBitsSet = 0;
    std::uint64_t sumBitsSet = 0;

    // One bucket per possible popcount value (0..64).
    std::array<std::size_t, 65> bucketCounts{};

    for (const std::uint64_t value : flags) {
      const std::size_t bitsSet =
          static_cast<std::size_t>(__builtin_popcountll(value));
      totalBitsSet += bitsSet;
      sumBitsSet += bitsSet;
      minBitsSet = std::min(minBitsSet, bitsSet);
      maxBitsSet = std::max(maxBitsSet, bitsSet);

      if (value == 0)
        ++zeroFlagTransfers;

      ++bucketCounts[bitsSet];
    }

    const double avgBitsSet =
        totalTransfers == 0 ? 0.0
                            : static_cast<double>(sumBitsSet) / totalTransfers;

    std::cout << "===== Flag Stats =====\n";
    std::cout << "Total transfers (edges):        " << totalTransfers << "\n";
    std::cout << "Total bits set to true:         " << totalBitsSet << " / "
              << totalTransfers * 64 << "\n";
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
    std::cout << "\n";

    std::cout << "\n----- Bits-Set Distribution (per-bit buckets) -----\n";
    for (std::size_t bits = 0; bits <= 64; ++bits) {
      if (bucketCounts[bits] == 0)
        continue; // skip empty buckets to keep output compact

      const double pct = totalTransfers == 0
                             ? 0.0
                             : 100.0 * bucketCounts[bits] / totalTransfers;
      const std::size_t barLen =
          static_cast<std::size_t>(pct / 2.0); // 50 chars ≈ 100%

      std::cout << std::setw(4) << bits << " bits: " << std::setw(10)
                << bucketCounts[bits] << " (" << std::fixed
                << std::setprecision(2) << std::setw(6) << pct << "%) "
                << std::string(barLen, '#') << "\n";
    }
    std::cout << "=======================\n";
  }

  void showPerEventStats() const {
    const std::size_t numEvents = data.numberOfStopEvents();

    std::size_t minOutDegree = std::numeric_limits<std::size_t>::max();
    std::size_t maxOutDegree = 0;
    std::uint64_t totalOutDegree = 0;

    std::size_t minNonZero = std::numeric_limits<std::size_t>::max();
    std::size_t maxNonZero = 0;
    std::uint64_t totalNonZero = 0;

    std::size_t eventsWithNoOutgoing = 0;
    std::size_t eventsFullyPruned = 0; // nonZero == 0, outDegree > 0
    std::size_t eventsFullyKept = 0;   // nonZero == outDegree, outDegree > 0

    std::array<std::size_t, 33> outDegreeBuckets{}; // last slot = "32+"
    std::array<std::size_t, 33> nonZeroBuckets{};   // last slot = "32+"
    std::array<std::size_t, 11> ratioBuckets{};     // 10%-wide buckets, 0..100

    for (StopEventId e(0); e < numEvents; ++e) {
      std::size_t outDegree = 0;
      std::size_t nonZero = 0;

      for (const Edge edge : data.stopEventGraph.edgesFrom(Vertex(e))) {
        ++outDegree;
        if (flags[edge] != 0)
          ++nonZero;
      }

      if (outDegree == 0) {
        ++eventsWithNoOutgoing;
        continue;
      }

      minOutDegree = std::min(minOutDegree, outDegree);
      maxOutDegree = std::max(maxOutDegree, outDegree);
      totalOutDegree += outDegree;

      minNonZero = std::min(minNonZero, nonZero);
      maxNonZero = std::max(maxNonZero, nonZero);
      totalNonZero += nonZero;

      if (nonZero == 0)
        ++eventsFullyPruned;
      if (nonZero == outDegree)
        ++eventsFullyKept;

      ++outDegreeBuckets[std::min(outDegree, outDegreeBuckets.size() - 1)];
      ++nonZeroBuckets[std::min(nonZero, nonZeroBuckets.size() - 1)];

      const double ratio = 100.0 * nonZero / outDegree;
      std::size_t ratioBucket = static_cast<std::size_t>(ratio / 10.0);
      if (ratioBucket > 10)
        ratioBucket = 10;
      ++ratioBuckets[ratioBucket];
    }

    const std::size_t numActiveEvents = numEvents - eventsWithNoOutgoing;

    std::cout << "===== Per-Event Outgoing-Transfer Stats =====\n";
    std::cout << "Total stop events:                 " << numEvents << "\n";
    std::cout << "Events with no outgoing transfers: " << eventsWithNoOutgoing
              << "\n";
    std::cout << "Events considered:                 " << numActiveEvents
              << "\n";

    if (numActiveEvents > 0) {
      std::cout << "\nOut-degree per event:               min " << minOutDegree
                << ", max " << maxOutDegree << ", avg " << std::fixed
                << std::setprecision(3)
                << (static_cast<double>(totalOutDegree) / numActiveEvents)
                << "\n";

      std::cout << "Non-zero-flag transfers per event:  min " << minNonZero
                << ", max " << maxNonZero << ", avg " << std::fixed
                << std::setprecision(3)
                << (static_cast<double>(totalNonZero) / numActiveEvents)
                << "\n";

      std::cout
          << "\nEvents where ALL outgoing transfers are flagged (0% pruned): "
          << eventsFullyKept << " / " << numActiveEvents << " (" << std::fixed
          << std::setprecision(2) << (100.0 * eventsFullyKept / numActiveEvents)
          << "%)\n";

      std::cout
          << "Events where NO outgoing transfers are flagged (100% pruned): "
          << eventsFullyPruned << " / " << numActiveEvents << " (" << std::fixed
          << std::setprecision(2)
          << (100.0 * eventsFullyPruned / numActiveEvents) << "%)\n";
    }

    auto printBucketArray = [&](const char *title,
                                const std::array<std::size_t, 33> &buckets) {
      std::cout << "\n----- " << title << " -----\n";
      for (std::size_t i = 0; i < buckets.size(); ++i) {
        if (buckets[i] == 0)
          continue;
        const double pct =
            numActiveEvents == 0 ? 0.0 : 100.0 * buckets[i] / numActiveEvents;
        const std::size_t barLen = static_cast<std::size_t>(pct / 2.0);
        const std::string label = (i == buckets.size() - 1)
                                      ? (std::to_string(i) + "+")
                                      : std::to_string(i);
        std::cout << std::setw(4) << label << ": " << std::setw(10)
                  << buckets[i] << " (" << std::fixed << std::setprecision(2)
                  << std::setw(6) << pct << "%) " << std::string(barLen, '#')
                  << "\n";
      }
    };

    printBucketArray("Out-Degree Distribution", outDegreeBuckets);
    printBucketArray("Non-Zero-Flag Transfers Distribution", nonZeroBuckets);

    std::cout
        << "\n----- Fraction of Outgoing Transfers Kept (per event) -----\n";
    for (std::size_t i = 0; i < ratioBuckets.size(); ++i) {
      if (ratioBuckets[i] == 0)
        continue;
      const double pct = numActiveEvents == 0
                             ? 0.0
                             : 100.0 * ratioBuckets[i] / numActiveEvents;
      const std::size_t barLen = static_cast<std::size_t>(pct / 2.0);
      const std::string label = (i == 10)
                                    ? "100%"
                                    : (std::to_string(i * 10) + "-" +
                                       std::to_string((i + 1) * 10) + "%");
      std::cout << std::setw(9) << label << ": " << std::setw(10)
                << ratioBuckets[i] << " (" << std::fixed << std::setprecision(2)
                << std::setw(6) << pct << "%) " << std::string(barLen, '#')
                << "\n";
    }

    std::cout << "=======================\n";
  }

  void run() {
    const unsigned numLevels = static_cast<unsigned>(data.getNumberOfLevels());

    precomputeStopEventCellIds();

    std::vector<IBEInfo> IBEs{};
    std::size_t nrOfBorderStops = collectAllIBEs(IBEs);

    setOwnGroupFlags();

    // IBEs.clear();
    // IBEs.push_back(IBEInfo{StopId(3301), TripId(64765), StopIndex(25 - 1),
    // 4});

    if (IBEs.empty()) {
      std::cout << "No border stops, skipping.\n";
      return;
    }

    std::cout << "Running Backward Forward Search, collected " << IBEs.size()
              << " many IBEs\n";
    Progress progress(nrOfBorderStops);

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

    assert(false);
  }

  static void saveFlags(const std::string &fileName,
                        const std::vector<std::uint64_t> &flags) {
    IO::serialize(fileName, flags);
  }

  static std::vector<std::uint64_t> loadFlags(const std::string &fileName) {
    std::vector<std::uint64_t> flags;
    IO::deserialize(fileName, flags);

    return flags;
  }

  void ingestFlags(const std::string &fileName) {
    flags.clear();
    IO::deserialize(fileName, flags);
  }
};

} // namespace TripBased
