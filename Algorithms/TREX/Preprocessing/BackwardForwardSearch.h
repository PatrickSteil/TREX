#pragma once

#include <cassert>
#include <iostream>
#include <vector>

#include "../../../DataStructures/Container/Queue.h"
#include "../../../DataStructures/TREX/TREXData.h"
#include "../../../Helpers/cell_bits.h"
#include "../../../Helpers/iota_ranger.h"

#include "BuilderIBEs.h"

// TODO
// - add cell information
// - handle "in between trips" (prop not)
// - process all border stops -> done via BackwardForwardSweeper
//
// infos:
// - man muss sich nicht die "inbetween" trip segmente anschauen, weil wenn die
// einen transfer zum aktuellen trip segment haben, dann wurden sie erreicht

namespace TripBased {

constexpr bool DEBUG = true;

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

class BackwardForwardSearch {
private:
  const TREXData &data;
  const TransferGraph &revTransferGraph;
  std::vector<std::uint32_t> &flags;

  PreallocatedQueue<QueueElement> queue;

  std::uint32_t timeStamp;
  std::vector<std::uint32_t> reachedTimeStamp;

private:
  void setFlag(const Edge edge, const std::uint16_t cell) {
    assert(edge < flags.size());
    flags[edge] |= expand(cell);
  }

  void clearNewRun() {
    timeStamp++;

    if (timeStamp == 0) {
      timeStamp = 1;
      std::fill(reachedTimeStamp.begin(), reachedTimeStamp.end(), 0);
    }

    queue.clear();
  }

  void startBackward(const IBEInfo &ibe) {
    const std::uint64_t cellToSet = data.getCellIdOfStop(ibe.stopId);

    clearNewRun();
    StopEventId startingEvent = data.getStopEventId(ibe.tripId, ibe.stopIndex);
    enqueue(startingEvent);

    std::size_t left = 0;
    std::size_t right = queue.size();

    while (left < right) {
      for (std::size_t i = left; i < right; ++i) {
        const auto &element = queue[i];
        //
        // if (DEBUG) {
        //   std::cout << "QueueElement [From " << (int)element.begin << " To "
        //             << (int)element.end << "]\n";
        // }

        const auto begin =
            revTransferGraph.beginEdgeFrom(Vertex(element.begin));
        const auto end = revTransferGraph.beginEdgeFrom(Vertex(element.end));

        for (auto edge : reverse_range((uint64_t)begin, (uint64_t)end)) {
          StopEventId toEvent{revTransferGraph.get(ToVertex, Edge(edge))};
          enqueue(toEvent);
        }
      }

      // now flag the transfers directly
      for (std::size_t i = left; i < right; ++i) {
        const auto &element = queue[i];

        // if (DEBUG) {
        //   const TripId trip = data.tripOfStopEvent[element.begin];
        //   assert(trip != noTripId);
        //   assert(trip == data.tripOfStopEvent[element.end - 1]);
        //
        //   std::cout << "Trip Segment [TripId " << (int)trip << ", Route "
        //             << (int)data.routeOfTrip[trip] << "]\n";
        // }

        flagTripSegment(element.begin, element.end, cellToSet);
        // TODO add check that end - 1 Event always must find parent, bcs we
        // added its trip Segment via it
      }

      left = right;
      right = queue.size();
    }
  }

  // [left, right)
  void flagTripSegment(const StopEventId left, const StopEventId right,
                       const std::uint16_t cellToSet) {
    for (StopEventId e = left; e < right; ++e) {
      // if (DEBUG) {
      //   std::cout << "\tScan from event " << (int)e << " with transfer range
      //   ["
      //             << (int)(left) << ", " << (int)(right) << "]\n";
      // }

      for (const Edge edge : data.stopEventGraph.edgesFrom(Vertex(e))) {
        StopEventId toEvent{data.stopEventGraph.get(ToVertex, edge)};
        assert(toEvent < data.numberOfStopEvents());

        if (reachedTimeStamp[toEvent] == timeStamp) {
          // if (DEBUG) {
          //   std::cout << "Found! from " << int(e) << " to " << int(toEvent)
          //             << std::endl;
          // }

          setFlag(edge, cellToSet);
          break;
        }
      }
    }
  }

  void enqueue(const StopEventId to) {
    const TripId trip = data.tripOfStopEvent[to];
    const StopEventId first = data.firstStopEventOfTrip[trip];

    StopEventId left = to;

    while (true) {
      if (reachedTimeStamp[left] == timeStamp) {
        ++left;
        break;
      }

      reachedTimeStamp[left] = timeStamp;

      if (left == first) {
        break;
      }

      --left;
    }

    if (left <= to) {
      queue.emplace(StopEventId(left), StopEventId(to + 1));
    }
  }

public:
  BackwardForwardSearch(const TREXData &data,
                        const TransferGraph &revTransferGraph,
                        std::vector<std::uint32_t> &flags)
      : data(data), revTransferGraph(revTransferGraph), flags(flags),
        timeStamp(1), reachedTimeStamp(data.numberOfStopEvents(), 0),
        queue(data.numberOfStopEvents()) {}

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

  std::vector<IBEInfo> collectAllIBEs() {
    auto inSameCell = [&](const StopId a, const StopId b) {
      return (data.getCellIdOfStop(a) == data.getCellIdOfStop(b));
    };

    std::vector<IBEInfo> IBEs;

    for (StopId stop(0); stop < data.numberOfStops(); ++stop) {
      for (const RAPTOR::RouteSegment &route :
           data.routesContainingStop(stop)) {
        if (route.stopIndex == 0)
          continue;

        RAPTOR::RouteSegment neighbourSeg(route.routeId,
                                          StopIndex(route.stopIndex - 1));

        if (!inSameCell(stop,
                        data.raptorData.stopOfRouteSegment(neighbourSeg))) {
          for (TripId trip : data.tripsOfRoute(route.routeId)) {
            IBEs.emplace_back(stop, trip, StopIndex(route.stopIndex - 1));
          }
        }
      }
    }

    return IBEs;
  }

  void processStop(const std::vector<IBEInfo> &IBEs, const std::size_t left,
                   const std::size_t right) {
    if (left >= right)
      return;

    if (DEBUG) {
      std::cout << "Process batch " << left << " to " << right << " [Stop "
                << (int)IBEs[left].stopId << "]" << std::endl;
    }

    search.run(IBEs, left, right);
  }

public:
  BackwardForwardSweeper(const TREXData &data)
      : data(data), revTransferGraph(),
        flags(data.stopEventGraph.numEdges(), 0),
        search(data, revTransferGraph, flags) {
    flipTransferGraph();
  }

  const std::vector<std::uint32_t> &getFlags() const { return flags; }

  void showStats() const {
    constexpr int NUM_LEVELS = 16;

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
    std::cout << "Total bits set to true:          " << totalBitsSet << " / "
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

  void run() {
    std::vector<IBEInfo> IBEs = collectAllIBEs();

    std::sort(IBEs.begin(), IBEs.end(),
              [&](const auto &left, const auto &right) {
                if (left.stopId != right.stopId)
                  return left.stopId < right.stopId;
                StopEventId leftEvent =
                    data.getStopEventId(left.tripId, left.stopIndex);
                StopEventId rightEvent =
                    data.getStopEventId(right.tripId, right.stopIndex);

                return std::tie(data.departureTime(leftEvent), leftEvent) <
                       std::tie(data.departureTime(rightEvent), rightEvent);
              });

    if (IBEs.empty()) {
      std::cout << "Warning: No IBEs found!\n";
      return;
    }

    std::size_t left = 0;
    for (std::size_t right = 0; right < IBEs.size(); ++right) {
      const bool isLast = (right + 1 == IBEs.size());
      const bool stopChanges =
          !isLast && IBEs[right + 1].stopId != IBEs[left].stopId;

      if (isLast || stopChanges) {
        processStop(IBEs, left, right + 1);
        left = right + 1;
      }
    }
  }
};

} // namespace TripBased
