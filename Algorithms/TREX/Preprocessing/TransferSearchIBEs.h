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

#include "../../../DataStructures/Container/Set.h"
/* #include "../../../DataStructures/Container/IndexedSetBranchless.h" */
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../../DataStructures/TREX/TREXData.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../TripBased/Query/Profiler.h"
#include "../../TripBased/Query/TimestampedReachedIndex.h"
#include "Types.h"

// NOTE: die Länge der extracted paths stimmt nicht, weil ich beim entpacken
// aufhöre sobald ich etwas sehe was ich vorher bereits schon entpackt habe
namespace TripBased {

template <typename PROFILER = NoProfiler> class TransferSearch {
public:
  using Profiler = PROFILER;
  using Type = TransferSearch<Profiler>;

private:
  struct TripLabel {
    TripLabel(const StopEventId begin = noStopEvent,
              const StopEventId end = noStopEvent, const u_int32_t parent = -1,
              const Edge parentTransfer = noEdge)
        : begin(begin), end(end), parent(parent),
          parentTransfer(parentTransfer) {}
    StopEventId begin;
    StopEventId end;
    u_int32_t parent;
    Edge parentTransfer;
  };

  struct EdgeRange {
    EdgeRange() : begin(noEdge), end(noEdge) {}

    Edge begin;
    Edge end;
  };

  // Stores the shortcut information, which we insert into the
  // augmentedStopEventGraph we keep track of the number of transfer we hid
  // inside
  struct ShortCutToInsert {
    StopEventId fromStopEventId;
    StopEventId toStopEventId;
    uint8_t hopCounter;

    ShortCutToInsert(StopEventId fromStopEventId, StopEventId toStopEventId,
                     uint8_t hopCounter)
        : fromStopEventId(fromStopEventId), toStopEventId(toStopEventId),
          hopCounter(hopCounter) {}

    bool operator<(const ShortCutToInsert &other) {
      return std::tie(fromStopEventId, toStopEventId, hopCounter) <
             std::tie(other.fromStopEventId, other.toStopEventId,
                      other.hopCounter);
    }
  };

public:
  TransferSearch(TREXData &data,
                 const std::vector<StopEventId> &fromStopEventId,
                 const std::vector<EdgeLabel> &edgeLabels,
                 const std::vector<RouteLabel> &routeLabels)
      : data(data), fromStopEventId(fromStopEventId), edgeLabels(edgeLabels),
        routeLabels(routeLabels),
        /* edgesToInsert(), */
        queue(data.numberOfStopEvents()), edgeRanges(data.numberOfStopEvents()),
        queueSize(0), reachedIndex(data),
        toBeUnpacked(data.numberOfStopEvents()),
        lastExtractedRun(data.stopEventGraph.numEdges(), 0), currentRun(0)
  /* , extractedPaths(0) */
  /* , totalLengthPfExtractedPaths(0) */
  /* , numAddedShortcuts(0) */
  {
    profiler.registerPhases({PHASE_SCAN_TRIPS});
    profiler.registerMetrics({METRIC_ROUNDS, METRIC_SCANNED_TRIPS,
                              METRIC_SCANNED_STOPS, METRIC_RELAXED_TRANSFERS,
                              METRIC_ENQUEUES});
  }

  inline void run(const TripId trip, const StopIndex stopIndex,
                  const uint8_t newLevel) noexcept {
    AssertMsg(data.isTrip(trip), "Trip is not valid!");
    AssertMsg(stopIndex < data.numberOfStopsInTrip(trip),
              "StopIndex " << (int)stopIndex << " is not valid!");
    AssertMsg((stopIndex + 1) < data.numberOfStopsInTrip(trip),
              "StopIndex " << (int)(stopIndex + 1) << " is not valid!");

    profiler.start();
    clear();

    minLevel = newLevel;
    currentCellId =
        data.getCellIdOfStop(data.getStop(trip, StopIndex(stopIndex + 1)));

    enqueue(trip, stopIndex);
    scanTrips(16);
    unpack();
    profiler.done();
  }

  inline Profiler &getProfiler() noexcept { return profiler; }

private:
  inline void clear() noexcept {
    queueSize = 0;
    reachedIndex.clear();
    toBeUnpacked.clear();

    if (currentRun == 0) {
      lastExtractedRun.assign(data.stopEventGraph.numEdges(), 0);
    }
    ++currentRun;
  }

  inline void scanTrips(const uint8_t MAX_ROUNDS = 16) noexcept {
    profiler.startPhase();
    u_int8_t currentRoundNumber = 0;
    size_t roundBegin = 0;
    size_t roundEnd = queueSize;
    while (roundBegin < roundEnd && currentRoundNumber < MAX_ROUNDS) {
      ++currentRoundNumber;
      profiler.countMetric(METRIC_ROUNDS);

      for (size_t i = roundBegin; i < roundEnd; i++) {
#ifdef ENABLE_PREFETCH
        if (i + 4 < roundEnd) {
          __builtin_prefetch(&data.arrivalEvents[queue[i + 4].begin]);
        }
#endif
        const TripLabel &label = queue[i];
        profiler.countMetric(METRIC_SCANNED_TRIPS);
        for (StopEventId j = label.begin; j < label.end; j++) {
          profiler.countMetric(METRIC_SCANNED_STOPS);
          // check if stop of j is outside this cell => add to 'toBeUnpacked'
          StopId currentStop = data.getStopOfStopEvent(j);
          /* toBeUnpacked.insert(i, !isStopInCell(currentStop)); */
          if (!isStopInCell(currentStop)) {
            toBeUnpacked.insert(i);
          }
        }
      }

      for (size_t i = roundBegin; i < roundEnd; i++) {
#ifdef ENABLE_PREFETCH
        if (i + 4 < roundEnd) {
          __builtin_prefetch(&data.arrivalEvents[queue[i + 4].begin]);
          __builtin_prefetch(&edgeRanges[i + 4]);
        }
#endif
        TripLabel &label = queue[i];
        edgeRanges[i].begin =
            data.stopEventGraph.beginEdgeFrom(Vertex(label.begin));
        edgeRanges[i].end =
            data.stopEventGraph.beginEdgeFrom(Vertex(label.end));
      }

      for (size_t i = roundBegin; i < roundEnd; i++) {
        const EdgeRange &label = edgeRanges[i];
        for (Edge edge = label.begin; edge < label.end; edge++) {
          profiler.countMetric(METRIC_RELAXED_TRANSFERS);
          enqueue(edge, i);
        }
      }
      roundBegin = roundEnd;
      roundEnd = queueSize;
    }
    profiler.donePhase(PHASE_SCAN_TRIPS);
  }

  inline bool isStopInCell(StopId stop) const {
    AssertMsg(data.isStop(stop), "Stop is not a valid stop!");
    return !((data.getCellIdOfStop(stop) ^ currentCellId) >> minLevel);
  }

  inline void enqueue(const TripId trip, const StopIndex index) noexcept {
    profiler.countMetric(METRIC_ENQUEUES);
    if (reachedIndex.alreadyReached(trip, index))
      return;
    const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
    queue[queueSize] = TripLabel(StopEventId(firstEvent + index),
                                 StopEventId(firstEvent + reachedIndex(trip)));
    queueSize++;
    AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
    reachedIndex.update(trip, index);
  }

  inline void enqueue(const Edge edge, const size_t parent) noexcept {
    profiler.countMetric(METRIC_ENQUEUES);
    const EdgeLabel &label = edgeLabels[edge];

    // break if a) already reached OR b) the stop if this transfer is not in the
    // same cell
    if (reachedIndex.alreadyReached(label.trip,
                                    label.stopEvent - label.firstEvent) ||
        !isStopInCell(data.getStop(
            label.trip, StopIndex(label.stopEvent - label.firstEvent - 1))))
        [[likely]]
      return;

    if (minLevel > data.stopEventGraph.get(LocalLevel, edge)) [[likely]]
      return;

    queue[queueSize] = TripLabel(
        label.stopEvent,
        StopEventId(label.firstEvent + reachedIndex(label.trip)), parent, edge);

    queueSize++;
    AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
    reachedIndex.update(label.trip,
                        StopIndex(label.stopEvent - label.firstEvent));
  }

  // all marked events which we want to marks as local for the next level
  inline void unpack() {
    /* for (auto start = toBeUnpacked.begin(), end = toBeUnpacked.end(); */
    /*      start < end; ++start) { */
    /* #ifdef ENABLE_PREFETCH */
    /*   if (start + 4 < end) { */
    /*     __builtin_prefetch(&queue[*(start + 4)]); */
    /*   } */
    /* #endif */
    /*   unpackStopEvent(*start); */
    /* } */
    // we need to loop over the collected queue elements,
    // and for each element, we stored the latest ('das hinterste') event which
    // we want to mark as local
    auto &indexToLoopOver = toBeUnpacked.getValues();

    for (size_t i(0); i < indexToLoopOver.size(); ++i) {
#ifdef ENABLE_PREFETCH
      if (i + 4 < indexToLoopOver.size()) {
        __builtin_prefetch(&queue[indexToLoopOver[i + 4]]);
      }
#endif
      unpackStopEvent(indexToLoopOver[i]);

      // STATS
      // ++extractedPaths;
    }
  }

  // unpacks a reached stop event
  inline void unpackStopEvent(size_t index) {
    AssertMsg(index < queueSize, "Index is out of bounds!");
    TripLabel label = queue[index];
    Edge currentEdge = label.parentTransfer;

    // new vertices for the shortcut
    /* StopEventId toVertex = label.begin; */
    /* StopEventId fromVertex = noStopEvent; // will be assigned */

    /* uint8_t currentHopCounter(0); */

    while (currentEdge != noEdge) {
      // commented this out since I want to create shortcuts, hence i need to
      // rewind all transfers, even if i have already seen it.
      if (lastExtractedRun[currentEdge] == currentRun)
        return;
      lastExtractedRun[currentEdge] = currentRun;

      /* // set the locallevel of the events */
      /* fromVertex = fromStopEventId[currentEdge]; */
      /* currentHopCounter += data.stopEventGraph.get(Hop, currentEdge); */

      data.stopEventGraph.set(LocalLevel, currentEdge, minLevel + 1);

      // set the locallevel of the events
      StopEventId e = fromStopEventId[currentEdge];
      data.getLocalLevelOfEvent(e) = minLevel + 1;

      index = label.parent;
      label = queue[index];
      currentEdge = label.parentTransfer;

      // STATS
      /* ++totalLengthPfExtractedPaths; */
    }

    AssertMsg(
        index == 0,
        "The origin of the journey does not start with the incomming event!");

    /* // only add a shortcut if we can skip at least 2 transfers */
    /* if ((currentHopCounter / (minLevel+1)) >= 2) { */
    /*     /1* AssertMsg(fromVertex != noStopEvent, "From StopEvent has not been
     * assigned properly"); *1/ */
    /*     /1* AssertMsg(fromVertex != toVertex, "From- and To StopEvent should
     * not be the same"); *1/ */
    /*     /1* edgesToInsert.emplace_back(fromVertex, toVertex,
     * currentHopCounter); *1/ */

    /*     // STATS */
    /*     ++numAddedShortcuts; */
    /* } */
  }

public:
  /* inline double getAvgPathLengthPerLevel() noexcept */
  /* { */
  /*     return (double)totalLengthPfExtractedPaths / (double)extractedPaths; */
  /* } */

  /* inline uint64_t getNumberOfAddedShortcuts() noexcept */
  /* { */
  /*     return numAddedShortcuts; */
  /* } */

  /* inline void resetStats() noexcept */
  /* { */
  /*     // STATS */
  /*     totalLengthPfExtractedPaths = 0; */
  /*     extractedPaths = 0; */
  /*     numAddedShortcuts = 0; */
  /* } */

private:
  TREXData &data;
  const std::vector<StopEventId> &fromStopEventId;

  const std::vector<EdgeLabel> &edgeLabels;
  const std::vector<RouteLabel> &routeLabels;

  /* std::vector<ShortCutToInsert> edgesToInsert; */

  std::vector<TripLabel> queue;
  std::vector<EdgeRange> edgeRanges;

  size_t queueSize;
  TimestampedReachedIndex reachedIndex;

  uint8_t minLevel;
  uint16_t currentCellId;

  Profiler profiler;

  IndexedSet<false, size_t> toBeUnpacked;
  /* IndexedSetBranchless<size_t> toBeUnpacked; */

  // like a timestamp, used to check in which run the stop event has already
  // been extracted
  std::vector<uint32_t> lastExtractedRun;
  uint32_t currentRun;

  // stats
  /* uint64_t extractedPaths; */
  /* uint64_t totalLengthPfExtractedPaths; */
  /* uint64_t numAddedShortcuts; */
};

} // namespace TripBased
