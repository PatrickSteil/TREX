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
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../../DataStructures/TREX/TREXData.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../TripBased/Query/Profiler.h"
#include "../../TripBased/Query/TimestampedReachedIndex.h"
#include "../../TripBased/Query/Types.h"

namespace TripBased {

template <typename PROFILER = NoProfiler> class TransferSearch {
public:
  using Profiler = PROFILER;
  using Type = TransferSearch<Profiler>;

private:
  struct TripLabel {
    TripLabel(const StopEventId begin = noStopEvent,
              const StopEventId end = noStopEvent,
              TripLabel *parentPtr = nullptr,
              const Edge parentTransfer = noEdge)
        : begin(begin), end(end), parentPtr(parentPtr),
          parentTransfer(parentTransfer) {}
    StopEventId begin;
    StopEventId end;
    TripLabel *parentPtr;
    Edge parentTransfer;
  };

  struct EdgeRange {
    EdgeRange() : begin(noEdge), end(noEdge) {}

    Edge begin;
    Edge end;
  };

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
  TransferSearch(TREXData &data, std::vector<EdgeLabel> &edgeLabels,
                 const std::vector<RouteLabel> &routeLabels,
                 const std::vector<uint16_t> &cellIdOfEvent)
      : data(data), edgeLabels(edgeLabels), routeLabels(routeLabels),
        cellIdOfEvent(cellIdOfEvent), queue(data.numberOfStopEvents()),
        edgeRanges(data.numberOfStopEvents()), queueSize(0), reachedIndex(data),
        toBeUnpacked(data.numberOfStopEvents()) {
    profiler.registerPhases({PHASE_SCAN_TRIPS, PHASE_TREX_UNPACK});
    profiler.registerMetrics({METRIC_ROUNDS, METRIC_SCANNED_TRIPS,
                              METRIC_SCANNED_STOPS, METRIC_RELAXED_TRANSFERS,
                              METRIC_ENQUEUES,
                              METRIC_TREX_STOPEVENT_TO_UNPACK});
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

    assert(newLevel < data.getNumberOfLevels());
    minLevel = newLevel;
    currentCellId =
        data.getCellIdOfStop(data.getStop(trip, StopIndex(stopIndex + 1)));
    AssertMsg(currentCellId !=
                  data.getCellIdOfStop(data.getStop(trip, stopIndex)),
              "CellId should be different!");

    enqueue(trip, StopIndex(stopIndex + 1));
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
          __builtin_prefetch(&cellIdOfEvent[queue[i + 4].begin]);
        }
#endif
        const TripLabel &label = queue[i];
        profiler.countMetric(METRIC_SCANNED_TRIPS);
        bool isInSameCell = true;
        for (StopEventId j = label.begin; j < label.end; j++) {
          profiler.countMetric(METRIC_SCANNED_STOPS);
          isInSameCell &= isEventInCell(cellIdOfEvent[j]);
        }
        if (!isInSameCell) {
          profiler.countMetric(METRIC_TREX_STOPEVENT_TO_UNPACK);
          toBeUnpacked.insert(i);
        }
      }

      for (size_t i = roundBegin; i < roundEnd; i++) {
#ifdef ENABLE_PREFETCH
        if (i + 4 < roundEnd) {
          __builtin_prefetch(&cellIdOfEvent[queue[i + 4].begin]);
          __builtin_prefetch(&edgeRanges[i + 4]);
        }
#endif
        TripLabel &label = queue[i];

        for (StopEventId j = label.begin; j < label.end; j++) {
          if (!isEventInCell(cellIdOfEvent[j])) [[unlikely]] {
            label.end = j;
          }
        }
        edgeRanges[i].begin =
            data.stopEventGraph.beginEdgeFrom(Vertex(label.begin));
        edgeRanges[i].end =
            data.stopEventGraph.beginEdgeFrom(Vertex(label.end));
      }

      for (size_t i = roundBegin; i < roundEnd; i++) {
#ifdef ENABLE_PREFETCH
        if (i + 4 < roundEnd) {
          __builtin_prefetch(&edgeLabels[edgeRanges[i + 4].begin]);
        }
#endif

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

  inline bool isEventInCell(const uint16_t cellId) const {
    return !((cellId ^ currentCellId) >> minLevel);
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
    if (minLevel > label.getRank()) [[likely]]
      return;

    const uint8_t reachedTrip = reachedIndex(label.getTrip());
    if (reachedTrip <= uint8_t(label.getStopIndex())) [[likely]]
      return;

    AssertMsg(parent < queueSize, "Parent cannot be outside the queue!");
    queue[queueSize] = TripLabel(
        label.getStopEvent(), StopEventId(label.getFirstEvent() + reachedTrip),
        &queue[parent], edge);

    queueSize++;
    AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
    reachedIndex.update(label.getTrip(), StopIndex(label.getStopIndex()));
  }

  inline void unpack() {
    profiler.startPhase();
    const auto &indexToLoopOver = toBeUnpacked.getValues();

    for (size_t i(0); i < indexToLoopOver.size(); ++i) {
#ifdef ENABLE_PREFETCH
      if (i + 4 < indexToLoopOver.size()) {
        __builtin_prefetch(&queue[indexToLoopOver[i + 4]]);
      }
#endif
      unpackStopEvent(indexToLoopOver[i]);
    }
    profiler.donePhase(PHASE_TREX_UNPACK);
  }

  inline void unpackStopEvent(size_t index) {
    AssertMsg(index < queueSize, "Index is out of bounds!");
    TripLabel label = queue[index];

    std::uint32_t toMark = 0;
    uint32_t lut[2] = {1u << (2 * minLevel), 1u << (2 * minLevel + 1)};

    const TripId trip = data.tripOfStopEvent[label.begin];
    const StopEventId endEvent = data.firstStopEventOfTrip[trip + 1];

    for (StopEventId e(label.end); e < endEvent; e++) {
      std::uint16_t toCellId = cellIdOfEvent[e];
      toMark |= lut[(toCellId >> minLevel) & 1];
    }

    uint8_t newRank = minLevel + 1;
    for (TripLabel *p = &queue[index]; p->parentTransfer != noEdge;
         p = p->parentPtr) {
      assert(p != nullptr);
      auto e = p->parentTransfer;
      edgeLabels[e].setRank(newRank);
      data.edgeFlags[e] |= toMark;
    }
  }

private:
  TREXData &data;

  std::vector<EdgeLabel> &edgeLabels;
  const std::vector<RouteLabel> &routeLabels;
  const std::vector<uint16_t> &cellIdOfEvent;

  std::vector<TripLabel> queue;
  std::vector<EdgeRange> edgeRanges;

  size_t queueSize;
  TimestampedReachedIndex reachedIndex;

  uint8_t minLevel;
  uint16_t currentCellId;

  Profiler profiler;

  IndexedSet<false, size_t> toBeUnpacked;
};

} // namespace TripBased
