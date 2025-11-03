/**********************************************************************************

 Copyright (c) 2025 Patrick Steil

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

#include <vector>

#include "../../DataStructures/PPTL/Data.h"
#include "Profiler.h"

namespace PPTL {

constexpr std::uint16_t INVALID_POS = static_cast<std::uint16_t>(-1);

template <typename PROFILER = NoProfiler>
class Query {
 public:
  using Profiler = PROFILER;

  Query(Data &data) : data(data) {
    std::fill(hashPos.begin(), hashPos.end(), INVALID_POS);
    profiler.registerPhases(
        {PHASE_FIND_FIRST_VERTEX, PHASE_INSERT_HASH, PHASE_RUN});
    profiler.registerMetrics({METRIC_INSERTED_HUBS, METRIC_CHECK_ARR_EVENTS,
                              METRIC_CHECK_HUBS, METRIC_FOUND_SOLUTIONS});
  };

  template <bool BINARY = true>
  int run(const StopId source, const int departureTime,
          const StopId target) noexcept {
    AssertMsg(data.teData.isStop(source), "Source is not valid!");
    AssertMsg(data.teData.isStop(target), "Target is not valid!");
    AssertMsg(0 <= departureTime, "Time is negative!");

    profiler.start();

    profiler.startPhase();
    prepareStartingVertex(source, departureTime);
    profiler.donePhase(PHASE_FIND_FIRST_VERTEX);

    if (startingVertex == noVertex) {
      profiler.done();
      return -1;
    }

    profiler.startPhase();
    prepareSet();
    profiler.donePhase(PHASE_INSERT_HASH);

    profiler.startPhase();

    const auto &arrEvents = data.teData.getArrivalsOfStop(target);

    size_t left = getIndexOfFirstEventAfterTime(arrEvents, departureTime);

    int finalTime = -1;

    if constexpr (BINARY) {
      finalTime = (arrEvents.size() < 16 ? scanHubs(arrEvents, left)
                                         : scanHubsBinary(arrEvents, left));
    } else {
      finalTime = scanHubs(arrEvents, left);
    }

    profiler.donePhase(PHASE_RUN);

    undoSet();
    profiler.done();

    return finalTime;
  }

  inline bool prepareStartingVertex(const StopId stop,
                                    const int time) noexcept {
    Vertex firstReachableNode =
        data.teData.getFirstReachableDepartureVertexAtStop(stop, time);

    startingVertex = noVertex;

    // Did we reach any transfer node?
    if (!data.teData.isEvent(firstReachableNode)) {
      return false;
    }

    AssertMsg(data.teData.isEvent(firstReachableNode),
              "First reachable node is not valid!");
    startingVertex = firstReachableNode;
    return true;
  }

  inline void prepareSet() noexcept {
    AssertMsg(data.teData.isEvent(startingVertex),
              "First reachable node is not valid!");

    for (auto &fwdHub : data.getFwdHubs(startingVertex)) {
      const std::uint16_t pathId = extractPathId(fwdHub);
      const std::uint16_t pathPos = extractPathPos(fwdHub);
      hashPos[pathId] = pathPos;

      profiler.countMetric(METRIC_INSERTED_HUBS);
    }
  }

  inline void undoSet() noexcept {
    AssertMsg(data.teData.isEvent(startingVertex),
              "First reachable node is not valid!");

    for (auto &fwdHub : data.getFwdHubs(startingVertex)) {
      const std::uint16_t pathId = extractPathId(fwdHub);
      hashPos[pathId] = INVALID_POS;

      profiler.countMetric(METRIC_INSERTED_HUBS);
    }
  }
  inline size_t getIndexOfFirstEventAfterTime(const auto &arrEvents,
                                              const int time) noexcept {
    auto it = std::lower_bound(
        arrEvents.begin(), arrEvents.end(), time,
        [&](const size_t event, const int time) {
          return data.teData.getTimeOfVertex(Vertex(event)) < time;
        });

    return std::distance(arrEvents.begin(), it);
  }

  inline int scanHubs(const auto &arrEvents, const size_t left = 0) noexcept {
    for (size_t i = left; i < arrEvents.size(); ++i) {
      const auto &arrEventAtTarget = arrEvents[i];

      int arrTime = data.teData.getTimeOfVertex(Vertex(arrEventAtTarget));

      profiler.countMetric(METRIC_CHECK_ARR_EVENTS);

      const auto &bwdLabels = data.getBwdHubs(Vertex(arrEventAtTarget));
      for (std::size_t index = 0; index < bwdLabels.size(); ++index) {
        profiler.countMetric(METRIC_CHECK_HUBS);

#ifdef ENABLE_PREFETCH
        if (index + 4 < bwdLabels.size()) {
          __builtin_prefetch(&hashPos[extractPathPos(bwdLabels[index + 4])]);
        }
#endif

        const auto &hub = bwdLabels[index];
        std::uint16_t pId = extractPathId(hub);
        std::uint16_t pPos = extractPathPos(hub);

        if (hashPos[pId] != INVALID_POS && hashPos[pId] <= pPos) {
          profiler.countMetric(METRIC_FOUND_SOLUTIONS);
          return arrTime;
        }
      }
    }
    return -1;
  }

  inline int scanHubsBinary(const auto &arrEvents,
                            const size_t left = 0) noexcept {
    if (arrEvents.empty()) return -1;

    // Use signed type to handle underflows correctly
    int i = static_cast<int>(left);
    int j = static_cast<int>(arrEvents.size()) - 1;

    AssertMsg(i <= j, "Left and Right are not valid!");

    bool found = false;
    while (i <= j) {
      size_t mid = i + (j - i) / 2;
      AssertMsg(mid < arrEvents.size(),
                "Mid ( " << mid << " ) is out of bounds (" << arrEvents.size()
                         << " )! i: " << i << ", j: " << j);

      const auto &arrEventAtTarget = arrEvents[mid];

      profiler.countMetric(METRIC_CHECK_ARR_EVENTS);

      const auto &bwdLabels = data.getBwdHubs(Vertex(arrEventAtTarget));
      for (std::size_t index = 0; index < bwdLabels.size(); ++index) {
        profiler.countMetric(METRIC_CHECK_HUBS);

#ifdef ENABLE_PREFETCH
        if (index + 4 < bwdLabels.size()) {
          __builtin_prefetch(&hashPos[extractPathPos(bwdLabels[index + 4])]);
        }
#endif

        const auto &hub = bwdLabels[index];
        std::uint16_t pId = extractPathId(hub);
        std::uint16_t pPos = extractPathPos(hub);

        found = (hashPos[pId] != INVALID_POS && hashPos[pId] <= pPos);

        if (found) {
          break;
        }
      }
      if (found) {
        j = mid - 1;
      } else {
        i = mid + 1;
      }
    }

    // Properly handle the case when no valid index is found
    if ((i == static_cast<int>(arrEvents.size()) - 1 && !found) ||
        i >= static_cast<int>(arrEvents.size())) {
      return -1;
    }
    profiler.countMetric(METRIC_FOUND_SOLUTIONS);
    return data.teData.getTimeOfVertex(Vertex(arrEvents[i]));
  }

  inline const Profiler &getProfiler() const noexcept { return profiler; }

  Data &data;
  Vertex startingVertex;
  Profiler profiler;
  std::array<std::uint16_t, 65536> hashPos;
};

}  // namespace PPTL
