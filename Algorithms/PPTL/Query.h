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

#include <unordered_map>
#include <vector>

#include "../../DataStructures/PPTL/Data.h"
#include "../PTL/Profiler.h"

namespace PPTL {

template <typename PROFILER = PTL::NoProfiler> class Query {
public:
  using Profiler = PROFILER;

  Query(Data &data) : data(data) {
    profiler.registerPhases(
        {PTL::PHASE_FIND_FIRST_VERTEX, PTL::PHASE_INSERT_HASH, PTL::PHASE_RUN});
    profiler.registerMetrics({PTL::METRIC_INSERTED_HUBS, PTL::METRIC_CHECK_ARR_EVENTS,
                              PTL::METRIC_CHECK_HUBS, PTL::METRIC_FOUND_SOLUTIONS});
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
    profiler.donePhase(PTL::PHASE_FIND_FIRST_VERTEX);

    if (startingVertex == noVertex) {
      profiler.done();
      return -1;
    }

    profiler.startPhase();
    prepareSet();
    profiler.donePhase(PTL::PHASE_INSERT_HASH);

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

    profiler.donePhase(PTL::PHASE_RUN);
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

    hash.clear();

    for (auto &fwdHub : data.getFwdHubs(startingVertex)) {
      const std::uint16_t pathId = extractPathId(fwdHub);
      const std::uint16_t pathPos = extractPathPos(fwdHub);

      auto it = hash.find(pathId);
      if (it == hash.end()) {
        hash.emplace(pathId, pathPos);
      } else {
        // Keep the smallest path position (earliest in path)
        it->second = std::min(it->second, pathPos);
      }

      profiler.countMetric(PTL::METRIC_INSERTED_HUBS);
    }
  }

  inline size_t getIndexOfFirstEventAfterTime(const auto &arrEvents,
                                              const int time) noexcept {
    auto it = std::lower_bound(arrEvents.begin(), arrEvents.end(), time,
                               [&](const size_t event, const int time) {
                                 return data.teData.getTimeOfVertex(
                                            Vertex(event)) < time;
                               });

    return std::distance(arrEvents.begin(), it);
  }

  inline int scanHubs(const auto &arrEvents, const size_t left = 0) noexcept {
    for (size_t i = left; i < arrEvents.size(); ++i) {
      const auto &arrEventAtTarget = arrEvents[i];

      int arrTime = data.teData.getTimeOfVertex(Vertex(arrEventAtTarget));

      profiler.countMetric(PTL::METRIC_CHECK_ARR_EVENTS);

      const auto &bwdLabels = data.getBwdHubs(Vertex(arrEventAtTarget));

      for (const auto &hub : bwdLabels) {
        profiler.countMetric(PTL::METRIC_CHECK_HUBS);
        const std::uint16_t pId = extractPathId(hub);
        const std::uint16_t pPos = extractPathPos(hub);

        auto it = hash.find(pId);
        if (it != hash.end() && it->second <= pPos) {
          profiler.countMetric(PTL::METRIC_FOUND_SOLUTIONS);
          return arrTime;
        }
      }
    }
    return -1;
  }

  inline int scanHubsBinary(const auto &arrEvents,
                            const size_t left = 0) noexcept {
    if (arrEvents.empty())
      return -1;

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

      profiler.countMetric(PTL::METRIC_CHECK_ARR_EVENTS);

      const auto &bwdLabels = data.getBwdHubs(Vertex(arrEventAtTarget));

      for (const auto &hub : bwdLabels) {
        profiler.countMetric(PTL::METRIC_CHECK_HUBS);
        std::uint16_t pId = extractPathId(hub);
        std::uint16_t pPos = extractPathPos(hub);

        auto it = hash.find(pId);
        found = (it != hash.end() && it->second <= pPos);

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
    profiler.countMetric(PTL::METRIC_FOUND_SOLUTIONS);
    return data.teData.getTimeOfVertex(Vertex(arrEvents[i]));
  }

  inline const Profiler &getProfiler() const noexcept { return profiler; }

  Data &data;
  Vertex startingVertex;
  std::unordered_map<std::uint16_t, std::uint16_t> hash;
  Profiler profiler;
};
} // namespace PPTL
