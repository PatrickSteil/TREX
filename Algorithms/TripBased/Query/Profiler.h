/**********************************************************************************

 Copyright (c) 2023-2025 Patrick Steil
 Copyright (c) 2019-2022 KIT ITI Algorithmics Group

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

#include <iostream>

#include "../../../Helpers/String/String.h"
#include "../../../Helpers/Timer.h"

namespace TripBased {

typedef enum {
  PHASE_SCAN_INITIAL,
  PHASE_EVALUATE_INITIAL,
  PHASE_SCAN_TRIPS,
  PHASE_FORWARD,
  PHASE_BACKWARD,
  PHASE_MAIN,
  PHASE_COLLECT_DEPTIMES,
  PHASE_GET_JOURNEYS,
  PHASE_ENQUEUE_COLLECTED_DEP_TIMES,
  PHASE_TREX_COLLECT_IBES,
  PHASE_TREX_SORT_IBES,
  PHASE_TREX_FILTER_IBES,
  PHASE_TREX_UNPACK,
  NUM_PHASES,
} Phase;

constexpr const char *PhaseNames[] = {
    "Scan initial transfers",
    "Evaluate initial transfers",
    "Scan trips",
    "Forward pruning search",
    "Backward pruning search",
    "Main search",
    "Get Journeys",
    "Enqueue the collected departure times",
    "Collect departure times",
    "Collect IBEs",
    "Sort IBEs",
    "Filter IBEs",
    "Unpacking transfers",
};

typedef enum {
  METRIC_ROUNDS,
  METRIC_SCANNED_TRIPS,
  METRIC_SCANNED_STOPS,
  METRIC_SCANNED_LEVEL_ZERO_TRIPS,
  METRIC_SCANNED_LEVEL_ZERO_STOPS,
  METRIC_RELAXED_TRANSFERS,
  METRIC_ENQUEUES,
  METRIC_ADD_JOURNEYS,
  METRIC_COUNT_DISTANCE,
  NUMBER_OF_RUNS,
  DISCARDED_EDGE,
  METRIC_TREX_COLLECTED_IBES,
  METRIC_TREX_STOPEVENT_TO_UNPACK,
  METRIC_TREX_NUMBER_OF_RUNS,
  NUM_METRICS
} Metric;

constexpr const char *MetricNames[] = {"Rounds",
                                       "Scanned trips",
                                       "Scanned stops",
                                       "Scanned level zero trips",
                                       "Scanned level zero stops",
                                       "Relaxed transfers",
                                       "Enqueued trips",
                                       "Added journeys",
                                       "Distance / MaxSpeed",
                                       "Number of Runs",
                                       "Number of discarded edges",
                                       "Number of collected IBEs",
                                       "Number of stop events to unpack",
                                       "Number of runs"};

class NoProfiler {
public:
  inline void
  registerPhases(const std::initializer_list<Phase> &) const noexcept {}
  inline void
  registerMetrics(const std::initializer_list<Metric> &) const noexcept {}

  inline void start() const noexcept {}
  inline void done() const noexcept {}

  inline void startPhase() const noexcept {}
  inline void donePhase(const Phase) const noexcept {}

  inline void countMetric(const Metric) const noexcept {}

  inline void printStatistics() const noexcept {}

  inline void printStatisticsAsCSV() const noexcept {}

  inline void reset() const noexcept {}
};

class AggregateProfiler : public NoProfiler {
public:
  AggregateProfiler()
      : totalTime(0.0), phaseTime(NUM_PHASES, 0.0), metricValue(NUM_METRICS, 0),
        numQueries(0) {}

  inline void
  registerPhases(const std::initializer_list<Phase> &phaseList) noexcept {
    for (const Phase phase : phaseList) {
      phases.push_back(phase);
    }
  }

  inline void
  registerMetrics(const std::initializer_list<Metric> &metricList) noexcept {
    for (const Metric metric : metricList) {
      metrics.push_back(metric);
    }
  }

  inline void start() noexcept { totalTimer.restart(); }

  inline void done() noexcept {
    totalTime += totalTimer.elapsedMicroseconds();
    numQueries++;
  }

  inline void startPhase() noexcept { phaseTimer.restart(); }

  inline void donePhase(const Phase phase) noexcept {
    phaseTime[phase] += phaseTimer.elapsedMicroseconds();
  }

  inline void countMetric(const Metric metric) noexcept {
    metricValue[metric]++;
  }

  inline double getTotalTime() const noexcept { return totalTime / numQueries; }

  inline double getPhaseTime(const Phase phase) const noexcept {
    return phaseTime[phase] / numQueries;
  }

  inline double getMetric(const Metric metric) const noexcept {
    return metricValue[metric] / static_cast<double>(numQueries);
  }

  inline void printStatistics() const noexcept {
    for (const Metric metric : metrics) {
      std::cout << MetricNames[metric] << ": "
                << String::prettyDouble(
                       metricValue[metric] / static_cast<double>(numQueries), 2)
                << std::endl;
    }
    for (const Phase phase : phases) {
      std::cout << PhaseNames[phase] << ": "
                << String::musToString(phaseTime[phase] /
                                       static_cast<double>(numQueries))
                << std::endl;
    }
    std::cout << "Total time: " << String::musToString(totalTime / numQueries)
              << std::endl;
  }

  inline void printStatisticsAsCSV() const noexcept {
    for (const Metric metric : metrics) {
      std::cout << "\"" << MetricNames[metric] << "\","
                << (float)(metricValue[metric] / (double)(numQueries))
                << std::endl;
    }
    for (const Phase phase : phases) {
      std::cout << "\"" << PhaseNames[phase] << "\","
                << (uint64_t)(phaseTime[phase] / (double)(numQueries))
                << std::endl;
    }
    std::cout << "\"Total time\"," << (uint64_t)(totalTime / numQueries)
              << std::endl;
  }

  inline void reset() noexcept {
    totalTime = 0.0;
    phaseTime.assign(NUM_PHASES, 0.0);
    metricValue.assign(NUM_METRICS, 0);
    numQueries = 0;
  }

private:
  Timer totalTimer;
  double totalTime;
  std::vector<Phase> phases;
  std::vector<Metric> metrics;
  Timer phaseTimer;
  std::vector<double> phaseTime;
  std::vector<long long> metricValue;
  size_t numQueries;
};

} // namespace TripBased
