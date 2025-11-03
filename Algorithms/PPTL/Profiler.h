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

#include "../../Helpers/String/String.h"
#include "../../Helpers/Timer.h"

namespace PPTL {

typedef enum {
  PHASE_FIND_FIRST_VERTEX,
  PHASE_INSERT_HASH,
  PHASE_RUN,
  NUM_PHASES
} Phase;

constexpr const char *PhaseNames[] = {"Find first reachable Vertex  ",
                                      "Building Hash-Hubs           ",
                                      "Run Query                    "};
typedef enum {
  METRIC_INSERTED_HUBS,
  METRIC_CHECK_ARR_EVENTS,
  METRIC_CHECK_HUBS,
  METRIC_FOUND_SOLUTIONS,
  NUM_METRICS
} Metric;

constexpr const char *MetricNames[] = {
    "# Inserted Hubs              ", "# Arrival Events             ",
    "# Check Hubs                 ", "# Solutions                  "};

class NoProfiler {
 public:
  inline void registerPhases(
      const std::initializer_list<Phase> &) const noexcept {}
  inline void registerMetrics(
      const std::initializer_list<Metric> &) const noexcept {}

  inline void start() const noexcept {}
  inline void done() const noexcept {}

  inline void startPhase() const noexcept {}
  inline void donePhase(const Phase) const noexcept {}

  inline void countMetric(const Metric) const noexcept {}

  inline void printStatistics() const noexcept {}
};

class AggregateProfiler : public NoProfiler {
 public:
  AggregateProfiler()
      : totalTime(0.0),
        phaseTime(NUM_PHASES, 0.0),
        metricValue(NUM_METRICS, 0),
        numQueries(0) {}

  inline void registerPhases(
      const std::initializer_list<Phase> &phaseList) noexcept {
    for (const Phase phase : phaseList) {
      phases.push_back(phase);
    }
  }

  inline void registerMetrics(
      const std::initializer_list<Metric> &metricList) noexcept {
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
    std::cout << "Total Time                   : "
              << String::musToString(totalTime / numQueries) << std::endl;
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

}  // namespace PPTL
