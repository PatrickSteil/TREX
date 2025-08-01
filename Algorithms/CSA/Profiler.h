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

#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "../../DataStructures/CSA/Data.h"
#include "../../Helpers/HighlightText.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/Types.h"

namespace CSA {

typedef enum {
  PHASE_CLEAR,
  PHASE_INITIALIZATION,
  PHASE_CONNECTION_SCAN,
  PHASE_FINAL_TRANSFERS,
  PHASE_UPWARD_SWEEP,
  PHASE_DOWNWARD_SEARCH,
  PHASE_REACHABLE_EA_QUERY,
  NUM_PHASES
} Phase;

constexpr const char* PhaseNames[] = {"Clear",
                                      "Initialization",
                                      "Connection scan",
                                      "Final transfers",
                                      "Final upward sweep",
                                      "Final downward search",
                                      "One Reachability-EA Query"};

typedef enum {
  METRIC_CONNECTIONS,
  METRIC_EDGES,
  METRIC_STOPS_BY_TRIP,
  METRIC_STOPS_BY_TRANSFER,
  NUM_METRICS
} Metric;

constexpr const char* MetricNames[] = {
    "Scanned connections",
    "Relaxed edges",
    "Updated stops by trip",
    "Updated stops by transfer",
};

class NoProfiler {
 public:
  inline void registerPhases(
      const std::initializer_list<Phase>&) const noexcept {}
  inline void registerMetrics(
      const std::initializer_list<Metric>&) const noexcept {}

  inline void initialize() const noexcept {}

  inline void start() const noexcept {}
  inline void done() const noexcept {}

  inline void startPhase() const noexcept {}
  inline void donePhase(const Phase) const noexcept {}

  inline void countMetric(const Metric) const noexcept {}
};

class SimpleProfiler : public NoProfiler {
 public:
  SimpleProfiler()
      : totalTime(0.0),
        phaseTime(NUM_PHASES, 0.0),
        metricValue(NUM_METRICS, 0) {}

  inline void registerPhases(
      const std::initializer_list<Phase>& phaseList) noexcept {
    for (const Phase phase : phaseList) {
      phases.push_back(phase);
    }
  }

  inline void registerMetrics(
      const std::initializer_list<Metric>& metricList) noexcept {
    for (const Metric metric : metricList) {
      metrics.push_back(metric);
    }
  }

  inline void initialize() noexcept {
    totalTime = 0.0;
    Vector::fill(phaseTime, 0.0);
    Vector::fill(metricValue, (size_t)0);
  }

  inline void start() noexcept {
    initialize();
    totalTimer.restart();
  }

  inline void done() noexcept {
    totalTime = totalTimer.elapsedMicroseconds();
    printStatistics();
  }

  inline void startPhase() noexcept { phaseTimer.restart(); }

  inline void donePhase(const Phase phase) noexcept {
    phaseTime[phase] += phaseTimer.elapsedMicroseconds();
  }

  inline void countMetric(const Metric metric) noexcept {
    metricValue[metric]++;
  }

  inline double getTotalTime() const noexcept { return totalTime; }

  inline double getPhaseTime(const Phase phase) const noexcept {
    return phaseTime[phase];
  }

  inline size_t getMetric(const Metric metric) const noexcept {
    return metricValue[metric];
  }

  inline SimpleProfiler& operator+=(const SimpleProfiler& other) noexcept {
    totalTime += other.totalTime;
    for (size_t i = 0; i < NUM_PHASES; i++) {
      phaseTime[i] += other.phaseTime[i];
    }
    for (size_t i = 0; i < NUM_METRICS; i++) {
      metricValue[i] += other.metricValue[i];
    }
    return *this;
  }

 private:
  inline void printStatistics() const noexcept {
    std::cout << std::endl;
    std::cout << "Total time: " << String::musToString(totalTime) << std::endl;
    for (const Phase phase : phases) {
      std::cout << "\t" << PhaseNames[phase] << ": "
                << String::musToString(phaseTime[phase]) << std::endl;
    }
    std::cout << std::endl;
    for (const Metric metric : metrics) {
      std::cout << MetricNames[metric] << ": "
                << String::prettyInt(metricValue[metric]) << std::endl;
    }
  }

 private:
  Timer totalTimer;
  double totalTime;
  Timer phaseTimer;
  std::vector<Phase> phases;
  std::vector<double> phaseTime;
  std::vector<Metric> metrics;
  std::vector<size_t> metricValue;
};

class AggregateProfiler : public NoProfiler {
 public:
  AggregateProfiler()
      : totalTime(0.0),
        phaseTime(NUM_PHASES, 0.0),
        metricValue(NUM_METRICS, 0),
        numQueries(0) {}

  inline void registerPhases(
      const std::initializer_list<Phase>& phaseList) noexcept {
    for (const Phase phase : phaseList) {
      phases.push_back(phase);
    }
  }

  inline void registerMetrics(
      const std::initializer_list<Metric>& metricList) noexcept {
    for (const Metric metric : metricList) {
      metrics.push_back(metric);
    }
  }

  inline void initialize() noexcept {
    totalTime = 0.0;
    Vector::fill(phaseTime, 0.0);
    Vector::fill(metricValue, (size_t)0);
    numQueries = 0;
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

  inline AggregateProfiler& operator+=(
      const AggregateProfiler& other) noexcept {
    totalTime += other.totalTime;
    for (size_t i = 0; i < NUM_PHASES; i++) {
      phaseTime[i] += other.phaseTime[i];
    }
    for (size_t i = 0; i < NUM_METRICS; i++) {
      metricValue[i] += other.metricValue[i];
    }
    numQueries += other.numQueries;
    return *this;
  }

  inline void printStatistics() const noexcept {
    std::cout << std::endl;
    std::cout << "Total time: " << String::musToString(getTotalTime())
              << std::endl;
    for (const Phase phase : phases) {
      std::cout << "\t" << PhaseNames[phase] << ": "
                << String::musToString(getPhaseTime(phase)) << std::endl;
    }
    std::cout << std::endl;
    for (const Metric metric : metrics) {
      std::cout << MetricNames[metric] << ": "
                << String::prettyDouble(getMetric(metric)) << std::endl;
    }
  }

 private:
  Timer totalTimer;
  double totalTime;
  Timer phaseTimer;
  std::vector<Phase> phases;
  std::vector<double> phaseTime;
  std::vector<Metric> metrics;
  std::vector<size_t> metricValue;
  size_t numQueries;
};

}  // namespace CSA
