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

#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "../../Algorithms/CSA/CSA.h"
#include "../../Algorithms/CSA/ProfileCSA.h"
#include "../../Algorithms/PPTL/Query.h"
#include "../../Algorithms/PTL/Query.h"
#include "../../Algorithms/RAPTOR/RAPTOR.h"
#include "../../Algorithms/TD/Query.h"
#include "../../Algorithms/TE/Query.h"
#include "../../Algorithms/TripBased/Query/ProfileOneToAllQuery.h"
#include "../../Algorithms/TripBased/Query/ProfileQuery.h"
#include "../../Algorithms/TripBased/Query/Query.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/PTL/Data.h"
#include "../../DataStructures/Queries/Queries.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TD/Data.h"
#include "../../DataStructures/TE/Data.h"
#include "../../DataStructures/TripBased/Data.h"
#include "../../Shell/Shell.h"

using namespace Shell;

class RunRAPTORQueries : public ParameterizedCommand {
public:
  RunRAPTORQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runRAPTORQueries",
            "Runs the given number of random RAPTOR queries.") {
    addParameter("RAPTOR input file");
    addParameter("Number of queries");
    addParameter("Number of rounds", "32");
  }

  virtual void execute() noexcept {
    const int maxRounds = getParameter<int>("Number of rounds");

    RAPTOR::Data raptorData =
        RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
    raptorData.useImplicitDepartureBufferTimes();
    raptorData.printInfo();
    RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false> algorithm(
        raptorData);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(raptorData.numberOfStops(), n);
    double numJourneys = 0;
    for (const StopQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target, maxRounds);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunOneRAPTORQuery : public ParameterizedCommand {
public:
  RunOneRAPTORQuery(BasicShell &shell)
      : ParameterizedCommand(shell, "runOneRAPTORQuery",
                             "Runs the given RAPTOR query.") {
    addParameter("RAPTOR input file");
    addParameter("Source StopId");
    addParameter("Target StopId");
    addParameter("Departure Time [HH:MM:SS]");
    addParameter("Number of rounds", "32");
  }

  virtual void execute() noexcept {
    const int maxRounds = getParameter<int>("Number of rounds");
    const StopId source = getParameter<StopId>("Source StopId");
    const StopId target = getParameter<StopId>("Target StopId");
    const std::string depTimeStr =
        getParameter<std::string>("Departure Time [HH:MM:SS]");
    const int departureTime = String::parseSeconds(depTimeStr);

    RAPTOR::Data raptorData =
        RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
    raptorData.useImplicitDepartureBufferTimes();
    raptorData.printInfo();
    RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false> algorithm(
        raptorData);

    algorithm.run(source, departureTime, target, maxRounds);
    algorithm.getProfiler().printStatistics();

    for (auto &j : algorithm.getJourneys()) {
      std::cout << "Journey (# trips: " << countTrips(j) << ")\n";

      for (auto &leg : j) {
        std::cout << leg << std::endl;
      }
    }
  }
};

class RunCSAQueries : public ParameterizedCommand {
public:
  RunCSAQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runCSAQueries",
            "Runs the given number of random CSA queries.") {
    addParameter("CSA input file");
    addParameter("Number of queries");
    addParameter("Target pruning?");
  }

  virtual void execute() noexcept {
    CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
    csaData.sortConnectionsAscending();
    csaData.printInfo();
    CSA::CSA<true, CSA::AggregateProfiler> algorithm(csaData);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(csaData.numberOfStops(), n);

    const bool targetPruning = getParameter<bool>("Target pruning?");

    for (const StopQuery &query : queries) {
      algorithm.run(query.source, query.departureTime,
                    targetPruning ? query.target : noStop);
    }
    algorithm.getProfiler().printStatistics();
  }
};

class RunProfileCSAQueries : public ParameterizedCommand {
public:
  RunProfileCSAQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runProfileCSAQueries",
            "Runs the given number of random ProfileCSA queries.") {
    addParameter("CSA input file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
    csaData.sortConnectionsAscending();
    csaData.printInfo();
    CSA::ProfileCSA<true, CSA::AggregateProfiler> algorithm(csaData);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(csaData.numberOfStops(), n);

    double numJourneys = 0;
    for (const StopQuery &query : queries) {
      algorithm.run(query.source, query.target, 0, 86400);
      numJourneys += algorithm.numberOfJourneys(query.source);
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunTripBasedQueries : public ParameterizedCommand {
public:
  RunTripBasedQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runTripBasedQueries",
            "Runs the given number of random TripBased queries.") {
    addParameter("Trip-Based input file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    const std::string tripFile = getParameter("Trip-Based input file");
    TripBased::Data tripBasedData(tripFile);
    tripBasedData.printInfo();
    TripBased::Query<TripBased::AggregateProfiler> algorithm(
        tripBasedData);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(tripBasedData.numberOfStops(), n);

    /* std::vector<std::vector<RAPTOR::Journey>> journeys = {}; */
    /* journeys.reserve(n); */

    double numberOfJourneys(0);

    for (const StopQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
      numberOfJourneys += algorithm.getJourneys().size();
      /* journeys.push_back(algorithm.getJourneys()); */
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numberOfJourneys / n)
              << std::endl;
  }
};

class CompareGeoToLength : public ParameterizedCommand {
public:
  CompareGeoToLength(BasicShell &shell)
      : ParameterizedCommand(
            shell, "compareGeoToLength",
            "Runs the given number of random TripBased queries and "
            "compares geo distance to number of events on path.") {
    addParameter("Trip-Based input file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    const std::string tripFile = getParameter("Trip-Based input file");
    TripBased::Data tbData(tripFile);
    tbData.printInfo();

    TripBased::Query<TripBased::NoProfiler> algorithm(tbData);

    const std::size_t n = getParameter<std::size_t>("Number of queries");

    // Column-major:
    // data[0][i]     = geo distance for query i
    // data[k][i]     = nrEvents for journeys with (k - 1) trips
    // k in [1, 17]
    std::array<std::vector<std::size_t>, 18> data;
    for (auto &col : data) {
      col.assign(n, 0);
    }

    const std::vector<StopQuery> queries =
        generateRandomStopQueries(tbData.numberOfStops(), n);

    for (std::size_t i = 0; i < n; i++) {
      const StopQuery &query = queries[i];

      const RAPTOR::Stop &fromStop = tbData.raptorData.stopData[query.source];
      const RAPTOR::Stop &toStop = tbData.raptorData.stopData[query.target];

      // Geo distance (stored in column 0)
      data[0][i] = static_cast<std::size_t>(
          geoDistanceInCM(fromStop.coordinates, toStop.coordinates) / 100.0);

      algorithm.run(query.source, query.departureTime, query.target);
      const auto journeys = algorithm.getJourneys();

      for (const auto &j : journeys) {
        const std::size_t nrTrips = countTrips(j);
        AssertMsg(nrTrips < 17, "NrTrips should be smaller than 17!");

        const std::size_t nrEvents = countEvents(j);

        const std::size_t col = nrTrips + 1;
        AssertMsg(col < data.size(), "NrTrips + 1 does not fit in array!");

        data[col][i] = nrEvents;
      }
    }

    // CSV output
    std::cout << "Index,GeoDistanceMeter";
    for (int j = 1; j < 18; j++) {
      std::cout << ",EventWith" << j << "Trips";
    }
    std::cout << "\n";

    for (std::size_t i = 0; i < n; i++) {
      std::cout << i << "," << data[0][i];
      for (int j = 1; j < 18; j++) {
        std::cout << "," << data[j][i];
      }
      std::cout << "\n";
    }
  }
};

class RunProfileTripBasedQueries : public ParameterizedCommand {
public:
  RunProfileTripBasedQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runProfileTripBasedQueries",
            "Runs the given number of random TripBased queries with "
            "a time range of [0, 24 hours).") {
    addParameter("Trip-Based input file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
    tripBasedData.printInfo();
    TripBased::ProfileQuery<TripBased::AggregateProfiler> algorithm(
        tripBasedData);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(tripBasedData.numberOfStops(), n);

    double numJourneys = 0;
    for (const StopQuery &query : queries) {
      algorithm.run(query.source, query.target, 0, 24 * 60 * 60 - 1);
      numJourneys += algorithm.getAllJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunProfileOneToAllTripBasedQueries
    : public ParameterizedCommand {
public:
  RunProfileOneToAllTripBasedQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runProfileOneToAllTripBasedQueries",
            "Runs the given number of random TripBased queries.") {
    addParameter("Trip-Based input file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
    tripBasedData.printInfo();
    TripBased::ProfileOneToAllQuery<TripBased::AggregateProfiler> algorithm(
        tripBasedData);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(tripBasedData.numberOfStops(), n);

    for (const StopQuery &query : queries) {
      algorithm.run(query.source, 0, 24 * 60 * 60 - 1);
    }
    algorithm.getProfiler().printStatistics();
  }
};

class RunGeoRankedTripBasedQueries : public ParameterizedCommand {
public:
  RunGeoRankedTripBasedQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runGeoRankedTripBasedQueries",
            "Runs TB queries to the 2^r th stop, where r is the geo rank. "
            "Source stops are chosen randomly.") {
    addParameter("TB input file");
    addParameter("Number of source stops");
    addParameter("Output csv file");
    addParameter("Lowest r");
  }

  virtual void execute() noexcept {
    const std::string file = getParameter("Output csv file");
    TripBased::Data tripBasedData(getParameter("TB input file"));
    tripBasedData.printInfo();

    TripBased::Query<TripBased::AggregateProfiler> algorithm(
        tripBasedData);

    const size_t n = getParameter<size_t>("Number of source stops");
    const int minR = getParameter<int>("Lowest r");

    AssertMsg(minR >= 0, "Lowest r must be >= 0");

    const size_t numStops = tripBasedData.numberOfStops();

    std::mt19937 randomGenerator(42);
    std::uniform_int_distribution<> stopDistribution(0, numStops - 1);
    std::uniform_int_distribution<> timeDistribution(0, (24 * 60 * 60) - 1);

    std::vector<StopId> sources;
    sources.reserve(n);

    for (size_t i = 0; i < n; i++) {
      sources.emplace_back(stopDistribution(randomGenerator));
    }

    // largest r such that 2^r < numStops
    int maxR = static_cast<int>(std::floor(std::log2(numStops - 1)));

    if (maxR < minR) {
      std::cout << "Too few stops; maxR < minR!" << std::endl;
      return;
    }

    const size_t numRanks = maxR - minR + 1;

    std::vector<double> queryRunTimes;
    queryRunTimes.reserve(n * numRanks);

    Progress progress(sources.size());

    for (StopId source : sources) {
      std::vector<size_t> allStopsSorted(numStops);
      std::iota(allStopsSorted.begin(), allStopsSorted.end(), 0);

      std::sort(allStopsSorted.begin(), allStopsSorted.end(),
                [&](size_t i1, size_t i2) {
                  return tripBasedData.raptorData.stopData[i1].dist(
                             tripBasedData.raptorData.stopData[source]) <
                         tripBasedData.raptorData.stopData[i2].dist(
                             tripBasedData.raptorData.stopData[source]);
                });

      for (int r = minR; r <= maxR; ++r) {
        const size_t idx = size_t(1) << r;
        assert(idx < allStopsSorted.size());

        StopId target = static_cast<StopId>(allStopsSorted[idx]);

        int depTime = timeDistribution(randomGenerator);

        algorithm.run(source, depTime, target);

        queryRunTimes.emplace_back(algorithm.getProfiler().getTotalTime());
        algorithm.getProfiler().reset();
      }

      progress++;
    }

    progress.finished();

    std::ofstream csv(file);
    AssertMsg(csv, "Cannot create output stream for " << file);
    AssertMsg(csv.is_open(), "Cannot open output stream for " << file);

    csv << "Index";
    for (int r = minR; r <= maxR; ++r) {
      csv << "," << r;
    }
    csv << "\n";

    auto it = queryRunTimes.begin();

    for (size_t i = 0; i < n; ++i) {
      csv << i;
      for (size_t j = 0; j < numRanks; ++j, ++it) {
        assert(it != queryRunTimes.end());
        csv << "," << *it;
      }
      csv << "\n";
    }

    assert(it == queryRunTimes.end());
  }
};

class RunTDDijkstraQueries : public ParameterizedCommand {
public:
  RunTDDijkstraQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "runTDDijkstraQueries",
                             "Runs the given number of random TDD queries.") {
    addParameter("TD input file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    TD::Data data = TD::Data::FromBinary(getParameter("TD input file"));
    data.printInfo();
    TD::EADijkstra<TimeDependentRouteGraph, TD::AggregateProfiler> algorithm(
        data.timeDependentGraph, DurationFunction);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(data.numberOfStops(), n);

    for (const StopQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
    }
    algorithm.getProfiler().printStatistics();
  }
};

class RunTEDijkstraQueries : public ParameterizedCommand {
public:
  RunTEDijkstraQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "runTEDijkstraQueries",
                             "Runs the given number of random TDD queries.") {
    addParameter("TE input file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    TE::Data data = TE::Data::FromBinary(getParameter("TE input file"));
    data.printInfo();
    TE::Query<TE::AggregateProfiler> algorithm(data);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(data.numberOfStops(), n);

    for (const StopQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
    }
    algorithm.getProfiler().printStatistics();
  }
};

class RunPTLQueries : public ParameterizedCommand {
public:
  RunPTLQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "runPTLQueries",
                             "Runs the given number of random PTL queries.") {
    addParameter("PTL input file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    PTL::Data data = PTL::Data::FromBinary(getParameter("PTL input file"));
    data.printInfo();
    PTL::Query<PTL::AggregateProfiler> algorithmLinear(data);
    PTL::Query<PTL::AggregateProfiler> algorithmBinary(data);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(data.numberOfStops(), n);

    for (const StopQuery &query : queries) {
      algorithmLinear.template run<false>(query.source, query.departureTime,
                                          query.target);
      algorithmBinary.run(query.source, query.departureTime, query.target);
    }
    std::cout << "Linear Search through Target Arrival Events:" << std::endl;
    algorithmLinear.getProfiler().printStatistics();
    std::cout << "Binary Search through Target Arrival Events:" << std::endl;
    algorithmBinary.getProfiler().printStatistics();
  }
};

class RunPPTLQueries : public ParameterizedCommand {
public:
  RunPPTLQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "runPPTLQueries",
                             "Runs the given number of random PPTL queries.") {
    addParameter("PPTL input file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    PPTL::Data data = PPTL::Data::FromBinary(getParameter("PPTL input file"));
    data.printInfo();
    PPTL::Query<PPTL::AggregateProfiler> algorithmLinear(data);
    PPTL::Query<PPTL::AggregateProfiler> algorithmBinary(data);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(data.numberOfStops(), n);

    for (const StopQuery &query : queries) {
      algorithmLinear.template run<false>(query.source, query.departureTime,
                                          query.target);
      algorithmBinary.run(query.source, query.departureTime, query.target);
    }
    std::cout << "Linear Search through Target Arrival Events:" << std::endl;
    algorithmLinear.getProfiler().printStatistics();
    std::cout << "Binary Search through Target Arrival Events:" << std::endl;
    algorithmBinary.getProfiler().printStatistics();
  }
};
