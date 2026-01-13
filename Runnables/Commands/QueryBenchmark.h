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
#include "../../Algorithms/CSA/DijkstraCSA.h"
#include "../../Algorithms/CSA/HLCSA.h"
#include "../../Algorithms/CSA/ProfileCSA.h"
#include "../../Algorithms/CSA/ULTRACSA.h"
#include "../../Algorithms/PPTL/Query.h"
#include "../../Algorithms/PTL/Query.h"
#include "../../Algorithms/RAPTOR/Bounded/BoundedMcRAPTOR.h"
#include "../../Algorithms/RAPTOR/DijkstraRAPTOR.h"
#include "../../Algorithms/RAPTOR/HLRAPTOR.h"
#include "../../Algorithms/RAPTOR/InitialTransfers.h"
#include "../../Algorithms/RAPTOR/MCR.h"
#include "../../Algorithms/RAPTOR/McRAPTOR.h"
#include "../../Algorithms/RAPTOR/MultimodalMCR.h"
#include "../../Algorithms/RAPTOR/MultimodalULTRAMcRAPTOR.h"
#include "../../Algorithms/RAPTOR/RAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRABounded/MultimodalUBMHydRA.h"
#include "../../Algorithms/RAPTOR/ULTRABounded/MultimodalUBMRAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRABounded/UBMHydRA.h"
#include "../../Algorithms/RAPTOR/ULTRABounded/UBMRAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRAMcRAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRARAPTOR.h"
#include "../../Algorithms/TD/Query.h"
#include "../../Algorithms/TE/Query.h"
#include "../../Algorithms/TripBased/BoundedMcQuery/BoundedMcQuery.h"
#include "../../Algorithms/TripBased/Query/McQuery.h"
#include "../../Algorithms/TripBased/Query/ProfileOneToAllQuery.h"
#include "../../Algorithms/TripBased/Query/ProfileQuery.h"
#include "../../Algorithms/TripBased/Query/Query.h"
#include "../../Algorithms/TripBased/Query/TransitiveQuery.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/PTL/Data.h"
#include "../../DataStructures/Queries/Queries.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/MultimodalData.h"
#include "../../DataStructures/TD/Data.h"
#include "../../DataStructures/TE/Data.h"
#include "../../DataStructures/TripBased/Data.h"
#include "../../DataStructures/TripBased/MultimodalData.h"
#include "../../Shell/Shell.h"

using namespace Shell;

class RunTransitiveRAPTORQueries : public ParameterizedCommand {
public:
  RunTransitiveRAPTORQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runTransitiveRAPTORQueries",
            "Runs the given number of random transitive RAPTOR queries.") {
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

class RunOneTransitiveRAPTORQuery : public ParameterizedCommand {
public:
  RunOneTransitiveRAPTORQuery(BasicShell &shell)
      : ParameterizedCommand(shell, "runOneTransitiveRAPTORQuery",
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

class RunDijkstraRAPTORQueries : public ParameterizedCommand {
public:
  RunDijkstraRAPTORQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runDijkstraRAPTORQueries",
            "Runs the given number of random Dijkstra RAPTOR queries.") {
    addParameter("RAPTOR input file");
    addParameter("CH data");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    RAPTOR::Data raptorData =
        RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
    raptorData.useImplicitDepartureBufferTimes();
    raptorData.printInfo();
    CH::CH ch(getParameter("CH data"));
    RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers,
                           RAPTOR::AggregateProfiler, true, false>
        algorithm(raptorData, ch);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(ch.numVertices(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunULTRARAPTORQueries : public ParameterizedCommand {
public:
  RunULTRARAPTORQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runULTRARAPTORQueries",
            "Runs the given number of random ULTRA-RAPTOR queries.") {
    addParameter("RAPTOR input file");
    addParameter("CH data");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    RAPTOR::Data raptorData =
        RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
    raptorData.useImplicitDepartureBufferTimes();
    raptorData.printInfo();
    CH::CH ch(getParameter("CH data"));
    RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algorithm(raptorData,
                                                                    ch);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(ch.numVertices(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunHLRAPTORQueries : public ParameterizedCommand {
public:
  RunHLRAPTORQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runHLRAPTORQueries",
            "Runs the given number of random HL-RAPTOR queries.") {
    addParameter("RAPTOR input file");
    addParameter("Out-hub file");
    addParameter("In-hub file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    RAPTOR::Data raptorData =
        RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
    raptorData.useImplicitDepartureBufferTimes();
    raptorData.printInfo();
    const TransferGraph outHubs(getParameter("Out-hub file"));
    const TransferGraph inHubs(getParameter("In-hub file"));
    RAPTOR::HLRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, outHubs,
                                                          inHubs);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(inHubs.numVertices(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunTransitiveMcRAPTORQueries : public ParameterizedCommand {
public:
  RunTransitiveMcRAPTORQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runTransitiveMcRAPTORQueries",
            "Runs the given number of random transitive McRAPTOR queries.") {
    addParameter("RAPTOR input file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    RAPTOR::Data raptorData =
        RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
    raptorData.useImplicitDepartureBufferTimes();
    raptorData.printInfo();
    RAPTOR::McRAPTOR<true, true, RAPTOR::AggregateProfiler> algorithm(
        raptorData);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(raptorData.numberOfStops(), n);

    double numJourneys = 0;
    for (const StopQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunTransitiveBoundedMcRAPTORQueries : public ParameterizedCommand {
public:
  RunTransitiveBoundedMcRAPTORQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "runTransitiveBoundedMcRAPTORQueries",
                             "Runs the given number of random transitive "
                             "Bounded McRAPTOR queries.") {
    addParameter("RAPTOR input file");
    addParameter("Number of queries");
    addParameter("Arrival slack");
    addParameter("Trip slack");
  }

  virtual void execute() noexcept {
    RAPTOR::Data raptorData =
        RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
    raptorData.useImplicitDepartureBufferTimes();
    raptorData.printInfo();
    const RAPTOR::Data reverseData = raptorData.reverseNetwork();
    RAPTOR::BoundedMcRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData,
                                                                 reverseData);

    const double arrivalSlack = getParameter<double>("Arrival slack");
    const double tripSlack = getParameter<double>("Trip slack");

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(raptorData.numberOfStops(), n);

    double numJourneys = 0;
    for (const StopQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target,
                    arrivalSlack, tripSlack);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunMCRQueries : public ParameterizedCommand {
public:
  RunMCRQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "runMCRQueries",
                             "Runs the given number of random MCR queries.") {
    addParameter("RAPTOR input file");
    addParameter("CH data");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    RAPTOR::Data raptorData =
        RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
    raptorData.useImplicitDepartureBufferTimes();
    raptorData.printInfo();
    CH::CH ch(getParameter("CH data"));
    RAPTOR::MCR<true, RAPTOR::AggregateProfiler> algorithm(raptorData, ch);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(ch.numVertices(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunMultimodalMCRQueries : public ParameterizedCommand {
public:
  RunMultimodalMCRQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runMultimodalMCRQueries",
            "Runs the given number of random multimodal MCR queries.") {
    addParameter("RAPTOR input file");
    addParameter("CH directory");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    RAPTOR::MultimodalData raptorData(getParameter("RAPTOR input file"));
    raptorData.useImplicitDepartureBufferTimes();
    raptorData.printInfo();
    switch (raptorData.modes.size()) {
    case 2:
      run<2>(raptorData);
      break;
    case 3:
      run<3>(raptorData);
      break;
    default:
      Ensure(false, "Unsupported number of modes!");
      break;
    }
  }

private:
  template <size_t NUM_MODES>
  inline void run(const RAPTOR::MultimodalData &raptorData) const noexcept {
    const std::string chDirectory(getParameter("CH directory"));
    std::vector<CH::CH> chData;
    for (const size_t mode : raptorData.modes) {
      chData.emplace_back(chDirectory + RAPTOR::TransferModeNames[mode] + "CH");
    }
    RAPTOR::MultimodalMCR<true, NUM_MODES, RAPTOR::AggregateProfiler> algorithm(
        raptorData, chData);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(chData[0].numVertices(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunULTRAMcRAPTORQueries : public ParameterizedCommand {
public:
  RunULTRAMcRAPTORQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runULTRAMcRAPTORQueries",
            "Runs the given number of random ULTRA-McRAPTOR queries.") {
    addParameter("RAPTOR input file");
    addParameter("CH data");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    RAPTOR::Data raptorData =
        RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
    raptorData.useImplicitDepartureBufferTimes();
    raptorData.printInfo();
    CH::CH ch(getParameter("CH data"));
    RAPTOR::ULTRAMcRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, ch);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(ch.numVertices(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunMultimodalULTRAMcRAPTORQueries : public ParameterizedCommand {
public:
  RunMultimodalULTRAMcRAPTORQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "runMultimodalULTRAMcRAPTORQueries",
                             "Runs the given number of random multimodal "
                             "ULTRA-McRAPTOR queries.") {
    addParameter("RAPTOR input file");
    addParameter("CH directory");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    RAPTOR::MultimodalData raptorData(getParameter("RAPTOR input file"));
    raptorData.useImplicitDepartureBufferTimes();
    raptorData.printInfo();
    switch (raptorData.modes.size()) {
    case 2:
      run<2>(raptorData);
      break;
    case 3:
      run<3>(raptorData);
      break;
    default:
      Ensure(false, "Unsupported number of modes!");
      break;
    }
  }

private:
  template <size_t NUM_MODES>
  inline void run(const RAPTOR::MultimodalData &raptorData) const noexcept {
    const std::string chDirectory(getParameter("CH directory"));
    std::vector<CH::CH> chData;
    for (const size_t mode : raptorData.modes) {
      chData.emplace_back(chDirectory + RAPTOR::TransferModeNames[mode] + "CH");
    }
    RAPTOR::MultimodalULTRAMcRAPTOR<NUM_MODES, RAPTOR::AggregateProfiler>
        algorithm(raptorData, chData);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(chData[0].numVertices(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunUBMRAPTORQueries : public ParameterizedCommand {
public:
  RunUBMRAPTORQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runUBMRAPTORQueries",
            "Runs the given number of random UBM-RAPTOR queries.") {
    addParameter("RAPTOR input file");
    addParameter("CH data");
    addParameter("Number of queries");
    addParameter("Arrival slack");
    addParameter("Trip slack");
  }

  virtual void execute() noexcept {
    RAPTOR::Data raptorData =
        RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
    raptorData.useImplicitDepartureBufferTimes();
    raptorData.printInfo();
    const RAPTOR::Data reverseData = raptorData.reverseNetwork();
    CH::CH ch(getParameter("CH data"));
    RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData,
                                                           reverseData, ch);

    const double arrivalSlack = getParameter<double>("Arrival slack");
    const double tripSlack = getParameter<double>("Trip slack");

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(ch.numVertices(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target,
                    arrivalSlack, tripSlack);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunUBMHydRAQueries : public ParameterizedCommand {
public:
  RunUBMHydRAQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runUBMHydRAQueries",
            "Runs the given number of random UBM-HydRA queries.") {
    addParameter("Trip-Based input file");
    addParameter("Bounded forward Trip-Based input file");
    addParameter("Bounded backward Trip-Based input file");
    addParameter("CH data");
    addParameter("Number of queries");
    addParameter("Arrival slack");
    addParameter("Trip slack");
  }

  virtual void execute() noexcept {
    const TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
    tripBasedData.printInfo();
    const TripBased::Data forwardBoundedData(
        getParameter("Bounded forward Trip-Based input file"));
    forwardBoundedData.printInfo();
    const TripBased::Data backwardBoundedData(
        getParameter("Bounded backward Trip-Based input file"));
    backwardBoundedData.printInfo();
    const CH::CH ch(getParameter("CH data"));

    RAPTOR::UBMHydRA<RAPTOR::AggregateProfiler> algorithm(
        tripBasedData, forwardBoundedData, backwardBoundedData, ch);

    const double arrivalSlack = getParameter<double>("Arrival slack");
    const double tripSlack = getParameter<double>("Trip slack");

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(ch.numVertices(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target,
                    arrivalSlack, tripSlack);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunMultimodalUBMRAPTORQueries : public ParameterizedCommand {
public:
  RunMultimodalUBMRAPTORQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runMultimodalUBMRAPTORQueries",
            "Runs the given number of random multimodal UBM-RAPTOR queries.") {
    addParameter("RAPTOR input file");
    addParameter("CH directory");
    addParameter("Number of queries");
    addParameter("Arrival slack");
    addParameter("Trip slack");
  }

  virtual void execute() noexcept {
    RAPTOR::MultimodalData raptorData(getParameter("RAPTOR input file"));
    raptorData.useImplicitDepartureBufferTimes();
    raptorData.printInfo();
    switch (raptorData.modes.size()) {
    case 2:
      run<2>(raptorData);
      break;
    case 3:
      run<3>(raptorData);
      break;
    default:
      Ensure(false, "Unsupported number of modes!");
      break;
    }
  }

private:
  template <size_t NUM_MODES>
  inline void run(const RAPTOR::MultimodalData &raptorData) const noexcept {
    const RAPTOR::Data pruningData = raptorData.getPruningData();
    const RAPTOR::Data reversePruningData = pruningData.reverseNetwork();
    const std::string chDirectory(getParameter("CH directory"));
    std::vector<CH::CH> chData;
    for (const size_t mode : raptorData.modes) {
      chData.emplace_back(chDirectory + RAPTOR::TransferModeNames[mode] + "CH");
    }
    RAPTOR::TransferGraph backwardTransitiveGraph =
        raptorData.raptorData.transferGraph;
    backwardTransitiveGraph.revert();
    RAPTOR::MultimodalUBMRAPTOR<NUM_MODES, RAPTOR::AggregateProfiler> algorithm(
        raptorData, pruningData, reversePruningData, backwardTransitiveGraph,
        chData);

    const double arrivalSlack = getParameter<double>("Arrival slack");
    const double tripSlack = getParameter<double>("Trip slack");

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(chData[0].numVertices(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target,
                    arrivalSlack, tripSlack);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunMultimodalUBMHydRAQueries : public ParameterizedCommand {
public:
  RunMultimodalUBMHydRAQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runMultimodalUBMHydRAQueries",
            "Runs the given number of random multimodal UBM-HydRA queries.") {
    addParameter("Trip-Based input file");
    addParameter("Bounded forward Trip-Based input file");
    addParameter("Bounded backward Trip-Based input file");
    addParameter("CH directory");
    addParameter("Number of queries");
    addParameter("Arrival slack");
    addParameter("Trip slack");
  }

  virtual void execute() noexcept {
    const TripBased::MultimodalData tripBasedData(
        getParameter("Trip-Based input file"));
    tripBasedData.printInfo();
    switch (tripBasedData.modes.size()) {
    case 2:
      run<2>(tripBasedData);
      break;
    case 3:
      run<3>(tripBasedData);
      break;
    default:
      Ensure(false, "Unsupported number of modes!");
      break;
    }
  }

private:
  template <size_t NUM_MODES>
  inline void
  run(const TripBased::MultimodalData &tripBasedData) const noexcept {
    const TripBased::MultimodalData forwardBoundedData(
        getParameter("Bounded forward Trip-Based input file"));
    forwardBoundedData.printInfo();
    Ensure(forwardBoundedData.modes == tripBasedData.modes,
           "Different transfer modes!");
    const TripBased::Data forwardPruningData =
        forwardBoundedData.getPruningData();
    const TripBased::MultimodalData backwardBoundedData(
        getParameter("Bounded backward Trip-Based input file"));
    backwardBoundedData.printInfo();
    Ensure(backwardBoundedData.modes == tripBasedData.modes,
           "Different transfer modes!");
    const TripBased::Data backwardPruningData =
        backwardBoundedData.getPruningData();
    const std::string chDirectory(getParameter("CH directory"));
    std::vector<CH::CH> chData;
    for (const size_t mode : tripBasedData.modes) {
      chData.emplace_back(chDirectory + RAPTOR::TransferModeNames[mode] + "CH");
    }
    RAPTOR::TransferGraph backwardTransitiveGraph =
        tripBasedData.tripData.raptorData.transferGraph;
    backwardTransitiveGraph.revert();
    RAPTOR::MultimodalUBMHydRA<NUM_MODES, RAPTOR::AggregateProfiler> algorithm(
        tripBasedData, forwardPruningData, backwardPruningData,
        backwardTransitiveGraph, chData);

    const double arrivalSlack = getParameter<double>("Arrival slack");
    const double tripSlack = getParameter<double>("Trip slack");

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(chData[0].numVertices(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target,
                    arrivalSlack, tripSlack);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunTransitiveCSAQueries : public ParameterizedCommand {
public:
  RunTransitiveCSAQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runTransitiveCSAQueries",
            "Runs the given number of random transitive CSA queries.") {
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

class RunTransitiveProfileCSAQueries : public ParameterizedCommand {
public:
  RunTransitiveProfileCSAQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runTransitiveProfileCSAQueries",
            "Runs the given number of random transitive ProfileCSA queries.") {
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

class RunDijkstraCSAQueries : public ParameterizedCommand {
public:
  RunDijkstraCSAQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runDijkstraCSAQueries",
            "Runs the given number of random Dijkstra-CSA queries.") {
    addParameter("CSA input file");
    addParameter("CH data");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
    csaData.sortConnectionsAscending();
    csaData.printInfo();
    CH::CH ch(getParameter("CH data"));
    CSA::DijkstraCSA<RAPTOR::CoreCHInitialTransfers, true,
                     CSA::AggregateProfiler>
        algorithm(csaData, ch);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(ch.numVertices(), n);

    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
    }
    algorithm.getProfiler().printStatistics();
  }
};

class RunULTRACSAQueries : public ParameterizedCommand {
public:
  RunULTRACSAQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runULTRACSAQueries",
            "Runs the given number of random ULTRA-CSA queries.") {
    addParameter("CSA input file");
    addParameter("CH data");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
    csaData.sortConnectionsAscending();
    csaData.printInfo();
    CH::CH ch(getParameter("CH data"));
    CSA::ULTRACSA<true, CSA::AggregateProfiler> algorithm(csaData, ch);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(ch.numVertices(), n);

    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
    }
    algorithm.getProfiler().printStatistics();
  }
};

class RunHLCSAQueries : public ParameterizedCommand {
public:
  RunHLCSAQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runHLCSAQueries",
            "Runs the given number of random HL-CSA queries.") {
    addParameter("CSA input file");
    addParameter("Out-hub file");
    addParameter("In-hub file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
    csaData.sortConnectionsAscending();
    csaData.printInfo();
    const TransferGraph outHubs(getParameter("Out-hub file"));
    const TransferGraph inHubs(getParameter("In-hub file"));
    CSA::HLCSA<CSA::AggregateProfiler> algorithm(csaData, outHubs, inHubs);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(inHubs.numVertices(), n);

    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
    }
    algorithm.getProfiler().printStatistics();
  }
};

class RunTransitiveTripBasedQueries : public ParameterizedCommand {
public:
  RunTransitiveTripBasedQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runTransitiveTripBasedQueries",
            "Runs the given number of random transitive TripBased queries.") {
    addParameter("Trip-Based input file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    const std::string tripFile = getParameter("Trip-Based input file");
    TripBased::Data tripBasedData(tripFile);
    tripBasedData.printInfo();
    TripBased::TransitiveQuery<TripBased::AggregateProfiler> algorithm(
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
            "Runs the given number of random transitive TripBased queries and "
            "compares geo distance to number of events on path.") {
    addParameter("Trip-Based input file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    const std::string tripFile = getParameter("Trip-Based input file");
    TripBased::Data tbData(tripFile);
    tbData.printInfo();

    TripBased::TransitiveQuery<TripBased::NoProfiler> algorithm(tbData);

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

/* class RunTransitiveTripBasedQueryExplizitly : public ParameterizedCommand {
 */
/* public: */
/*     RunTransitiveTripBasedQueryExplizitly(BasicShell& shell) */
/*         : ParameterizedCommand(shell,
 * "runTransitiveTripBasedQueryExplizitly", */
/*             "Runs the given query on the transitive TripBased Algorithm.") */
/*     { */
/*         addParameter("Trip-Based input file"); */
/* 	addParameter("FromStopId"); */
/* 	addParameter("ToStopId"); */
/* 	addParameter("DepartureTime"); */
/*     } */

/*     virtual void execute() noexcept */
/*     { */
/*         const std::string tripFile = getParameter("Trip-Based input file");
 */
/* 	const StopId source = StopId(getParameter<int>("FromStopId")); */
/* 	const StopId target = StopId(getParameter<int>("ToStopId")); */
/* 	const int departureTime = getParameter<int>("DepartureTime"); */
/*         TripBased::Data tripBasedData(tripFile); */
/*         tripBasedData.printInfo(); */
/*         TripBased::TransitiveQuery<TripBased::AggregateProfiler>
 * algorithm(tripBasedData); */

/* 	algorithm.run(source, departureTime, target); */
/* 	for (auto j : algorithm.getJourneys()) { */
/* 		std::cout << "New Journey:" << std::endl; */
/* 		for (auto leg : j) std::cout << leg << std::endl; */
/* 		std::cout << std::endl; */
/* 	} */
/*         algorithm.getProfiler().printStatistics(); */

/*     } */
/* }; */

class RunTransitiveProfileTripBasedQueries : public ParameterizedCommand {
public:
  RunTransitiveProfileTripBasedQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runTransitiveProfileTripBasedQueries",
            "Runs the given number of random transitive TripBased queries with "
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

class RunTransitiveProfileOneToAllTripBasedQueries
    : public ParameterizedCommand {
public:
  RunTransitiveProfileOneToAllTripBasedQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runTransitiveProfileOneToAllTripBasedQueries",
            "Runs the given number of random transitive TripBased queries.") {
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

class RunULTRATripBasedQueries : public ParameterizedCommand {
public:
  RunULTRATripBasedQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runULTRATripBasedQueries",
            "Runs the given number of random ULTRA-TripBased queries.") {
    addParameter("Trip-Based input file");
    addParameter("CH data");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
    tripBasedData.printInfo();
    CH::CH ch(getParameter("CH data"));
    TripBased::Query<TripBased::AggregateProfiler> algorithm(tripBasedData, ch);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(ch.numVertices(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunULTRAMcTripBasedQueries : public ParameterizedCommand {
public:
  RunULTRAMcTripBasedQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runULTRAMcTripBasedQueries",
            "Runs the given number of random ULTRA-McTripBased queries.") {
    addParameter("Trip-Based input file");
    addParameter("CH data");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
    tripBasedData.printInfo();
    CH::CH ch(getParameter("CH data"));
    TripBased::McQuery<TripBased::AggregateProfiler> algorithm(tripBasedData,
                                                               ch);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(ch.numVertices(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class RunBoundedULTRAMcTripBasedQueries : public ParameterizedCommand {
public:
  RunBoundedULTRAMcTripBasedQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "runBoundedULTRAMcTripBasedQueries",
                             "Runs the given number of random Bounded "
                             "ULTRA-McTripBased queries.") {
    addParameter("Trip-Based input file");
    addParameter("Bounded forward Trip-Based input file");
    addParameter("Bounded backward Trip-Based input file");
    addParameter("CH data");
    addParameter("Number of queries");
    addParameter("Arrival slack");
    addParameter("Trip slack");
  }

  virtual void execute() noexcept {
    TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
    tripBasedData.printInfo();
    TripBased::Data forwardBoundedData(
        getParameter("Bounded forward Trip-Based input file"));
    forwardBoundedData.printInfo();
    TripBased::Data backwardBoundedData(
        getParameter("Bounded backward Trip-Based input file"));
    backwardBoundedData.printInfo();
    CH::CH ch(getParameter("CH data"));
    TripBased::BoundedMcQuery<TripBased::AggregateProfiler> algorithm(
        tripBasedData, forwardBoundedData, backwardBoundedData, ch);

    const double arrivalSlack = getParameter<double>("Arrival slack");
    const double tripSlack = getParameter<double>("Trip slack");

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(ch.numVertices(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target,
                    arrivalSlack, tripSlack);
      numJourneys += algorithm.getJourneys().size();
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

class ComputeTransferTimeSavings : public ParameterizedCommand {
public:
  ComputeTransferTimeSavings(BasicShell &shell)
      : ParameterizedCommand(
            shell, "computeTransferTimeSavings",
            "Computes the savings in transfer time of a 3-criteria (bounded) "
            "Pareto set compared to a 2-criteria one.") {
    addParameter("RAPTOR input file");
    addParameter("CH data");
    addParameter("Number of queries");
    addParameter("Output file");
  }

  virtual void execute() noexcept {
    RAPTOR::Data raptorData =
        RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
    raptorData.useImplicitDepartureBufferTimes();
    raptorData.printInfo();
    const RAPTOR::Data reverseData = raptorData.reverseNetwork();
    CH::CH ch(getParameter("CH data"));
    RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData,
                                                           reverseData, ch);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(ch.numVertices(), n);

    IO::OFStream outputFile(getParameter("Output file"));
    outputFile << "ArrivalSlack";
    for (const double tripSlack : tripSlacks) {
      const int slackAsInt = tripSlack * 100 - 100;
      for (const double threshold : thresholds) {
        const int thresholdAsInt = threshold * 100;
        outputFile << "\tTripSlack" << slackAsInt << "Savings"
                   << thresholdAsInt;
      }
    }
    outputFile << "\n";
    outputFile.flush();

    for (const double arrivalSlack : arrivalSlacks) {
      outputFile << arrivalSlack;
      for (const double tripSlack : tripSlacks) {
        std::cout << "Arrival slack: " << arrivalSlack
                  << ", trip slack: " << tripSlack << std::endl;
        std::vector<double> transferTimeSavings;
        for (const VertexQuery &query : queries) {
          algorithm.run(query.source, query.departureTime, query.target,
                        arrivalSlack, tripSlack);
          const std::vector<RAPTOR::WalkingParetoLabel> fullLabels =
              algorithm.getResults();
          const std::vector<RAPTOR::ArrivalLabel> &anchorLabels =
              algorithm.getAnchorLabels();
          RAPTOR::WalkingParetoLabel bestLabel;
          RAPTOR::WalkingParetoLabel bestAnchorLabel;
          for (const RAPTOR::WalkingParetoLabel &label : fullLabels) {
            if (label.walkingDistance <= bestLabel.walkingDistance) {
              bestLabel = label;
            }
            if (label.walkingDistance <= bestAnchorLabel.walkingDistance &&
                isAnchorLabel(label, anchorLabels)) {
              bestAnchorLabel = label;
            }
          }
          if (bestAnchorLabel.walkingDistance == 0) {
            transferTimeSavings.emplace_back(0);
          } else {
            transferTimeSavings.emplace_back(
                (bestAnchorLabel.walkingDistance - bestLabel.walkingDistance) /
                static_cast<double>(bestAnchorLabel.walkingDistance));
          }
        }
        std::sort(transferTimeSavings.begin(), transferTimeSavings.end(),
                  [&](const double a, const double b) { return a > b; });
        size_t j = 0;
        std::vector<size_t> savingsCount(thresholds.size(), 0);
        for (const double s : transferTimeSavings) {
          while (s < thresholds[j]) {
            j++;
            if (j == thresholds.size())
              break;
          }
          if (j == thresholds.size())
            break;
          savingsCount[j]++;
        }
        for (const size_t c : savingsCount) {
          const double ratio =
              c / static_cast<double>(transferTimeSavings.size());
          outputFile << "\t" << ratio;
        }
      }
      outputFile << "\n";
      outputFile.flush();
    }
  }

private:
  std::vector<double> thresholds{0.75, 0.5, 0.25};
  std::vector<double> arrivalSlacks{1, 1.1, 1.2, 1.3, 1.4, 1.5};
  std::vector<double> tripSlacks{1, 1.25, 1.5};

  inline bool isAnchorLabel(
      const RAPTOR::WalkingParetoLabel &label,
      const std::vector<RAPTOR::ArrivalLabel> &anchorLabels) const noexcept {
    for (const RAPTOR::ArrivalLabel &anchorLabel : anchorLabels) {
      if (label.arrivalTime != anchorLabel.arrivalTime)
        continue;
      if (label.numberOfTrips != anchorLabel.numberOfTrips)
        continue;
      return true;
    }
    return false;
  }
};

class RunGeoRankedRAPTORQueries : public ParameterizedCommand {
public:
  RunGeoRankedRAPTORQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runGeoRankedRAPTORQueries",
            "Runs RAPTOR queries to the 2^r th stop, where r is the geo rank. "
            "Source stops are chosen randomly.") {
    addParameter("RAPTOR input file");
    addParameter("Number of source stops");
    addParameter("Output csv file");
    addParameter("Lowest r");
  }

  virtual void execute() noexcept {
    const std::string file = getParameter("Output csv file");
    RAPTOR::Data raptor =
        RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
    raptor.useImplicitDepartureBufferTimes();
    raptor.printInfo();
    RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false> algorithm(
        raptor);

    const size_t n = getParameter<size_t>("Number of source stops");
    const int minR = getParameter<int>("Lowest r");

    std::mt19937 randomGenerator(42);
    std::uniform_int_distribution<> stopDistribution(0, raptor.numberOfStops() -
                                                            1);
    std::uniform_int_distribution<> timeDistribution(0, (24 * 60 * 60) - 1);

    std::vector<StopId> sources;
    sources.reserve(n);

    for (size_t i = 0; i < n; i++) {
      sources.emplace_back(stopDistribution(randomGenerator));
    }

    int maxR = std::floor(std::log2(raptor.numberOfStops()));

    if (maxR <= minR) {
      std::cout << "Too few stops; maxR <= minR!" << std::endl;
      return;
    }

    std::vector<double> queryRunTimes;
    queryRunTimes.reserve(n * (maxR - minR + 1));

    for (auto &source : sources) {
      std::vector<size_t> allStopsSorted(raptor.numberOfStops());
      std::iota(allStopsSorted.begin(), allStopsSorted.end(), 0);

      std::sort(allStopsSorted.begin(), allStopsSorted.end(),
                [&](int i1, int i2) {
                  return raptor.stopData[i1].dist(raptor.stopData[source]) <
                         raptor.stopData[i2].dist(raptor.stopData[source]);
                });

      for (int r = minR; r <= maxR; ++r) {
        if (static_cast<size_t>(1 << r) >= allStopsSorted.size()) {
          std::cout << "TOOO MUCH!! r: " << r << " vs " << allStopsSorted.size()
                    << std::endl;
          break;
        }
        auto target = allStopsSorted[(1 << r)];

        int depTime = timeDistribution(randomGenerator);

        algorithm.run(static_cast<StopId>(source), depTime,
                      static_cast<StopId>(target));
        queryRunTimes.emplace_back(algorithm.getProfiler().totalTime);
        algorithm.getProfiler().reset();
      }
    }

    std::ofstream csv(file);
    AssertMsg(csv, "Cannot create output stream for " << file);
    AssertMsg(csv.is_open(), "Cannot open output stream for " << file);

    csv << "Index";

    for (int r = minR; r <= maxR; ++r) {
      csv << "," << r;
    }
    csv << "\n";

    size_t i = 0;

    auto it = queryRunTimes.begin();

    while (i < n) {
      csv << i;
      for (int r = minR; r <= maxR; ++r, ++it) {
        assert(it != queryRunTimes.end());
        csv << "," << (*it);
      }
      csv << "\n";
      ++i;
    }
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
    TripBased::TransitiveQuery<TripBased::AggregateProfiler> algorithm(
        tripBasedData);

    const size_t n = getParameter<size_t>("Number of source stops");
    const int minR = getParameter<int>("Lowest r");

    std::mt19937 randomGenerator(42);
    std::uniform_int_distribution<> stopDistribution(
        0, tripBasedData.numberOfStops() - 1);
    std::uniform_int_distribution<> timeDistribution(0, (24 * 60 * 60) - 1);

    std::vector<StopId> sources;
    sources.reserve(n);

    for (size_t i = 0; i < n; i++) {
      sources.emplace_back(stopDistribution(randomGenerator));
    }

    int maxR = std::floor(std::log2(tripBasedData.numberOfStops()));

    if (maxR <= minR) {
      std::cout << "Too few stops; maxR <= minR!" << std::endl;
      return;
    }

    std::vector<double> queryRunTimes;
    queryRunTimes.reserve(n * (maxR - minR + 1));

    for (auto &source : sources) {
      std::vector<size_t> allStopsSorted(tripBasedData.numberOfStops());
      std::iota(allStopsSorted.begin(), allStopsSorted.end(), 0);

      std::sort(allStopsSorted.begin(), allStopsSorted.end(),
                [&](int i1, int i2) {
                  return tripBasedData.raptorData.stopData[i1].dist(
                             tripBasedData.raptorData.stopData[source]) <
                         tripBasedData.raptorData.stopData[i2].dist(
                             tripBasedData.raptorData.stopData[source]);
                });

      for (int r = minR; r <= maxR; ++r) {
        if (static_cast<size_t>(1 << r) >= allStopsSorted.size()) {
          std::cout << "TOOO MUCH!! r: " << r << " vs " << allStopsSorted.size()
                    << std::endl;
          break;
        }
        auto target = allStopsSorted[(1 << r)];

        int depTime = timeDistribution(randomGenerator);

        algorithm.run(static_cast<StopId>(source), depTime,
                      static_cast<StopId>(target));
        queryRunTimes.emplace_back(algorithm.getProfiler().getTotalTime());
        algorithm.getProfiler().reset();
      }
    }

    std::ofstream csv(file);
    AssertMsg(csv, "Cannot create output stream for " << file);
    AssertMsg(csv.is_open(), "Cannot open output stream for " << file);

    csv << "Index";

    for (int r = minR; r <= maxR; ++r) {
      csv << "," << r;
    }
    csv << "\n";

    size_t i = 0;

    auto it = queryRunTimes.begin();

    while (i < n) {
      csv << i;
      for (int r = minR; r <= maxR; ++r, ++it) {
        assert(it != queryRunTimes.end());
        csv << "," << (*it);
      }
      csv << "\n";
      ++i;
    }
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
