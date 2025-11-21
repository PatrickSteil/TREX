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
#include <string>

#include "../../Algorithms/TransferPattern/Preprocessing/TransferPatternBuilder.h"
#include "../../Algorithms/TransferPattern/Query/DFSQuery.h"
#include "../../Algorithms/TransferPattern/Query/Query.h"
#include "../../Algorithms/TransferPattern/Query/QueryAStar.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/Queries/Queries.h"
#include "../../DataStructures/TransferPattern/Data.h"
#include "../../DataStructures/TripBased/Data.h"
#include "../../Helpers/MultiThreading.h"
#include "../../Helpers/String/String.h"
#include "../../Shell/Shell.h"

using namespace Shell;

class RunTransferPatternQueries : public ParameterizedCommand {
 public:
  RunTransferPatternQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runTPQueries",
            "Runs the given number of random Transfer Pattern Queries.") {
    addParameter("Input file (TP Data)");
    addParameter("Number of queries");
    addParameter("A Star enabled?", "false");
  }

  virtual void execute() noexcept {
    const bool useAStar = getParameter<bool>("A Star enabled?");
    const std::string inputFile = getParameter("Input file (TP Data)");

    TransferPattern::Data data(inputFile);
    data.printInfo();

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(data.raptorData.numberOfStops(), n);

    if (!useAStar) {
      TransferPattern::Query<TransferPattern::AggregateProfiler> algorithm(
          data);
      double numJourneys = 0;
      for (const StopQuery &query : queries) {
        algorithm.run(query.source, query.departureTime, query.target);
        numJourneys += algorithm.getJourneys().size();
      }

      std::cout << "#### Stats ####" << std::endl;
      algorithm.getProfiler().printStatistics();
      std::cout << "Avg. journeys                : "
                << String::prettyDouble(numJourneys / n) << std::endl;
    } else {
      TransferPattern::QueryAStar<TransferPattern::AggregateProfiler> algorithm(
          data);
      double numJourneys = 0;
      for (const StopQuery &query : queries) {
        algorithm.run(query.source, query.departureTime, query.target);
        numJourneys += algorithm.getJourneys().size();
      }

      std::cout << "#### Stats ####" << std::endl;
      algorithm.getProfiler().printStatistics();
      std::cout << "Avg. journeys                : "
                << String::prettyDouble(numJourneys / n) << std::endl;
    }
  }
};

class RunDFSTransferPatternQueries : public ParameterizedCommand {
 public:
  RunDFSTransferPatternQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "runDFSTPQueries",
                             "Runs the given number of random DFS-based "
                             "Transfer Pattern Queries.") {
    addParameter("Input file (TP Data)");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file (TP Data)");

    TransferPattern::Data data(inputFile);
    data.printInfo();

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(data.raptorData.numberOfStops(), n);

    TransferPattern::DFSQuery<TransferPattern::AggregateProfiler> algorithm(
        data);
    double numJourneys = 0;
    for (const StopQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
      numJourneys += algorithm.getNumJourneysFound();
      /* numJourneys += algorithm.getJourneys().size(); */
    }

    std::cout << "#### Stats ####" << std::endl;
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys                : "
              << String::prettyDouble(numJourneys / n) << std::endl;
  }
};

class ComputeTPUsingTB : public ParameterizedCommand {
 public:
  ComputeTPUsingTB(BasicShell &shell)
      : ParameterizedCommand(
            shell, "computeTPUsingTB",
            "Computs all Transfer Patterns using TB Profile Queries!") {
    addParameter("Input file (TripBased Data)");
    addParameter("Output file (TP Data)");
    addParameter("Number of threads", "max");
    addParameter("Pin multiplier", "1");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file (TripBased Data)");
    const std::string outputFile = getParameter("Output file (TP Data)");
    const int numberOfThreads = getNumberOfThreads();
    const int pinMultiplier = getParameter<int>("Pin multiplier");

    TripBased::Data data(inputFile);
    data.printInfo();

    TransferPattern::Data tpData(data.raptorData);

    std::cout << "Computing Transfer Pattern with " << (int)numberOfThreads
              << " # of threads!" << std::endl;

    if (numberOfThreads == 0) {
      TransferPattern::ComputeTransferPatternUsingTripBased(data, tpData);
    } else {
      TransferPattern::ComputeTransferPatternUsingTripBased(
          data, tpData, numberOfThreads, pinMultiplier);
    }

    long long totalNumVertices(0);
    long long totalNumEdges(0);

    for (const StopId stop : tpData.raptorData.stops()) {
      totalNumVertices += tpData.transferPatternOfStop[stop].numVertices();
      totalNumEdges += tpData.transferPatternOfStop[stop].numEdges();
    }

    std::cout << "Total Size:       "
              << String::bytesToString(tpData.byteSize()) << std::endl;
    std::cout << "Average # Nodes:  "
              << String::prettyDouble(totalNumVertices /
                                      data.raptorData.numberOfStops())
              << std::endl;
    std::cout << "Average # Edges:  "
              << String::prettyDouble(totalNumEdges /
                                      data.raptorData.numberOfStops())
              << std::endl;

    tpData.serialize(outputFile);
  }

 private:
  inline int getNumberOfThreads() const noexcept {
    if (getParameter("Number of threads") == "max") {
      return numberOfCores();
    } else {
      return getParameter<int>("Number of threads");
    }
  }
};

class ExportTPDAGOfStop : public ParameterizedCommand {
 public:
  ExportTPDAGOfStop(BasicShell &shell)
      : ParameterizedCommand(shell, "exportTPDAGofStop",
                             "Exports the computed Transfer Patterns of the "
                             "given stop in the given type file.") {
    addParameter("Input file (TP Data)");
    addParameter("Output file");
    addParameter("StopId");
    addParameter("File Type (GML | CSV)");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file (TP Data)");
    const std::string outputFile = getParameter("Output file");
    const std::string type = getParameter("File Type (GML | CSV)");

    TransferPattern::Data data(inputFile);
    data.printInfo();

    const size_t stop = getParameter<size_t>("StopId");

    if (!(stop < data.raptorData.numberOfStops())) {
      std::cout << "Stop Out of range!" << std::endl;
      return;
    }
    if (type == "GML") {
      Graph::toGML(outputFile, data.transferPatternOfStop[stop]);
    } else {
      Graph::toEdgeListCSV(outputFile, data.transferPatternOfStop[stop]);
    }
  }
};
