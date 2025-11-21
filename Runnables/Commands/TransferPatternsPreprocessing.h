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
    addParameter("Number of queries", "10000");
    addParameter("Query input file", "");
    addParameter("A Star enabled?", "false");
  }

  virtual void execute() noexcept {
    const bool useAStar = getParameter<bool>("A Star enabled?");
    const std::string inputFile = getParameter("Input file (TP Data)");
    const std::string queryFile = getParameter("Query input file");

    TransferPattern::Data data(inputFile);
    data.printInfo();

    const size_t n = getParameter<size_t>("Number of queries");

    std::vector<StopQuery> queries;
    if (queryFile == "") {
      queries = generateRandomStopQueries(data.raptorData.numberOfStops(), n);
    } else {
      queries = loadFromFileStopQueries(queryFile);
    }

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
                << String::prettyDouble(numJourneys / queries.size())
                << std::endl;
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
                << String::prettyDouble(numJourneys / queries.size())
                << std::endl;
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
    addParameter("Number of queries", "10000");
    addParameter("Query input file", "");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file (TP Data)");
    const std::string queryFile = getParameter("Query input file");

    TransferPattern::Data data(inputFile);
    data.printInfo();

    const size_t n = getParameter<size_t>("Number of queries");
    std::vector<StopQuery> queries;
    if (queryFile == "") {
      queries = generateRandomStopQueries(data.raptorData.numberOfStops(), n);
    } else {
      queries = loadFromFileStopQueries(queryFile);
    }

    TransferPattern::DFSQuery<TransferPattern::AggregateProfiler> algorithm(
        data);
    double numJourneys = 0;
    for (const StopQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
      numJourneys += algorithm.getNumJourneysFound();
    }

    std::cout << "#### Stats ####" << std::endl;
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. journeys                : "
              << String::prettyDouble(numJourneys / queries.size())
              << std::endl;
  }
};

class GenerateRandomSourceTargetQueries : public ParameterizedCommand {
public:
  GenerateRandomSourceTargetQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "generateRandomQueries",
                             "Given TB data, generate random stop-to-stop "
                             "queries and save to file.") {
    addParameter("Input file (TB Data)");
    addParameter("Output file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file (TB Data)");
    const std::string outputFile = getParameter("Output file");

    TripBased::Data data(inputFile);
    data.printInfo();

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(data.raptorData.numberOfStops(), n);

    std::ofstream out(outputFile);
    if (!out) {
      std::cout << "File not loadable!" << std::endl;
      return;
    }

    for (const StopQuery &query : queries) {
      out << (int)query.source << " " << (int)query.target << " "
          << (int)query.departureTime << "\n";
    }

    out.close();
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

class ComputeSelectedTPUsingTB : public ParameterizedCommand {
public:
  ComputeSelectedTPUsingTB(BasicShell &shell)
      : ParameterizedCommand(
            shell, "computeSelectedTPUsingTB",
            "Computs all Transfer Patterns using TB Profile Queries!") {
    addParameter("Input file (TripBased Data)");
    addParameter("Input file (Stop List)");
    addParameter("Output file (TP Data)");
    addParameter("Number of threads", "max");
    addParameter("Pin multiplier", "1");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file (TripBased Data)");
    const std::string stopInputFile = getParameter("Input file (Stop List)");
    const std::string outputFile = getParameter("Output file (TP Data)");
    const int numberOfThreads = getNumberOfThreads();
    const int pinMultiplier = getParameter<int>("Pin multiplier");

    if (numberOfThreads <= 0) {
      std::cout << "Positive number of threads required!" << std::endl;
      return;
    }

    std::vector<StopId> sourceStops = loadSourceStops(stopInputFile);

    TripBased::Data data(inputFile);
    data.printInfo();

    std::cout << "Computing Selected Transfer Pattern with "
              << (int)numberOfThreads << " # of threads!" << std::endl;

    TransferPattern::Data tpData(data.raptorData);

    TransferPattern::ComputeTransferPatternUsingTripBased(
        data, tpData, sourceStops, numberOfThreads, pinMultiplier);

    long long totalNumVertices(0);
    long long totalNumEdges(0);

    for (const StopId stop : sourceStops) {
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

  inline std::vector<StopId> loadSourceStops(const std::string &fileName) {
    std::vector<StopId> result;
    std::ifstream file(fileName);

    if (!file) {
      std::cerr << "Error: Could not open file " << fileName << "\n";
      return result;
    }

    int from, to, deptime;

    while (file >> from >> to >> deptime) {
      result.push_back(StopId(from));
    }

    if (!file.eof()) {
      std::cerr << "Warning: Some lines could not be parsed.\n";
    }

    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());

    return result; // RVO applies
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
