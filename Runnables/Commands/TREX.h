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

#include <algorithm>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "../../Algorithms/TREX/BorderStops.h"
#include "../../Algorithms/TREX/Preprocessing/BuilderIBEs.h"
#include "../../Algorithms/TREX/Preprocessing/TBTEGraph.h"
#include "../../Algorithms/TREX/Query/TREXProfileQuery.h"
#include "../../Algorithms/TREX/Query/TREXQuery.h"
#include "../../Algorithms/TripBased/Preprocessing/StopEventGraphBuilder.h"
#include "../../Algorithms/TripBased/Preprocessing/ULTRABuilderTransitive.h"
#include "../../Algorithms/TripBased/Query/TransitiveOneToManyQuery.h"
#include "../../Algorithms/TripBased/Query/TransitiveQuery.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/Graph/Utils/IO.h"
#include "../../DataStructures/Queries/Queries.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TREX/TREXData.h"
#include "../../DataStructures/TripBased/Data.h"
#include "../../Helpers/Console/Progress.h"
#include "../../Helpers/MultiThreading.h"
#include "../../Helpers/String/String.h"
#include "../../Shell/Shell.h"

using namespace Shell;

class ApplyPartitionFile : public ParameterizedCommand {
public:
  ApplyPartitionFile(BasicShell &shell)
      : ParameterizedCommand(
            shell, "applyPartitionFile",
            "Applies the given partition to the TREX data. Also give the "
            "number of levels and the number of cells per level!") {
    addParameter("Input file (Partition File)");
    addParameter("Input file (Number of levels)");
    addParameter("Input file (TREX Data)");
  }

  virtual void execute() noexcept {
    const std::string raptorFile = getParameter("Input file (TREX Data)");
    const int numberOfLevels =
        getParameter<int>("Input file (Number of levels)");
    const std::string partitionFile =
        getParameter("Input file (Partition File)");

    TripBased::TREXData data(raptorFile);
    data.setNumberOfLevels(numberOfLevels);
    data.printInfo();

    data.createCompactLayoutGraph();
    data.readPartitionFile(partitionFile);
    data.serialize(raptorFile);
  }
};

class RAPTORToTREX : public ParameterizedCommand {
public:
  RAPTORToTREX(BasicShell &shell)
      : ParameterizedCommand(shell, "raptorToTREX",
                             "Reads RAPTOR Data, Number of Levels and Number "
                             "of Cells per Level and saves it to a TREX Data") {
    addParameter("Input file (RAPTOR Data)");
    addParameter("Output file (TREX Data)");
    addParameter("Number of levels");
    addParameter("Route-based pruning?", "true");
    addParameter("Number of threads", "max");
    addParameter("Pin multiplier", "1");
  }

  virtual void execute() noexcept {
    const std::string raptorFile = getParameter("Input file (RAPTOR Data)");
    const std::string mltbFile = getParameter("Output file (TREX Data)");
    const int numLevels = getParameter<int>("Number of levels");
    const bool routeBasedPruning = getParameter<bool>("Route-based pruning?");
    const int numberOfThreads = getNumberOfThreads();
    const int pinMultiplier = getParameter<int>("Pin multiplier");

    RAPTOR::Data raptor(raptorFile);
    /* raptor.normalizeInstantaneousTravel(); */

    TripBased::TREXData data(raptor, numLevels);

    if (numberOfThreads == 0) {
      if (routeBasedPruning) {
        TripBased::ComputeStopEventGraphRouteBased(data);
      } else {
        TripBased::ComputeStopEventGraph(data);
      }
    } else {
      if (routeBasedPruning) {
        TripBased::ComputeStopEventGraphRouteBased(data, numberOfThreads,
                                                   pinMultiplier);
      } else {
        TripBased::ComputeStopEventGraph(data, numberOfThreads, pinMultiplier);
      }
    }

    data.resetInformationToStopEventGraph();
    /* data.convertStopEventGraphToDynamicEventGraph(); */
    data.printInfo();
    data.serialize(mltbFile);
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

class BuildTBTEGraph : public ParameterizedCommand {
public:
  BuildTBTEGraph(BasicShell &shell)
      : ParameterizedCommand(shell, "buildTBTEGraph",
                             "Given the TREX data, builds the TBTE Graph.") {
    addParameter("Input file (TREX Data)");
  }

  virtual void execute() noexcept {
    const std::string mltbFile = getParameter("Input file (TREX Data)");
    TripBased::TREXData data(mltbFile);
    data.printInfo();

    TripBased::TBTEGraph tbte(data);
    tbte.buildTBTEGraph();
  };
};

class CreateCompactLayoutGraph : public ParameterizedCommand {
public:
  CreateCompactLayoutGraph(BasicShell &shell)
      : ParameterizedCommand(
            shell, "createCompactLayoutGraph",
            "Creates the compact layout graph of the given TREX data, it "
            "writes the Compact Layout Graph into METIS Format. If wanted, "
            "write a GRAPHML file.") {
    addParameter("Input file (TREX Data)");
    addParameter("Output file (METIS File)");
    addParameter("Write Dimacs?", "false");
    addParameter("Write GRAPHML?", "false");
  }

  virtual void execute() noexcept {
    const std::string mltbFile = getParameter("Input file (TREX Data)");
    const std::string metisFile = getParameter("Output file (METIS File)");
    const bool writeDimacs = getParameter<bool>("Write Dimacs?");
    const bool writeGRAPHML = getParameter<bool>("Write GRAPHML?");

    TripBased::TREXData data(mltbFile);
    data.printInfo();

    data.createCompactLayoutGraph();
    data.writeLayoutGraphToMETIS(metisFile, writeGRAPHML);

    if (writeDimacs) {
      Graph::toDimacs(metisFile, data.layoutGraph,
                      data.layoutGraph.getEdgeAttributes().get(Weight));
    }

    data.serialize(mltbFile);
  }
};

class Customization : public ParameterizedCommand {
public:
  Customization(BasicShell &shell)
      : ParameterizedCommand(shell, "customize",
                             "Computes the customization of TREX") {
    addParameter("Input file (TREX Data)");
    addParameter("Output file (TREX Data)");
    addParameter("Number of threads", "max");
    addParameter("Pin multiplier", "1");
  }

  virtual void execute() noexcept {
    const std::string mltbFile = getParameter("Input file (TREX Data)");
    const std::string output = getParameter("Output file (TREX Data)");
    const int numberOfThreads = getNumberOfThreads();
    const int pinMultiplier = getParameter<int>("Pin multiplier");

    TripBased::TREXData data(mltbFile);
    // reset
    data.resetInformationToStopEventGraph();
    data.printInfo();

    TripBased::Builder bobTheBuilder(data, numberOfThreads, pinMultiplier);

    bobTheBuilder.run();

    std::cout << "******* Stats *******\n";
    bobTheBuilder.getProfiler().printStatistics();
    data.serialize(output);
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

class PartialCustomization : public ParameterizedCommand {
public:
  PartialCustomization(BasicShell &shell)
      : ParameterizedCommand(shell, "partialCustomize",
                             "Computes a partial customization of TREX") {
    addParameter("Input file (TREX Data)");
    addParameter("Output file (TREX Data)");
    addParameter("Number of threads", "max");
    addParameter("Pin multiplier", "1");
  }

  virtual void execute() noexcept {
    const std::string mltbFile = getParameter("Input file (TREX Data)");
    const std::string output = getParameter("Output file (TREX Data)");
    const int numberOfThreads = getNumberOfThreads();
    const int pinMultiplier = getParameter<int>("Pin multiplier");

    TripBased::TREXData data(mltbFile);
    // reset
    data.resetInformationToStopEventGraph();
    data.printInfo();

    TripBased::Builder bobTheBuilder(data, numberOfThreads, pinMultiplier);

    std::unordered_set<uint16_t> selectedCells;
    selectedCells.insert(0);
    bobTheBuilder.run(selectedCells);

    std::cout << "******* Stats *******\n";
    bobTheBuilder.getProfiler().printStatistics();
    data.serialize(output);
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

class ShowInfoOfTREX : public ParameterizedCommand {
public:
  ShowInfoOfTREX(BasicShell &shell)
      : ParameterizedCommand(shell, "showInfoOfTREX",
                             "Shows Information about the given TREX file.") {
    addParameter("Input file (TREX Data)");
    addParameter("Write to csv?", "false");
    addParameter("Output file (csv)", "false");
  }

  virtual void execute() noexcept {
    const std::string tripFile = getParameter("Input file (TREX Data)");
    const bool writeToCSV = getParameter<bool>("Write to csv?");
    const std::string fileName = getParameter("Output file (csv)");
    TripBased::TREXData data(tripFile);
    data.printInfo();

    std::vector<size_t> numLocalTransfers(data.getNumberOfLevels() + 1, 0);
    std::vector<size_t> numHopsPerLevel(data.getNumberOfLevels() + 1, 0);

    for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
      ++numLocalTransfers[data.stopEventGraph.get(LocalLevel, edge)];
      /* numHopsPerLevel[data.stopEventGraph.get(LocalLevel, edge)] +=
       * data.stopEventGraph.get(Hop, edge); */
    }

    std::cout << "** Number of Local Transfers **" << std::endl;

    for (size_t level(0); level < numLocalTransfers.size(); ++level) {
      std::cout << "Level " << level << ":       "
                << String::prettyInt(numLocalTransfers[level]) << "    "
                << String::prettyDouble((100.0 * numLocalTransfers[level] /
                                         data.stopEventGraph.numEdges()))
                << " %" << std::endl;
    }

    /*         std::cout << "** Avg # of hops per Level **" << std::endl; */

    /*         for (size_t level(0); level < numLocalTransfers.size(); ++level)
     * { */
    /*             std::cout << "Level " << level << ":      " */
    /*                       << String::prettyDouble(numHopsPerLevel[level] /
     * (double)numLocalTransfers[level]) */
    /*                       << std::endl; */
    /*         } */

    if (writeToCSV)
      data.writeLocalLevelOfTripsToCSV(fileName);
  }
};

class RunTREXQuery : public ParameterizedCommand {
public:
  RunTREXQuery(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runTREXQueries",
            "Runs the given number of random MultiLevel TB queries.") {
    addParameter("Input file (TREX Data)");
    addParameter("Number of queries");
    /* addParameter("Compare to TB?"); */
    /* addParameter("TB Input for eval"); */
  }

  virtual void execute() noexcept {
    const std::string tripFile = getParameter("Input file (TREX Data)");
    /* const bool eval = getParameter<bool>("Compare to TB?"); */
    /* const std::string evalFile = getParameter("TB Input for eval"); */

    TripBased::TREXData data(tripFile);
    data.printInfo();
    TripBased::TREXQuery<TripBased::AggregateProfiler> algorithm(data);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(data.numberOfStops(), n);

    /* std::vector<std::vector<std::pair<int, int>>> result; */
    /* result.assign(n, {}); */

    size_t numberOfJourneys = 0;

    /* size_t i(0); */
    for (const StopQuery &query : queries) {
      algorithm.run(query.source, query.departureTime, query.target);
      numberOfJourneys += algorithm.getJourneys().size();
      /* result[i].reserve(algorithm.getArrivals().size()); */

      /*             std::cout << "TREX Query" << std::endl; */
      /*             for (auto& journey : algorithm.getJourneys()) { */
      /*                 std::cout << query << std::endl; */
      /*                 for (auto& leg : journey) { */
      /*                     std::cout << (int)leg.from << " -> " << (int)leg.to
       * << " @ " << leg.departureTime << " -> " << leg.arrivalTime <<
       * (leg.usesRoute ? ", route: " : ", transfer: ") << (int)leg.routeId; */
      /*                     if (!leg.usesRoute && Edge(leg.routeId) != noEdge)
       * { */
      /*                         uint8_t lcl = std::min( */
      /*                             data.getLowestCommonLevel(StopId(leg.from),
       * query.source), */
      /*                             data.getLowestCommonLevel(StopId(leg.from),
       * query.target)); */
      /*                         std::cout << " LocalLevel: " <<
       * (int)data.stopEventGraph.get(LocalLevel, Edge(leg.routeId)) << " and
       * lcl: " << (int)lcl; */
      /*                     } */

      /*                     std::cout << std::endl; */
      /*                 } */
      /*                 std::cout << std::endl; */
      /*             } */

      /* for (auto &arr : algorithm.getArrivals()) { */
      /*   result[i].push_back(std::make_pair(arr.numberOfTrips,
       * arr.arrivalTime)); */
      /* } */

      /* i += 1; */
    }
    algorithm.getProfiler().printStatistics();
    std::cout << "Avg. Journeys: "
              << String::prettyDouble(numberOfJourneys / (float)queries.size())
              << std::endl;
    /* algorithm.showTransferLevels(); */

    /* if (eval) { */
    /*   size_t wrongQueries = 0; */
    /*   std::cout << "Evaluation against TB:" << std::endl; */
    /*   TripBased::Data trip(evalFile); */
    /*   trip.printInfo(); */
    /*   TripBased::TransitiveQuery<TripBased::AggregateProfiler> tripAlgorithm(
     */
    /*       trip); */
    /*   std::vector<std::vector<std::pair<int, int>>> tripResult; */
    /*   tripResult.assign(n, {}); */

    /*   numberOfJourneys = 0; */
    /*   i = 0; */
    /*   for (const StopQuery &query : queries) { */
    /*     tripAlgorithm.run(query.source, query.departureTime, query.target);
     */
    /*     numberOfJourneys += tripAlgorithm.getJourneys().size(); */

    /*     std::cout << "TB Query" << std::endl; */
    /*     for (auto &journey : tripAlgorithm.getJourneys()) { */
    /*       std::cout << query << std::endl; */
    /*       for (auto &leg : journey) { */
    /*         std::cout << (int)leg.from << " -> " << (int)leg.to << " @ " */
    /*                   << leg.departureTime << " -> " << leg.arrivalTime */
    /*                   << (leg.usesRoute ? ", route: " : ", transfer: ") */
    /*                   << (int)leg.routeId; */
    /*         if (!leg.usesRoute && Edge(leg.routeId) != noEdge) { */
    /*           uint8_t lcl = std::min( */
    /*               data.getLowestCommonLevel(StopId(leg.from), query.source),
     */
    /*               data.getLowestCommonLevel(StopId(leg.from), query.target));
     */
    /*           std::cout << " LocalLevel: " */
    /*                     << (int)data.stopEventGraph.get(LocalLevel, */
    /*                                                     Edge(leg.routeId)) */
    /*                     << " and lcl : " << (int)lcl; */
    /*         } */

    /*         std::cout << std::endl; */
    /*       } */
    /*       std::cout << std::endl; */
    /*     } */

    /*     tripResult[i].reserve(tripAlgorithm.getArrivals().size()); */

    /*     for (auto &arr : tripAlgorithm.getArrivals()) { */
    /*       tripResult[i].push_back( */
    /*           std::make_pair(arr.numberOfTrips, arr.arrivalTime)); */
    /*     } */

    /*     i += 1; */
    /*   } */
    /*   tripAlgorithm.getProfiler().printStatistics(); */
    /*   std::cout << "Avg. Journeys: " */
    /*             << String::prettyDouble(numberOfJourneys / */
    /*                                     (float)queries.size()) */
    /*             << std::endl; */

    /*   for (size_t i(0); i < queries.size(); ++i) { */
    /*     // computes the results from TB, which are not in TREX */
    /*     std::set<std::pair<int, int>> set1(tripResult[i].begin(), */
    /*                                        tripResult[i].end()); */
    /*     std::set<std::pair<int, int>> set2(result[i].begin(),
     * result[i].end()); */
    /*     std::set<std::pair<int, int>> difference; */
    /*     std::set_difference(set1.begin(), set1.end(), set2.begin(),
     * set2.end(), */
    /*                         std::inserter(difference, difference.begin()));
     */

    /*     if (difference.size() > 0) { */
    /*       ++wrongQueries; */
    /*       std::cout << "Query: " << queries[i] << std::endl; */
    /*     } */
    /*   } */

    /*   std::cout << "Wrong queries: " << wrongQueries << std::endl; */
    /* } */
  }
};

class RunTREXProfileQueries : public ParameterizedCommand {
public:
  RunTREXProfileQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "runTREXProfileQueries",
                             "Runs the given number of random transitive "
                             "TripBased queries with "
                             "a time range of [0, 24 hours).") {
    addParameter("TREX input file");
    addParameter("Number of queries");
  }

  virtual void execute() noexcept {
    TripBased::TREXData data(getParameter("TREX input file"));
    data.printInfo();
    TripBased::TREXProfileQuery<TripBased::AggregateProfiler> algorithm(data);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<StopQuery> queries =
        generateRandomStopQueries(data.numberOfStops(), n);

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

class WriteTREXToCSV : public ParameterizedCommand {
public:
  WriteTREXToCSV(BasicShell &shell)
      : ParameterizedCommand(shell, "writeTREXToCSV",
                             "Writes TREX Data to csv files") {
    addParameter("Input file (TREX Data)");
    addParameter("Output file (CSV files)");
  }

  virtual void execute() noexcept {
    const std::string mltb = getParameter("Input file (TREX Data)");
    const std::string output = getParameter("Output file (CSV files)");

    TripBased::TREXData data(mltb);
    data.printInfo();

    data.raptorData.writeCSV(output);
    data.writePartitionToCSV(output + "partition.csv");
    data.writeUnionFindToFile(output + "unionFind.csv");

    Graph::toEdgeListCSV(output + "transfer", data.stopEventGraph);
  }
};

class EventDistributionOverTime : public ParameterizedCommand {
public:
  EventDistributionOverTime(BasicShell &shell)
      : ParameterizedCommand(shell, "eventDistribution",
                             "Shows the distribution of events over time. "
                             "Each bucket contains "
                             "the number of events in this bucket.") {
    addParameter("Input file (TREX Data)");
  }

  virtual void execute() noexcept {
    const std::string mltb = getParameter("Input file (TREX Data)");
    const int numBuckets = 24;

    TripBased::TREXData data(mltb);
    data.printInfo();

    std::vector<size_t> buckets(numBuckets, 0);

    size_t offset = 60 * 60;

    for (size_t eventId(0); eventId < data.numberOfStopEvents(); ++eventId) {
      auto &depTime = data.departureTime(StopEventId(eventId));

      if ((24 * 60 * 60) <= depTime)
        continue;

      // this is due to our implicit representation
      if (depTime < 0) {
        depTime = 0;
      }
      buckets[(int)depTime / offset]++;
    }

    for (int i(0); i < numBuckets; ++i) {
      std::cout << i << "," << buckets[i] << std::endl;
    }
  }
};

class RunGeoRankedTREXQueries : public ParameterizedCommand {
public:
  RunGeoRankedTREXQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "runGeoRankedTREXQueries",
                             "Runs TREX queries to the 2^r th stop, where "
                             "r is the geo rank. "
                             "Source stops are chosen randomly.") {
    addParameter("TREX input file");
    addParameter("Number of source stops");
    addParameter("Output csv file");
    addParameter("Lowest r");
  }

  virtual void execute() noexcept {
    const std::string file = getParameter("Output csv file");
    TripBased::TREXData data(getParameter("TREX input file"));
    data.printInfo();
    TripBased::TREXQuery<TripBased::AggregateProfiler> algorithm(data);

    const size_t n = getParameter<size_t>("Number of source stops");
    const int minR = getParameter<int>("Lowest r");

    std::mt19937 randomGenerator(42);
    std::uniform_int_distribution<> stopDistribution(0,
                                                     data.numberOfStops() - 1);
    std::uniform_int_distribution<> timeDistribution(0, (24 * 60 * 60) - 1);

    std::vector<StopId> sources;
    sources.reserve(n);

    for (size_t i = 0; i < n; i++) {
      sources.emplace_back(stopDistribution(randomGenerator));
    }

    int maxR = std::floor(std::log2(data.numberOfStops()));

    if (maxR <= minR) {
      std::cout << "Too few stops; maxR <= minR!" << std::endl;
      return;
    }

    std::vector<double> queryRunTimes;
    queryRunTimes.reserve(n * (maxR - minR + 1));

    for (auto &source : sources) {
      std::vector<size_t> allStopsSorted(data.numberOfStops());
      std::iota(allStopsSorted.begin(), allStopsSorted.end(), 0);

      std::sort(allStopsSorted.begin(), allStopsSorted.end(),
                [&](int i1, int i2) {
                  return data.raptorData.stopData[i1].dist(
                             data.raptorData.stopData[source]) <
                         data.raptorData.stopData[i2].dist(
                             data.raptorData.stopData[source]);
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

class CheckBorderStops : public ParameterizedCommand {
public:
  CheckBorderStops(BasicShell &shell)
      : ParameterizedCommand(shell, "checkBorderStops",
                             "Check stop-to-stop (only border stops) and see "
                             "which transfers are used.") {
    addParameter("Input file (TREX Data)");
    addParameter("Output file (csv)", "transfers.csv");
    addParameter("Number of threads", "max");
  }

  virtual void execute() noexcept {
    const std::string mltb = getParameter("Input file (TREX Data)");
    const std::string file = getParameter("Output file (csv)");
    const int numberOfThreads = getNumberOfThreads();

    TripBased::TREXData data(mltb);
    data.printInfo();

    std::vector<std::uint8_t> rankEstimate(data.stopEventGraph.numEdges(), 0);

    const int numLevels = data.numberOfLevels;
    TripBased::BorderStops checker(data);

    std::unordered_set<std::uint32_t> stopSet;
    std::vector<StopId> targets;
    targets.reserve(data.numberOfStops());

    omp_set_num_threads(numberOfThreads);

    std::vector<TripBased::TransitiveOneToManyQuery<>> query(
        numberOfThreads, TripBased::TransitiveOneToManyQuery<>(data));

    for (int level = 0; level < numLevels; ++level) {
      std::cout << "*** Level " << (numLevels - level) << " ***" << std::endl;
      for (int cell = 0; cell < (1 << (numLevels - level)); ++cell) {
        auto inAndOutTrips =
            checker.collectIncommingAndOutgoingTrips(level, cell);

        std::sort(inAndOutTrips.first.begin(), inAndOutTrips.first.end(),
                  [&](const auto &left, const auto &right) {
                    return std::tie(data.departureTime(data.getStopEventId(
                                        left.first, left.second)),
                                    left.first, left.second) <
                           std::tie(data.departureTime(data.getStopEventId(
                                        right.first, right.second)),
                                    right.first, right.second);
                  });

        std::cout << "\nCell: " << cell << ", " << inAndOutTrips.first.size()
                  << std::endl;

        stopSet.clear();
        targets.clear();

        for (auto [t, stopIndex] : inAndOutTrips.second) {
          for (StopIndex i = stopIndex; i < data.numberOfStopsInTrip(t); ++i) {
            StopId stop = data.getStop(t, i);
            stopSet.insert(static_cast<std::uint32_t>(stop));
          }
        }

        for (auto t : stopSet) {
          targets.push_back(static_cast<StopId>(t));
        }

        /* auto isTransferInCell = [&](const std::uint16_t stopCell) -> bool
         * {
         */
        /*   return cell == (stopCell >> level); */
        /* }; */

        Progress progress(inAndOutTrips.first.size());
#pragma omp parallel for
        for (std::size_t j = 0; j < inAndOutTrips.first.size(); ++j) {
          int tId = omp_get_thread_num();
          auto [t, stopIndex] = inAndOutTrips.first[j];
          for (StopIndex i = StopIndex(0); i < stopIndex; ++i) {
            StopId source = data.getStop(t, i);
            int depTime = data.departureTime(data.getStopEventId(t, i));

            query[tId].run(source, depTime, targets, cell, level);
            for (auto t : targets) {
              auto journeys = query[tId].getJourneys(t);

              for (auto &j : journeys) {
                for (auto &leg : j) {
                  if (!leg.usesRoute && leg.transferId != noEdge) {
                    AssertMsg(leg.transferId < rankEstimate.size(),
                              "Transfer Id is out of range!");
                    /* StopId toStop = data.getStopOfStopEvent(StopEventId(
                     */
                    /*     data.stopEventGraph.get(ToVertex,
                     * leg.transferId)));
                     */

                    /* if (!isTransferInCell(data.cellIds[toStop])) */
                    /*   continue; */
                    rankEstimate[leg.transferId] = level + 1;
                  }
                }
              }
            }
          }
          ++progress;
        }
      }
    }

    std::ofstream csv(file);
    AssertMsg(csv, "Cannot create output stream for " << file);
    AssertMsg(csv.is_open(), "Cannot open output stream for " << file);

    csv << "FromVertex,ToVertex,RankEstimator\n";
    for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
      csv << (int)from << "," << (int)data.stopEventGraph.get(ToVertex, edge)
          << "," << (int)rankEstimate[edge] << "\n";
    }
    csv.close();
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

class ExportTREXTimeExpandedGraph : public ParameterizedCommand {
public:
  ExportTREXTimeExpandedGraph(BasicShell &shell)
      : ParameterizedCommand(
            shell, "exportTREXAsTE",
            "Export TREX data into a Time Expanded-like graph") {
    addParameter("Input file (TREX Data)");
    addParameter("Output file (csv file)");
  }

  virtual void execute() noexcept {
    const std::string tb = getParameter("Input file (TREX Data)");
    const std::string outputFilename = getParameter("Output file (csv file)");

    TripBased::TREXData data(tb);
    data.printInfo();

    DynamicTripBasedTimeExpGraph graph;
    graph.addVertices(data.numberOfStopEvents());

    // Transfers with Hop: 1
    for (const auto [edge, from] : data.stopEventGraph.edgesWithFromVertex()) {
      const Vertex to = data.stopEventGraph.get(ToVertex, edge);
      graph.addEdge(from, to).set(Hop, 1);
    }

    // Trip Edges with Hop: 0
    for (std::size_t i = 0; i < data.numberOfTrips(); ++i) {
      const Vertex firstEvent =
          static_cast<Vertex>(data.firstStopEventOfTrip[i]);
      const Vertex lastEvent =
          static_cast<Vertex>(data.firstStopEventOfTrip[i + 1] - 1);

      for (Vertex v = firstEvent; v < lastEvent; ++v) {
        graph.addEdge(v, Vertex(v + 1)).set(Hop, 0);
      }
    }

    Graph::printInfo(graph);
    std::string csvGraph = outputFilename + ".graph";
    std::string csvStops = outputFilename + ".stops";
    Graph::toEdgeListCSV(csvGraph, graph);

    std::ofstream csv(csvStops);
    AssertMsg(csv, "Cannot create output stream for " << csvStops);
    AssertMsg(csv.is_open(), "Cannot open output stream for " << csvStops);

    csv << "Vertex,StopId,CellId\n";
    for (StopEventId v(0); v < data.numberOfStopEvents(); ++v) {
      StopId stop = data.getStopOfStopEvent(v);
      csv << (int)v << "," << (int)stop << "," << data.getCellIdOfStop(stop)
          << "\n";
    }

    csv.close();
  }
};

class ShowInducedCellOfNetwork : public ParameterizedCommand {
public:
  ShowInducedCellOfNetwork(BasicShell &shell)
      : ParameterizedCommand(
            shell, "showInducedCellOfNetwork",
            "Show stops, trips, lines inside the given cell.") {
    addParameter("Input file (TREX Data)");
    addParameter("CellId", "0");
    addParameter("Level", "0");
  }

  virtual void execute() noexcept {
    const std::string tb = getParameter("Input file (TREX Data)");
    const int cellId = getParameter<int>("CellId");
    const int level = getParameter<int>("Level");

    TripBased::TREXData data(tb);
    data.printInfo();

    std::unordered_set<std::uint32_t> collectedRoutes;
    std::unordered_set<std::uint32_t> collectedStops;
    std::vector<StopId> stopsInCell;

    auto isInCell = [&](const StopId stop, const int level,
                        const int cell) -> bool {
      assert(level >= 0 && level < 16);
      assert(cell == 0 || cell == 1);
      return ((data.getCellIdOfStop(stop) >> level) & 1) ==
             static_cast<std::uint16_t>(cell);
    };

    for (StopId stop(0); stop < data.numberOfStops(); ++stop) {
      if (isInCell(stop, level, cellId)) {
        stopsInCell.push_back(stop);
        for (const RAPTOR::RouteSegment &route :
             data.routesContainingStop(stop)) {
          collectedRoutes.insert(static_cast<std::uint32_t>(route.routeId));
        }
      }
    }

    std::cout << "Stops inside cell:\n";
    for (auto sId : stopsInCell) {
      std::cout << (int)sId << std::endl;
    }

    std::cout << "Stops on crossing routes:\n";
    std::cout << "RouteId,StopId\n";
    for (auto rId : collectedRoutes) {
      for (auto sId : data.raptorData.stopsOfRoute(static_cast<RouteId>(rId))) {
        std::cout << (int)rId << "," << (int)sId << std::endl;
      }
    }
  }
};

class StopsImportance : public ParameterizedCommand {
public:
  StopsImportance(BasicShell &shell)
      : ParameterizedCommand(shell, "stopsImportance",
                             "Export the importance of each stop into a csv.") {
    addParameter("Input file (TREX Data)");
    addParameter("Output csv file");
  }

  virtual void execute() noexcept {
    const std::string mltbFile = getParameter("Input file (TREX Data)");
    const std::string file = getParameter("Output csv file");

    TripBased::TREXData mltb(mltbFile);

    std::vector<int> rankOfStop(mltb.numberOfStops(), 0);

    for (const auto [edge, from] : mltb.stopEventGraph.edgesWithFromVertex()) {
      StopId fromS(mltb.getStopOfStopEvent(StopEventId(from)));
      StopId toS(mltb.getStopOfStopEvent(
          StopEventId(mltb.stopEventGraph.get(ToVertex, edge))));

      rankOfStop[fromS] = std::max(
          rankOfStop[fromS], (int)mltb.stopEventGraph.get(LocalLevel, edge));
      rankOfStop[toS] = std::max(
          rankOfStop[toS], (int)mltb.stopEventGraph.get(LocalLevel, edge));
    }

    std::ofstream csv(file);
    AssertMsg(csv, "Cannot create output stream for " << file);
    AssertMsg(csv.is_open(), "Cannot open output stream for " << file);

    csv << "StopId,Rank,Lat,Lon\n";
    for (StopId stop : mltb.stops()) {
      csv << (int)stop << "," << rankOfStop[stop] << ","
          << mltb.raptorData.stopData[stop].coordinates.latitude << ","
          << mltb.raptorData.stopData[stop].coordinates.longitude << "\n";
    }
    csv.close();
  }
};

class WriteTREXStopFailureDistribution : public ParameterizedCommand {
public:
  WriteTREXStopFailureDistribution(BasicShell &shell)
      : ParameterizedCommand(
            shell, "writeTREXStopFailureDistribution",
            "Writes some metrics per stop of the given TREX data "
            "to a csv file.") {
    addParameter("TREX Binary");
    addParameter("Output file");
  }

  virtual void execute() noexcept {
    const std::string networkFile = getParameter("TREX Binary");
    const std::string outputFile = getParameter("Output file");

    TripBased::TREXData data(networkFile);
    data.exportStopFailureDistribution(outputFile);
  }
};
