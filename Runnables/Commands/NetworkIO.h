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

#include <string>

#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/GTFS/Data.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/Intermediate/Data.h"
#include "../../DataStructures/PTL/Data.h"
#include "../../DataStructures/PPTL/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/MultimodalData.h"
#include "../../DataStructures/TD/Data.h"
#include "../../DataStructures/TE/Data.h"
#include "../../DataStructures/TripBased/MultimodalData.h"
#include "../../Shell/Shell.h"

using namespace Shell;

class ParseGTFS : public ParameterizedCommand {
public:
  ParseGTFS(BasicShell &shell)
      : ParameterizedCommand(shell, "parseGTFS",
                             "Parses raw GTFS data from the given directory "
                             "and converts it to a binary representation.") {
    addParameter("Input directory");
    addParameter("Output file");
  }

  virtual void execute() noexcept {
    const std::string gtfsDirectory = getParameter("Input directory");
    const std::string outputFile = getParameter("Output file");

    GTFS::Data data = GTFS::Data::FromGTFS(gtfsDirectory);
    data.printInfo();
    data.serialize(outputFile);
  }
};

class GTFSToIntermediate : public ParameterizedCommand {
public:
  GTFSToIntermediate(BasicShell &shell)
      : ParameterizedCommand(
            shell, "gtfsToIntermediate",
            "Converts binary GTFS data to the intermediate network format.") {
    addParameter("Input directory");
    addParameter("First day");
    addParameter("Last day");
    addParameter("Use days of operation?");
    addParameter("Use frequencies?");
    addParameter("Output file");
  }

  virtual void execute() noexcept {
    const std::string gtfsDirectory = getParameter("Input directory");
    const std::string outputFile = getParameter("Output file");
    const int firstDay = stringToDay(getParameter("First day"));
    const int lastDay = stringToDay(getParameter("Last day"));
    const bool useDaysOfOperation =
        getParameter<bool>("Use days of operation?");
    const bool useFrequencies = getParameter<bool>("Use frequencies?");

    GTFS::Data gtfs = GTFS::Data::FromBinary(gtfsDirectory);
    gtfs.printInfo();
    Intermediate::Data inter = Intermediate::Data::FromGTFS(
        gtfs, firstDay, lastDay, !useDaysOfOperation, !useFrequencies);
    inter.printInfo();
    inter.serialize(outputFile);
  }
};

class IntermediateToCSA : public ParameterizedCommand {
public:
  IntermediateToCSA(BasicShell &shell)
      : ParameterizedCommand(
            shell, "intermediateToCSA",
            "Converts binary intermediate data to CSA network format.") {
    addParameter("Input file");
    addParameter("Output file");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file");
    const std::string outputFile = getParameter("Output file");

    Intermediate::Data inter = Intermediate::Data::FromBinary(inputFile);
    inter.printInfo();
    CSA::Data data = CSA::Data::FromIntermediate(inter);
    data.printInfo();
    data.serialize(outputFile);
  }
};

class IntermediateToRAPTOR : public ParameterizedCommand {
public:
  IntermediateToRAPTOR(BasicShell &shell)
      : ParameterizedCommand(
            shell, "intermediateToRAPTOR",
            "Converts binary intermediate data to RAPTOR network format.") {
    addParameter("Input file");
    addParameter("Output file");
    addParameter("Route type", "FIFO",
                 {"Geographic", "FIFO", "Opt-FIFO", "Offset", "Frequency"});
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file");
    const std::string outputFile = getParameter("Output file");
    const std::string routeTypeString = getParameter("Route type");
    int routeType;
    if (routeTypeString == "Geographic") {
      routeType = 0;
    } else if (routeTypeString == "FIFO") {
      routeType = 1;
    } else if (routeTypeString == "Opt-FIFO") {
      routeType = 2;
    } else if (routeTypeString == "Offset") {
      routeType = 3;
    } else {
      routeType = 4;
    }

    Intermediate::Data inter = Intermediate::Data::FromBinary(inputFile);
    inter.printInfo();
    RAPTOR::Data data = RAPTOR::Data::FromIntermediate(inter, routeType);
    data.printInfo();
    Graph::printInfo(data.transferGraph);
    data.transferGraph.printAnalysis();
    data.serialize(outputFile);
  }
};

class IntermediateToTD : public ParameterizedCommand {
public:
  IntermediateToTD(BasicShell &shell)
      : ParameterizedCommand(
            shell, "intermediateToTD",
            "Converts binary intermediate data to TD format.") {
    addParameter("Input file");
    addParameter("Output file");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file");
    const std::string outputFile = getParameter("Output file");

    Intermediate::Data inter = Intermediate::Data::FromBinary(inputFile);
    inter.printInfo();

    TD::Data data = TD::Data::FromIntermediate(inter);
    data.printInfo();
    Graph::printInfo(data.timeDependentGraph);
    data.serialize(outputFile);
  }
};

class IntermediateToTE : public ParameterizedCommand {
public:
  IntermediateToTE(BasicShell &shell)
      : ParameterizedCommand(
            shell, "intermediateToTE",
            "Converts binary intermediate data to TE format.") {
    addParameter("Input file");
    addParameter("Output file");
    addParameter("Extract Footpaths?", "true");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file");
    const std::string outputFile = getParameter("Output file");
    const bool extractFootpaths = getParameter<bool>("Extract Footpaths?");

    Intermediate::Data inter = Intermediate::Data::FromBinary(inputFile);
    inter.printInfo();

    TE::Data data = TE::Data::FromIntermediate(inter, extractFootpaths);
    data.printInfo();
    Graph::printInfo(data.timeExpandedGraph);
    data.serialize(outputFile);
  }
};

class TEToPTL : public ParameterizedCommand {
public:
  TEToPTL(BasicShell &shell)
      : ParameterizedCommand(shell, "tEToPTL",
                             "Converts TE data to PTL format.") {
    addParameter("Input file");
    addParameter("Output file");
    addParameter("Input file (labels)", "");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file");
    const std::string outputFile = getParameter("Output file");
    const std::string inputFileLabels = getParameter("Input file (labels)");

    TE::Data data = TE::Data(inputFile);
    data.printInfo();

    PTL::Data ptl(data);

    if (inputFileLabels != "") {
      ptl.readViennotLabels(inputFileLabels);
      ptl.sortLabels();
    }

    ptl.printInfo();

    ptl.serialize(outputFile);
  }
};

class TEToPPTL : public ParameterizedCommand {
public:
  TEToPPTL(BasicShell &shell)
      : ParameterizedCommand(shell, "tEToPPTL",
                             "Converts TE data to PPTL format.") {
    addParameter("Input file");
    addParameter("Output file");
    addParameter("Input file (labels)", "");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file");
    const std::string outputFile = getParameter("Output file");
    const std::string inputFileLabels = getParameter("Input file (labels)");

    TE::Data data = TE::Data(inputFile);
    data.printInfo();

    PPTL::Data pptl(data);

    if (inputFileLabels != "") {
      pptl.loadPathHub(inputFileLabels);
      pptl.sortLabels();
    }

    pptl.printInfo();
    pptl.serialize(outputFile);
  }
};

class BuildMultimodalRAPTORData : public ParameterizedCommand {
public:
  BuildMultimodalRAPTORData(BasicShell &shell)
      : ParameterizedCommand(
            shell, "buildMultimodalRAPTORData",
            "Builds multimodal RAPTOR data based on RAPTOR data.") {
    addParameter("RAPTOR data");
    addParameter("Output file");
  }

  virtual void execute() noexcept {
    const RAPTOR::Data raptorData(getParameter("RAPTOR data"));
    raptorData.printInfo();
    const RAPTOR::MultimodalData multimodalData(raptorData);
    multimodalData.printInfo();
    multimodalData.serialize(getParameter("Output file"));
  }
};

class AddModeToMultimodalRAPTORData : public ParameterizedCommand {
public:
  AddModeToMultimodalRAPTORData(BasicShell &shell)
      : ParameterizedCommand(shell, "addModeToMultimodalRAPTORData",
                             "Adds a transfer graph for the specified mode to "
                             "multimodal RAPTOR data.") {
    addParameter("Multimodal RAPTOR data");
    addParameter("Transfer graph");
    addParameter("Mode");
    addParameter("Output file");
  }

  virtual void execute() noexcept {
    RAPTOR::MultimodalData multimodalData(
        getParameter("Multimodal RAPTOR data"));
    multimodalData.printInfo();
    RAPTOR::TransferGraph graph;
    graph.readBinary(getParameter("Transfer graph"));
    const size_t mode = RAPTOR::getTransferModeFromName(getParameter("Mode"));
    multimodalData.addTransferGraph(mode, graph);
    multimodalData.printInfo();
    multimodalData.serialize(getParameter("Output file"));
  }
};

class BuildMultimodalTripBasedData : public ParameterizedCommand {
public:
  BuildMultimodalTripBasedData(BasicShell &shell)
      : ParameterizedCommand(
            shell, "buildMultimodalTripBasedData",
            "Builds multimodal Trip-Based data based on Trip-Based data.") {
    addParameter("Trip-Based data");
    addParameter("Output file");
  }

  virtual void execute() noexcept {
    const TripBased::Data tripBasedData(getParameter("Trip-Based data"));
    tripBasedData.printInfo();
    const TripBased::MultimodalData multimodalData(tripBasedData);
    multimodalData.printInfo();
    multimodalData.serialize(getParameter("Output file"));
  }
};

class AddModeToMultimodalTripBasedData : public ParameterizedCommand {
public:
  AddModeToMultimodalTripBasedData(BasicShell &shell)
      : ParameterizedCommand(shell, "addModeToMultimodalTripBasedData",
                             "Adds a transfer graph for the specified mode to "
                             "multimodal Trip-Based data.") {
    addParameter("Multimodal Trip-Based data");
    addParameter("Transfer graph");
    addParameter("Mode");
    addParameter("Output file");
  }

  virtual void execute() noexcept {
    TripBased::MultimodalData multimodalData(
        getParameter("Multimodal Trip-Based data"));
    multimodalData.printInfo();
    TransferGraph graph;
    graph.readBinary(getParameter("Transfer graph"));
    const size_t mode = RAPTOR::getTransferModeFromName(getParameter("Mode"));
    multimodalData.addTransferGraph(mode, graph);
    multimodalData.printInfo();
    multimodalData.serialize(getParameter("Output file"));
  }
};

class LoadDimacsGraph : public ParameterizedCommand {
public:
  LoadDimacsGraph(BasicShell &shell)
      : ParameterizedCommand(
            shell, "loadDimacsGraph",
            "Converts DIMACS graph data to our transfer graph format.") {
    addParameter("Input file");
    addParameter("Output file");
    addParameter("Graph type", "dynamic", {"static", "dynamic"});
    addParameter("Coordinate factor", "0.000001");
  }

  virtual void execute() noexcept {
    std::string graphType = getParameter("Graph type");
    if (graphType == "static") {
      load<TransferGraph>();
    } else {
      load<DynamicTransferGraph>();
    }
  }

private:
  template <typename GRAPH_TYPE> inline void load() const noexcept {
    DimacsGraphWithCoordinates dimacs;
    dimacs.fromDimacs<true>(getParameter("Input file"),
                            getParameter<double>("Coordinate factor"));
    Graph::printInfo(dimacs);
    dimacs.printAnalysis();
    GRAPH_TYPE graph;
    Graph::move(std::move(dimacs), graph);
    Graph::printInfo(graph);
    graph.printAnalysis();
    graph.writeBinary(getParameter("Output file"));
  }
};

class WriteIntermediateToCSV : public ParameterizedCommand {
public:
  WriteIntermediateToCSV(BasicShell &shell)
      : ParameterizedCommand(
            shell, "writeIntermediateToCSV",
            "Writes all the intermediate Data into csv files.") {
    addParameter("Intermediate Binary");
    addParameter("Output file");
  }

  virtual void execute() noexcept {
    const std::string networkFile = getParameter("Intermediate Binary");
    const std::string outputFile = getParameter("Output file");

    Intermediate::Data network = Intermediate::Data::FromBinary(networkFile);
    network.writeCSV(outputFile);
  }
};

class WriteRAPTORToCSV : public ParameterizedCommand {
public:
  WriteRAPTORToCSV(BasicShell &shell)
      : ParameterizedCommand(shell, "writeRAPTORToCSV",
                             "Writes all the RAPTOR Data into csv files.") {
    addParameter("RAPTOR Binary");
    addParameter("Output file");
  }

  virtual void execute() noexcept {
    const std::string networkFile = getParameter("RAPTOR Binary");
    const std::string outputFile = getParameter("Output file");

    RAPTOR::Data network = RAPTOR::Data::FromBinary(networkFile);
    network.dontUseImplicitDepartureBufferTimes();
    network.dontUseImplicitArrivalBufferTimes();
    network.writeCSV(outputFile);
  }
};
/*
class WriteLayoutGraphToGraphML : public ParameterizedCommand {
public:
    WriteLayoutGraphToGraphML(BasicShell& shell)
        : ParameterizedCommand(shell, "writeLayoutGraphToGraphML", "Writes the
Layout Graph into a GraphML file.")
    {
        addParameter("RAPTOR Binary");
        addParameter("Output file (Layout Graph)");
    }

    virtual void execute() noexcept
    {
        const std::string networkFile = getParameter("RAPTOR Binary");
        const std::string outputFileLayout = getParameter("Output file (Layout
Graph)");

        RAPTOR::Data network = RAPTOR::Data(networkFile);
        network.createGraphForMETIS(RAPTOR::TRIP_WEIGHTED |
RAPTOR::TRANSFER_WEIGHTED, true);

        Graph::toGML(outputFileLayout, network.layoutGraph);
    }
};
*/
class WriteTripBasedToCSV : public ParameterizedCommand {
public:
  WriteTripBasedToCSV(BasicShell &shell)
      : ParameterizedCommand(shell, "writeTripBasedToCSV",
                             "Writes all the TripBased Data into csv files.") {
    addParameter("Trip Based Binary");
    addParameter("Output file");
  }

  virtual void execute() noexcept {
    const std::string networkFile = getParameter("Trip Based Binary");
    const std::string outputFile = getParameter("Output file");

    TripBased::Data network = TripBased::Data(networkFile);
    network.raptorData.dontUseImplicitDepartureBufferTimes();
    network.raptorData.dontUseImplicitArrivalBufferTimes();
    network.raptorData.writeCSV(outputFile);

    Graph::toEdgeListCSV(outputFile + "transfers", network.stopEventGraph);
  }
};
/*
class WriteTripBasedToGraphML : public ParameterizedCommand {
public:
    WriteTripBasedToGraphML(BasicShell& shell)
        : ParameterizedCommand(shell, "writeTripBasedToGraphML", "Writes the
StopEvent Graph into a GraphML file.")
    {
        addParameter("Trip Based Binary");
        addParameter("Output file (StopEvent Graph)");
        addParameter("Output file (Layout Graph)");
    }

    virtual void execute() noexcept
    {
        const std::string networkFile = getParameter("Trip Based Binary");
        const std::string outputFileStopEvent = getParameter("Output file
(StopEvent Graph)"); const std::string outputFileLayout = getParameter("Output
file (Layout Graph)");

        TripBased::Data network = TripBased::Data(networkFile);
        network.raptorData.createGraphForMETIS(RAPTOR::TRIP_WEIGHTED |
RAPTOR::TRANSFER_WEIGHTED, true);

        Graph::toGML(outputFileLayout, network.raptorData.layoutGraph);
        Graph::toGML(outputFileStopEvent, network.stopEventGraph);
    }
};
*/

class ExportTEGraphToHubLabelFile : public ParameterizedCommand {
public:
  ExportTEGraphToHubLabelFile(BasicShell &shell)
      : ParameterizedCommand(shell, "exportTEGraphToHubLabelFile",
                             "Writes all the TE Data into text file.") {
    addParameter("TE Binary");
    /* addParameter("Output file"); */
    addParameter("Output file (Dimacs)");
    addParameter("Output file (Paths)");
    /* addParameter("Output file (Metis)"); */
    /* addParameter("Output file (Additional Info)"); */
  }

  virtual void execute() noexcept {
    const std::string networkFile = getParameter("TE Binary");
    /* const std::string outputFile = getParameter("Output file"); */
    const std::string outputFileDimcas = getParameter("Output file (Dimacs)");
    const std::string outputFilePaths = getParameter("Output file (Paths)");
    /* const std::string outputFileMetis = getParameter("Output file (Metis)");
     */
    /* const std::string outputFileOrder = getParameter("Output file (Additional
     * Info)"); */

    TE::Data network = TE::Data(networkFile);

    Graph::toDimacs(outputFileDimcas, network.timeExpandedGraph,
                    network.timeExpandedGraph[Hop]);
    std::cout << "Graph exported successfully to " << outputFileDimcas
              << std::endl;

    network.exportPaths(outputFilePaths);

    /* std::ofstream file(outputFile); */
    /* if (!file.is_open()) { */
    /*     std::cerr << "Error: Could not open the file " << outputFile <<
     * std::endl; */
    /*     return; */
    /* } */

    /* // Write each edge in the format: [src_id] [dst_id] [arc_length] */
    /* for (const auto [edge, from] :
     * network.timeExpandedGraph.edgesWithFromVertex()) { */
    /*     file << static_cast<size_t>(from + 1) << " " <<
     * static_cast<size_t>(network.timeExpandedGraph.get(ToVertex, edge) + 1) <<
     * " " << static_cast<size_t>(network.timeExpandedGraph.get(TravelTime,
     * edge)) << std::endl; */
    /* } */

    /* // Close the file stream */
    /* file.close(); */
    /* std::cout << "Graph exported successfully to " << outputFile <<
     * std::endl; */

    /* TimeExpandedGraph reverseTEG; */
    /* Graph::copy(network.timeExpandedGraph, reverseTEG); */
    /* reverseTEG.revert(); */

    /* Progress progressWriting(network.timeExpandedGraph.numVertices()); */

    /* unsigned long n = network.timeExpandedGraph.numVertices(); */
    /* unsigned long m = network.timeExpandedGraph.numEdges(); */

    /* std::ofstream metisFile(outputFileMetis); */

    /* // n [NUMBER of nodes]  m [NUMBER of edges]     f [int] */
    /* // f values: */
    /* /1* */
    /*         f values: */
    /* 1 :     edge-weighted graph */
    /* 10:     node-weighted graph */
    /* 11:     edge & node - weighted graph */
    /*  *1/ */

    /* metisFile << n << " " << m << " 1"; */

    /* for (Vertex vertex : network.timeExpandedGraph.vertices()) { */
    /*   metisFile << "\n"; */
    /*   for (Edge edge : reverseTEG.edgesFrom(vertex)) { */
    /*     metisFile << std::size_t(reverseTEG.get(ToVertex, edge).value() + 1)
     */
    /*               << " 1 "; */
    /*   } */
    /*   for (Edge edge : network.timeExpandedGraph.edgesFrom(vertex)) { */
    /*     metisFile << std::size_t( */
    /*                      network.timeExpandedGraph.get(ToVertex,
     * edge).value() + */
    /*                      1) */
    /*               << " 1 "; */
    /*   } */
    /*   progressWriting++; */
    /* } */

    /* metisFile.close(); */
    /* progressWriting.finished(); */

    /*     network.writeAdditionalInfoOfVertex(outputFileOrder); */
    /*     std::cout << "Additional info exported successfully to " <<
     * outputFileOrder */
    /*               << std::endl; */
  }
};

class WriteRAPTORLayoutGraphToMetis : public ParameterizedCommand {
public:
  WriteRAPTORLayoutGraphToMetis(BasicShell &shell)
      : ParameterizedCommand(shell, "writeRAPTORLayoutGraphToMetis",
                             "Writes the layout graph of the given RAPTOR data "
                             "as metis graph.") {
    addParameter("RAPTOR Binary");
    addParameter("Output file");
  }

  virtual void execute() noexcept {
    const std::string networkFile = getParameter("RAPTOR Binary");
    const std::string outputFile = getParameter("Output file");

    RAPTOR::Data data(networkFile);
    data.writeLayoutGraphToFile(outputFile);
  }
};
