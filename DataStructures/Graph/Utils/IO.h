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

#include <vector>

#include "../../../Helpers/Assert.h"
#include "../../../Helpers/IO/Serialization.h"
#include "../../Geometry/Point.h"
#include "../Classes/DynamicGraph.h"
#include "../Classes/EdgeList.h"
#include "../Classes/GraphInterface.h"
#include "../Classes/StaticGraph.h"
#include "Conversion.h"

namespace Graph {

constexpr int dimacs_scale = 1000000;

template <typename GRAPH>
inline void fromDimacs(const std::string &fileBaseName, GRAPH &graph) noexcept {
  EdgeList<typename GRAPH::ListOfVertexAttributes,
           typename GRAPH::ListOfEdgeAttributes>
      edgeList;
  edgeList.fromDimacs(fileBaseName);
  move(std::move(edgeList), graph);
}

template <typename GRAPH, typename WEIGHT_TYPE>
inline void toDimacs(const std::string &fileBaseName, const GRAPH &graph,
                     const std::vector<WEIGHT_TYPE> &weight) noexcept {
  std::ofstream grOs(fileBaseName + ".gr");
  AssertMsg(grOs, "Cannot create output stream for " << fileBaseName << ".gr");
  AssertMsg(grOs.is_open(),
            "Cannot open output stream for " << fileBaseName << ".gr");
  grOs << "p sp " << graph.numVertices() << " " << graph.numEdges()
       << std::endl;
  for (const auto [edge, from] : graph.edgesWithFromVertex()) {
    grOs << "a " << (from + 1) << " " << (graph.get(ToVertex, edge) + 1) << " ";
    if constexpr (std::is_same_v<WEIGHT_TYPE, uint8_t>) {
      grOs << (int)weight[edge];
    } else {
      grOs << weight[edge];
    }
    grOs << std::endl;
  }
  grOs.close();
  if constexpr (GRAPH::HasVertexAttribute(Coordinates)) {
    std::ofstream coOs(fileBaseName + ".co");
    AssertMsg(coOs,
              "Cannot create output stream for " << fileBaseName << ".co");
    AssertMsg(coOs.is_open(),
              "Cannot open output stream for " << fileBaseName << ".co");
    coOs << "p aux sp co " << graph.numVertices() << std::endl;
    for (const Vertex vertex : graph.vertices()) {
      coOs << "v " << (vertex + 1) << " "
           << static_cast<long long>(graph.get(Coordinates, vertex).x *
                                     dimacs_scale)
           << " "
           << static_cast<long long>(graph.get(Coordinates, vertex).y *
                                     dimacs_scale)
           << std::endl;
    }
    coOs.close();
  }
}

template <typename GRAPH>
inline void toDimacs(const std::string &fileBaseName,
                     const GRAPH &graph) noexcept {
  toDimacs(fileBaseName, graph, graph.get(Weight));
}

// Patrick Steil - to export TB Data conviently with Arc-Flag Info
// ----------------------------

std::string join(std::vector<bool> const &vec) {
  if (vec.empty()) {
    return std::string();
  }

  return std::accumulate(vec.begin() + 1, vec.end(),
                         std::to_string((int)vec[0]),
                         [](const std::string &a, bool b) {
                           return a + ";" + std::to_string((int)b);
                         });
}

template <typename GRAPH>
inline void toEdgeListCSV(const std::string &fileBaseName,
                          const GRAPH &graph) noexcept {
  std::ofstream csv(fileBaseName + ".csv");
  AssertMsg(csv, "Cannot create output stream for " << fileBaseName << ".csv");
  AssertMsg(csv.is_open(),
            "Cannot open output stream for " << fileBaseName << ".csv");

  csv << "FromVertex,ToVertex";

  if constexpr (GRAPH::HasEdgeAttribute(TravelTime)) csv << ",TravelTime";
  if constexpr (GRAPH::HasEdgeAttribute(Distance)) csv << ",Distance";
  if constexpr (GRAPH::HasEdgeAttribute(ViaVertex)) csv << ",ViaVertex";
  if constexpr (GRAPH::HasEdgeAttribute(Weight)) csv << ",Weight";
  if constexpr (GRAPH::HasEdgeAttribute(Capacity)) csv << ",Capacity";
  if constexpr (GRAPH::HasEdgeAttribute(BundleSize)) csv << ",BundleSize";
  if constexpr (GRAPH::HasEdgeAttribute(ReverseEdge)) csv << ",ReverseEdge";
  if constexpr (GRAPH::HasEdgeAttribute(EdgeFlags)) csv << ",EdgeFlags";
  if constexpr (GRAPH::HasEdgeAttribute(ARCFlag)) csv << ",ARCFlag";
  if constexpr (GRAPH::HasEdgeAttribute(LocalLevel)) csv << ",LocalLevel";
  if constexpr (GRAPH::HasEdgeAttribute(Hop)) csv << ",Hop";
  if constexpr (GRAPH::HasEdgeAttribute(StopVertex)) csv << ",StopVertexx";

  csv << "\n";

  for (const auto [edge, from] : graph.edgesWithFromVertex()) {
    csv << size_t(from) << "," << size_t(graph.get(ToVertex, edge));
    if constexpr (GRAPH::HasEdgeAttribute(TravelTime))
      csv << "," << (int)graph.get(TravelTime, edge);
    if constexpr (GRAPH::HasEdgeAttribute(Distance))
      csv << "," << (int)graph.get(Distance, edge);
    if constexpr (GRAPH::HasEdgeAttribute(ViaVertex))
      csv << "," << size_t(graph.get(ViaVertex, edge));
    if constexpr (GRAPH::HasEdgeAttribute(Weight))
      csv << "," << (int)graph.get(Weight, edge);
    if constexpr (GRAPH::HasEdgeAttribute(Capacity))
      csv << "," << (int)graph.get(Capacity, edge);
    if constexpr (GRAPH::HasEdgeAttribute(BundleSize))
      csv << "," << (int)graph.get(BundleSize, edge);
    if constexpr (GRAPH::HasEdgeAttribute(ReverseEdge))
      csv << "," << size_t(graph.get(ReverseEdge, edge));
    if constexpr (GRAPH::HasEdgeAttribute(EdgeFlags))
      csv << "," << join(graph.get(EdgeFlags, edge));
    if constexpr (GRAPH::HasEdgeAttribute(ARCFlag))
      csv << "," << join(graph.get(ARCFlag, edge));
    if constexpr (GRAPH::HasEdgeAttribute(LocalLevel))
      csv << "," << (int)graph.get(LocalLevel, edge);
    if constexpr (GRAPH::HasEdgeAttribute(Hop))
      csv << "," << (int)graph.get(Hop, edge);
    if constexpr (GRAPH::HasEdgeAttribute(StopVertex))
      csv << "," << (int)graph.get(StopVertex, edge);
    csv << "\n";
  }
  csv.close();
}

// ----------------------------

template <typename GRAPH>
inline void toGML(const std::string &fileBaseName,
                  const GRAPH &graph) noexcept {
  std::ofstream gml(fileBaseName + ".graphml");
  AssertMsg(gml,
            "Cannot create output stream for " << fileBaseName << ".graphml");
  AssertMsg(gml.is_open(),
            "Cannot open output stream for " << fileBaseName << ".graphml");
  gml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  gml << "<graphml xmlns=\"http://graphml.graphdrawing.org/xmlns\" "
         "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
         "xsi:schemaLocation=\"http://graphml.graphdrawing.org/xmlns "
         "http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd\">\n";
  // Added some attributes to graphml
  // First: nodes
  if constexpr (GRAPH::HasVertexAttribute(Coordinates)) {
    gml << "        <key id=\"lat_n\" for=\"node\" attr.name=\"latitude\" "
           "attr.type=\"double\"/>\n";
    gml << "        <key id=\"lon_n\" for=\"node\" attr.name=\"longitude\" "
           "attr.type=\"double\"/>\n";
  }
  if constexpr (GRAPH::HasVertexAttribute(Size))
    gml << "        <key id=\"size_n\" for=\"node\" attr.name=\"size\" "
           "attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasVertexAttribute(Weight))
    gml << "        <key id=\"weight_n\" for=\"node\" attr.name=\"weight\" "
           "attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasVertexAttribute(ViaVertex))
    gml << "        <key id=\"viavertex_n\" for=\"node\" "
           "attr.name=\"viavertex\" attr.type=\"int\"/>\n";

  // Second: edges

  if constexpr (GRAPH::HasEdgeAttribute(TravelTime))
    gml << "        <key id=\"traveltime_e\" for=\"edge\" "
           "attr.name=\"traveltime\" attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(Distance))
    gml << "        <key id=\"distance_e\" for=\"edge\" attr.name=\"distance\" "
           "attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(Size))
    gml << "        <key id=\"size_e\" for=\"edge\" attr.name=\"edgesize\" "
           "attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(BundleSize))
    gml << "        <key id=\"bundlesize_e\" for=\"edge\" "
           "attr.name=\"bundlesize\" attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(Weight))
    gml << "        <key id=\"weight_e\" for=\"edge\" attr.name=\"edgeweight\" "
           "attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(ReverseEdge))
    gml << "        <key id=\"reverseedge_e\" for=\"edge\" "
           "attr.name=\"reverseegde\" attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(Capacity))
    gml << "        <key id=\"capacity_e\" for=\"edge\" attr.name=\"capacity\" "
           "attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(ViaVertex))
    gml << "        <key id=\"viavertex_e\" for=\"edge\" "
           "attr.name=\"viavertex\" attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(ARCFlag))
    gml << "        <key id=\"arcflag_e\" for=\"edge\" attr.name=\"arcflag\" "
           "attr.type=\"string\"/>\n";  // I don't know how else to 'elegantly'
                                        // store a vector of booleans inside
                                        // graphml
  if constexpr (GRAPH::HasEdgeAttribute(LocalLevel))
    gml << "        <key id=\"locallevel_e\" for=\"edge\" "
           "attr.name=\"locallevel\" attr.type=\"int\"/>\n";
  if constexpr (GRAPH::HasEdgeAttribute(Hop))
    gml << "        <key id=\"hop_e\" for=\"edge\" attr.name=\"hop\" "
           "attr.type=\"int\"/>\n";

  gml << "    <graph id=\"G\" edgedefault=\"directed\">\n";
  for (const Vertex vertex : graph.vertices()) {
    gml << "        <node id=\"" << size_t(vertex) << "\">\n";

    // write the attributes
    if constexpr (GRAPH::HasVertexAttribute(Coordinates)) {
      gml << "            <data key=\"lat_n\">"
          << (float)graph.get(Coordinates, vertex).latitude << "</data>\n";
      gml << "            <data key=\"lon_n\">"
          << (float)graph.get(Coordinates, vertex).longitude << "</data>\n";
    }
    if constexpr (GRAPH::HasVertexAttribute(Size))
      gml << "            <data key=\"size_n\">" << (int)graph.get(Size, vertex)
          << "</data>\n";
    if constexpr (GRAPH::HasVertexAttribute(Weight))
      gml << "            <data key=\"weight_n\">"
          << (int)graph.get(Weight, vertex) << "</data>\n";
    if constexpr (GRAPH::HasVertexAttribute(ViaVertex))
      gml << "            <data key=\"viavertex_n\">"
          << (int)graph.get(ViaVertex, vertex) << "</data>\n";

    gml << "        </node>\n";
  }
  for (const auto [edge, from] : graph.edgesWithFromVertex()) {
    gml << "        <edge source=\"" << size_t(from) << "\" target=\""
        << size_t(graph.get(ToVertex, edge)) << "\">\n";

    // write the attributes
    if constexpr (GRAPH::HasEdgeAttribute(TravelTime))
      gml << "            <data key=\"traveltime_e\">"
          << (int)graph.get(TravelTime, edge) << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(Distance))
      gml << "            <data key=\"distance_e\">"
          << (int)graph.get(Distance, edge) << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(Size))
      gml << "            <data key=\"size_e\">" << (int)graph.get(Size, edge)
          << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(BundleSize))
      gml << "            <data key=\"bundlesize_e\">"
          << (int)graph.get(BundleSize, edge) << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(Weight))
      gml << "            <data key=\"weight_e\">"
          << (int)graph.get(Weight, edge) << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(ReverseEdge))
      gml << "            <data key=\"reverseedge_e\">"
          << (int)graph.get(ReverseEdge, edge) << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(Capacity))
      gml << "            <data key=\"capacity_e\">"
          << (int)graph.get(Capacity, edge) << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(ViaVertex))
      gml << "            <data key=\"viavertex_e\">"
          << size_t(graph.get(ViaVertex, edge)) << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(ARCFlag))
      gml << "            <data key=\"arcflag_e\">"
          << join(graph.get(ARCFlag, edge)) << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(LocalLevel))
      gml << "            <data key=\"locallevel_e\">"
          << (int)graph.get(LocalLevel, edge) << "</data>\n";
    if constexpr (GRAPH::HasEdgeAttribute(Hop))
      gml << "            <data key=\"hop_e\">" << (int)graph.get(Hop, edge)
          << "</data>\n";

    gml << "        </edge>\n";
  }
  gml << "    </graph>\n";
  gml << "</graphml>" << std::endl;
  gml.close();
}

template <typename GRAPH>
inline void fromStrasserBinary(const std::string &fileName, GRAPH &graph,
                               const double timeFactor = 1,
                               const double distanceFactor = 1,
                               const double coordinateFactor = 1) noexcept {
  std::vector<float> latitude;
  std::vector<float> longitude;
  std::vector<int> weight;
  std::vector<int> geo_distance;
  std::vector<int> travel_time;
  std::vector<int> first_out;
  std::vector<int> head;
  IO::deserialize(fileName + "/first_out", first_out);
  IO::deserialize(fileName + "/head", head);
  if constexpr (GRAPH::HasVertexAttribute(Coordinates))
    IO::deserialize(fileName + "/latitude", latitude);
  if constexpr (GRAPH::HasVertexAttribute(Coordinates))
    IO::deserialize(fileName + "/longitude", longitude);
  if constexpr (GRAPH::HasEdgeAttribute(Weight))
    IO::deserialize(fileName + "/weight", weight);
  if constexpr (GRAPH::HasEdgeAttribute(Distance))
    IO::deserialize(fileName + "/geo_distance", geo_distance);
  if constexpr (GRAPH::HasEdgeAttribute(TravelTime))
    IO::deserialize(fileName + "/travel_time", travel_time);
  AssertMsg(latitude.size() == longitude.size(),
            "Latitude and longitude vector have different sizes! ("
                << latitude.size() << " vs. " << longitude.size() << ")");
  SimpleDynamicGraph temp;
  temp.addVertices(first_out.size() - 1);
  for (const Vertex vertex : temp.vertices()) {
    for (int i = first_out[vertex]; i < first_out[vertex + 1]; i++) {
      temp.addEdge(vertex, Vertex(head[i]));
    }
  }
  Graph::move(std::move(temp), graph);
  for (const Vertex vertex : graph.vertices()) {
    if constexpr (GRAPH::HasVertexAttribute(Coordinates)) {
      graph.set(Coordinates, vertex,
                Geometry::Point(Construct::LatLong,
                                latitude[vertex] * coordinateFactor,
                                longitude[vertex] * coordinateFactor));
    }
    for (int i = first_out[vertex]; i < first_out[vertex + 1]; i++) {
      const Edge edge = graph.findEdge(vertex, Vertex(head[i]));
      if constexpr (GRAPH::HasEdgeAttribute(Weight)) {
        graph.set(Weight, edge, weight[i]);
      }
      if constexpr (GRAPH::HasEdgeAttribute(Distance)) {
        graph.set(
            Distance, edge,
            static_cast<int>((geo_distance[i] * distanceFactor * 10) + 5) / 10);
      }
      if constexpr (GRAPH::HasEdgeAttribute(TravelTime)) {
        graph.set(
            TravelTime, edge,
            static_cast<int>((travel_time[i] * timeFactor * 10) + 5) / 10);
      }
      (void)edge;
    }
  }
  Assert(graph.satisfiesInvariants());
}

}  // namespace Graph
