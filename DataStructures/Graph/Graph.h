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

#include "Classes/DynamicGraph.h"
#include "Classes/EdgeList.h"
#include "Classes/GraphInterface.h"
#include "Classes/StaticGraph.h"
#include "Utils/Utils.h"

using NoVertexAttributes = List<>;
using WithCoordinates = List<Attribute<Coordinates, Geometry::Point>>;
using WithSize = List<Attribute<Size, size_t>>;

using NoEdgeAttributes = List<>;
using WithTravelTime = List<Attribute<TravelTime, int>>;
using WithTravelTimeAndDistance =
    List<Attribute<TravelTime, int>, Attribute<Distance, int>>;
using WithTravelTimeAndEdgeFlags =
    List<Attribute<TravelTime, int>, Attribute<EdgeFlags, std::vector<bool>>>;
using WithTravelTimeAndBundleSize =
    List<Attribute<TravelTime, int>, Attribute<BundleSize, int>>;
using WithReverseEdges = List<Attribute<ReverseEdge, Edge>>;
using WithCapacity = List<Attribute<Capacity, int>>;
using WithWeight = List<Attribute<Weight, int>>;
using WithWeightAndCoordinates =
    List<Attribute<Weight, int>, Attribute<Coordinates, Geometry::Point>>;
using WithWeightAndCoordinatesAndSize =
    List<Attribute<Weight, int>, Attribute<Coordinates, Geometry::Point>,
         Attribute<Size, size_t>>;
using WithViaVertex = List<Attribute<ViaVertex, Vertex>>;
using WithViaVertexAndWeight =
    List<Attribute<ViaVertex, Vertex>, Attribute<Weight, int>>;
using WithReverseEdgesAndViaVertex =
    List<Attribute<ReverseEdge, Edge>, Attribute<ViaVertex, Vertex>>;
using WithReverseEdgesAndWeight =
    List<Attribute<ReverseEdge, Edge>, Attribute<Weight, int>>;
using WithReverseEdgesAndCapacity =
    List<Attribute<ReverseEdge, Edge>, Attribute<Capacity, int>>;

using StrasserGraph =
    StaticGraph<NoVertexAttributes, WithTravelTimeAndDistance>;
using StrasserGraphWithCoordinates =
    StaticGraph<WithCoordinates, WithTravelTimeAndDistance>;

using TransferGraph = StaticGraph<WithCoordinates, WithTravelTime>;
using DynamicTransferGraph = DynamicGraph<WithCoordinates, WithTravelTime>;
using TransferEdgeList = EdgeList<WithCoordinates, WithTravelTime>;
using EdgeFlagsTransferGraph =
    DynamicGraph<WithCoordinates, WithTravelTimeAndEdgeFlags>;

using SimpleDynamicGraph = DynamicGraph<NoVertexAttributes, NoEdgeAttributes>;
using SimpleStaticGraph = StaticGraph<NoVertexAttributes, NoEdgeAttributes>;
using SimpleEdgeList = EdgeList<NoVertexAttributes, NoEdgeAttributes>;

using DynamicFlowGraph =
    DynamicGraph<NoVertexAttributes, WithReverseEdgesAndCapacity>;
using StaticFlowGraph =
    StaticGraph<NoVertexAttributes, WithReverseEdgesAndCapacity>;

using CHConstructionGraph =
    EdgeList<NoVertexAttributes, WithViaVertexAndWeight>;
using CHCoreGraph = DynamicGraph<NoVertexAttributes, WithViaVertexAndWeight>;
using CHGraph = StaticGraph<NoVertexAttributes, WithViaVertexAndWeight>;

using DimacsGraph = EdgeList<NoVertexAttributes, WithTravelTime>;
using DimacsGraphWithCoordinates = EdgeList<WithCoordinates, WithTravelTime>;

using TravelTimeGraph = StaticGraph<NoVertexAttributes, WithTravelTime>;

using CondensationGraph = DynamicGraph<WithSize, WithTravelTime>;

using BundledGraph = StaticGraph<WithCoordinates, WithTravelTimeAndBundleSize>;
using DynamicBundledGraph =
    DynamicGraph<WithCoordinates, WithTravelTimeAndBundleSize>;

using StaticGraphWithReverseEdge =
    StaticGraph<NoVertexAttributes, WithReverseEdges>;
using DynamicGraphWithReverseEdge =
    DynamicGraph<NoVertexAttributes, WithReverseEdges>;

// Arc-Flag TB
using DynamicGraphWithWeights = DynamicGraph<WithWeight, WithWeight>;
using DynamicGraphWithWeightsAndCoordinates =
    DynamicGraph<WithWeightAndCoordinates, WithWeight>;
using DynamicGraphWithWeightsAndCoordinatesAndSize =
    DynamicGraph<WithWeightAndCoordinatesAndSize, WithWeight>;

using StaticGraphWithWeightsAndCoordinates =
    StaticGraph<WithWeightAndCoordinates, WithWeight>;
using StaticGraphWithWeightsAndCoordinatesAndSize =
    StaticGraph<WithWeightAndCoordinatesAndSize, WithWeight>;

// New Attributes
using WithARCFlag = List<Attribute<ARCFlag, std::vector<bool>>>;
using WithTravelTimeAndARCFlag =
    List<Attribute<TravelTime, int>, Attribute<ARCFlag, std::vector<bool>>>;

using TransferGraphWithARCFlag =
    StaticGraph<WithCoordinates, WithTravelTimeAndARCFlag>;
using DynamicTransferGraphWithARCFlag =
    DynamicGraph<WithCoordinates, WithTravelTimeAndARCFlag>;
using SimpleDynamicGraphWithARCFlag =
    DynamicGraph<NoVertexAttributes, WithARCFlag>;
using SimpleEdgeListWithARCFlag = EdgeList<NoVertexAttributes, WithARCFlag>;

// TREX
using WithLocalLevel = List<Attribute<LocalLevel, uint8_t>>;
using WithTravelTimeAndLocalLevel =
    List<Attribute<TravelTime, int>, Attribute<LocalLevel, uint8_t>>;
using WithTravelTimeAndLocalLevelAndFromVertex =
    List<Attribute<TravelTime, int>, Attribute<LocalLevel, uint8_t>,
         Attribute<FromVertex, Vertex>>;

using WithLocalLevelAndHop =
    List<Attribute<LocalLevel, uint16_t>, Attribute<Hop, uint8_t>>;
using WithTravelTimeAndLocalLevel =
    List<Attribute<TravelTime, int>, Attribute<LocalLevel, uint8_t>>;
using WithTravelTimeAndLocalLevelAndHop =
    List<Attribute<TravelTime, int>, Attribute<LocalLevel, uint16_t>,
         Attribute<Hop, uint8_t>>;
using WithTravelTimeAndLocalLevelAndHopAndFromVertex =
    List<Attribute<TravelTime, int>, Attribute<LocalLevel, uint16_t>,
         Attribute<Hop, uint8_t>, Attribute<FromVertex, Vertex>>;

using TransferGraphWithLocalLevel =
    StaticGraph<NoVertexAttributes, WithTravelTimeAndLocalLevel>;
using TransferGraphWithLocalLevelAndHop =
    StaticGraph<NoVertexAttributes, WithTravelTimeAndLocalLevelAndHop>;
using DynamicTransferGraphWithLocalLevelAndHop =
    DynamicGraph<NoVertexAttributes, WithTravelTimeAndLocalLevelAndHop>;
using DynamicTransferGraphWithLocalLevelAndHopAndFromVertex =
    DynamicGraph<NoVertexAttributes,
                 WithTravelTimeAndLocalLevelAndHopAndFromVertex>;
using SimpleDynamicGraphWithLocalLevel =
    DynamicGraph<NoVertexAttributes, WithLocalLevel>;
using SimpleEdgeListWithLocalLevel =
    EdgeList<NoVertexAttributes, WithLocalLevel>;

using EdgeListTransferGraphWithLocalLevelAndHop =
    EdgeList<NoVertexAttributes, WithTravelTimeAndLocalLevelAndHop>;

using WithHop = List<Attribute<Hop, uint8_t>>;
using WithStopVertex = List<Attribute<StopVertex, StopId>>;

using DynamicTripBasedTimeExpGraph = DynamicGraph<NoVertexAttributes, WithHop>;
using TripBasedTimeExpGraph = StaticGraph<NoVertexAttributes, WithHop>;

// Transfer Pattern
using EdgeListDAGTransferPattern = EdgeList<WithViaVertex, WithTravelTime>;
using DynamicDAGTransferPattern = DynamicGraph<WithViaVertex, WithTravelTime>;
using StaticDAGTransferPattern = StaticGraph<WithViaVertex, WithTravelTime>;

// @todo check which type of graph
using DynamicQueryGraph = DynamicGraph<NoVertexAttributes, WithTravelTime>;

// TD
using WithDurationFunctionAndTravelTimeAndTransferCost = List<
    Attribute<DurationFunction, std::vector<std::pair<uint32_t, uint32_t>>>,
    Attribute<TravelTime, int>, Attribute<TransferCost, uint8_t>>;
using WithRouteVertex = List<Attribute<RouteVertex, RouteId>>;

using DynamicTimeDependentRouteGraph =
    DynamicGraph<WithRouteVertex,
                 WithDurationFunctionAndTravelTimeAndTransferCost>;
using TimeDependentRouteGraph =
    StaticGraph<WithRouteVertex,
                WithDurationFunctionAndTravelTimeAndTransferCost>;

// TE
using WithTravelTimeAndHop =
    List<Attribute<TravelTime, int>, Attribute<Hop, uint8_t>>;

using DynamicTimeExpandedGraph =
    DynamicGraph<WithStopVertex, WithTravelTimeAndHop>;
using TimeExpandedGraph = StaticGraph<WithStopVertex, WithTravelTimeAndHop>;

// TB-TE
using WithCellId = List<Attribute<CellId, uint16_t>>;
using WithTransferCostAndOriginalEdge =
    List<Attribute<TransferCost, uint16_t>, Attribute<OriginalEdge, Edge>>;

using DynamicTBTEGraph =
    DynamicGraph<WithCellId, WithTransferCostAndOriginalEdge>;
using EdgeListTBTEGraph = EdgeList<WithCellId, WithTransferCostAndOriginalEdge>;

#include "Utils/Conversion.h"
#include "Utils/IO.h"
