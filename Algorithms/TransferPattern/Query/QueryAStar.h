/**********************************************************************************

 Copyright (c) 2023-2025 Patrick Steil

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

#include <unordered_map>
#include <vector>

#include "../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../../DataStructures/TransferPattern/Data.h"
#include "../../../DataStructures/TransferPattern/Entities/Bags.h"
#include "../../../Helpers/Vector/Vector.h"
#include "Profiler.h"
#include "TimestampedAlreadySeen.h"

namespace TransferPattern {

template <typename PROFILER = NoProfiler>
class QueryAStar {
 public:
  using Profiler = PROFILER;
  using Type = Query<Profiler>;

 public:
  struct DijkstraLabel {
    DijkstraLabel()
        : arrivalTime(never),
          parentDepartureTime(never),
          numberOfTrips(255),
          routeId(noRouteId),
          parentStop(noStop),
          parentIndex(-1),
          bestTravelTime(0),
          bestNumTrips(0) {}

    template <typename LABEL>
    DijkstraLabel(LABEL &parentLabel, int travelTime)
        : arrivalTime(parentLabel.arrivalTime + travelTime),
          parentDepartureTime(parentLabel),
          numberOfTrips(parentLabel.numberOfTrips),
          routeId(noRouteId),
          parentStop(parentLabel.parentStop),
          parentIndex(parentLabel.parentIndex),
          bestTravelTime(0),
          bestNumTrips(0) {}

    DijkstraLabel(const DijkstraLabel &other)
        : arrivalTime(other.arrivalTime + other.bestTravelTime),
          parentDepartureTime(other.parentDepartureTime),
          numberOfTrips(other.numberOfTrips + other.bestNumTrips),
          routeId(other.routeId),
          parentStop(other.parentStop),
          parentIndex(other.parentIndex),
          bestTravelTime(other.bestTravelTime),
          bestNumTrips(other.bestNumTrips) {}

    DijkstraLabel(int newArrivalTime, int newParentDepartureTime,
                  int newNumberOfTrips, RouteId newRouteId, StopId parentStop,
                  size_t parentIndex, int newBestTravelTime,
                  uint8_t newBestNumTrips)
        : arrivalTime(newArrivalTime),
          parentDepartureTime(newParentDepartureTime),
          numberOfTrips(newNumberOfTrips),
          routeId(newRouteId),
          parentStop(parentStop),
          parentIndex(parentIndex),
          bestTravelTime(newBestTravelTime),
          bestNumTrips(newBestNumTrips) {}

    DijkstraLabel(int departureTime, StopId sourceStop)
        : arrivalTime(departureTime),
          parentDepartureTime(departureTime),
          numberOfTrips(0),
          routeId(noRouteId),
          parentStop(sourceStop),
          parentIndex(0),
          bestTravelTime(0),
          bestNumTrips(0) {}

    inline void set(int newArrivalTime, int newParentDepartureTime,
                    int newNumberOfTrips, RouteId newRouteId,
                    StopId newParentStop, size_t newParentIndex,
                    int newBestTravelTime, uint8_t newBestNumTrips) {
      arrivalTime = newArrivalTime;
      parentDepartureTime = newParentDepartureTime;
      numberOfTrips = newNumberOfTrips;
      routeId = newRouteId;
      parentStop = newParentStop;
      parentIndex = newParentIndex;
      bestTravelTime = newBestTravelTime;
      bestNumTrips = newBestNumTrips;
    }

    inline int getKey() const { return arrivalTime + bestTravelTime; }

    inline bool hasSmallerKey(DijkstraLabel *other) const {
      return getKey() < other->getKey();
    }

    template <typename OTHER_LABEL>
    inline bool dominates(OTHER_LABEL &other) const {
      return arrivalTime <= other.arrivalTime &&
             numberOfTrips <= other.numberOfTrips;
    }

    friend std::ostream &operator<<(std::ostream &out, DijkstraLabel &r) {
      return out << r.arrivalTime << "," << (int)r.numberOfTrips << ","
                 << (int)r.parentStop << "," << (int)r.routeId;
    }

    int arrivalTime;
    int parentDepartureTime;
    uint8_t numberOfTrips;
    RouteId routeId;
    StopId parentStop;
    size_t parentIndex;
    int bestTravelTime;
    uint8_t bestNumTrips;
  };

  using DijkstraBagType = DijkstraBag<DijkstraLabel>;

 public:
  QueryAStar(Data &data)
      : data(data),
        sourceStop(noVertex),
        targetStop(noVertex),
        sourceDepartureTime(0),
        queryGraph(),
        queue(data.maxNumVerticesAndNumEdgesInTP().first),
        left(0),
        right(0),
        alreadySeen(data.maxNumVerticesAndNumEdgesInTP().first),
        dijkstraBags(data.raptorData.numberOfStops()),
        timestampsForBags(data.raptorData.numberOfStops(), 0),
        currentTimestamp(0) {
    profiler.registerPhases({PHASE_EXTRACT_QUERY_GRAPH, PHASE_CLEAR,
                             /* PHASE_CLEAR_QUERY_GRAPH, PHASE_CLEAR_PQ, */
                             PHASE_INIT_SOURCE_LABELS, PHASE_EVAL_GRAPH,
                             PHASE_EXTRACT_JOURNEYS});
    profiler.registerMetrics(
        {METRIC_NUM_VERTICES_QUERY_GRAPH, METRIC_NUM_EDGES_QUERY_GRAPH,
         METRIC_SEETLED_VERTICES, METRIC_RELAXED_TRANSFER_EDGES,
         METRIC_INCORPERATED_LABELS});
  }

  // ######

  // * the two main methods, call to run a query from source to target departing
  // @ departureTime
  inline void run(const Vertex source, const int departureTime,
                  const Vertex target) {
    AssertMsg(data.raptorData.isStop(source),
              "Source " << (int)source << " is not a valid stop!");
    AssertMsg(data.raptorData.isStop(target),
              "Target " << (int)target << " is not a valid stop!");

    run(StopId(source), departureTime, StopId(target));
  }

  inline void run(StopId source, int departureTime, StopId target) {
    profiler.start();

    clear();
    sourceStop = source;
    targetStop = target;
    sourceDepartureTime = departureTime;

    prepBag(sourceStop);
    prepBag(targetStop);

    extractQueryGraph();
    initializeSourceLabels();
    evaluateQueryGraph();
    profiler.done();
  }

  inline std::vector<RAPTOR::Journey> getJourneys() {
    return getJourneys(Vertex(targetStop));
  }

  inline std::vector<RAPTOR::Journey> getJourneys(Vertex vertex) {
    std::vector<RAPTOR::Journey> journeys;
    for (size_t index(0); index < dijkstraBags[vertex].nonHeapSize(); ++index)
      getJourney(journeys, vertex, index);
    return journeys;
  }

  inline void getJourney(std::vector<RAPTOR::Journey> &journeys, Vertex vertex,
                         size_t index) {
    RAPTOR::Journey journey;
    do {
      AssertMsg(timestampsForBags[vertex] == currentTimestamp,
                "Something is wrong!");
      DijkstraLabel &label = dijkstraBags[vertex].access(index);
      journey.emplace_back(label.parentStop, StopId(vertex),
                           label.parentDepartureTime, label.arrivalTime,
                           label.routeId != noRouteId, label.routeId);
      vertex = Vertex(label.parentStop);
      index = label.parentIndex;
    } while (journey.back().from != sourceStop);

    journeys.emplace_back(Vector::reverse(journey));
  }

  // ######

 private:
  // @todo check performance
  inline void clear() noexcept {
    profiler.startPhase();

    /* profiler.startPhase(); */
    queryGraph.clear();
    queryGraph.addVertices(data.raptorData.numberOfStops());

    // should prop be evaluated which reserve size is fastest
    queryGraph.reserve(data.raptorData.numberOfStops(),
                       data.raptorData.numberOfStops() << 1);
    /* profiler.donePhase(PHASE_CLEAR_QUERY_GRAPH); */

    left = 0;
    right = 0;

    alreadySeen.clear();

    /* profiler.startPhase(); */
    Q.clear();
    /* profiler.donePhase(PHASE_CLEAR_PQ); */

    ++currentTimestamp;

    if (currentTimestamp == 0) [[unlikely]]
      Vector::fill(dijkstraBags);

    profiler.donePhase(PHASE_CLEAR);
  }

  // ######

  // * Extracting the Query Graph from TP of stop
  inline void extractQueryGraph() {
    profiler.startPhase();

    const StaticDAGTransferPattern &sourceTP =
        data.transferPatternOfStop[sourceStop];

    Vertex currentVertex(targetStop);

    addVertexToQueryGraph(currentVertex);

    while (queueIsNotEmpty()) {
      currentVertex = popFront();

      // relax all outgoing edges and add (reversed edge) to queryGraph
      for (const Edge edge : sourceTP.edgesFrom(currentVertex)) {
        Vertex successor = sourceTP.get(ToVertex, edge);
        Vertex viaSuccessor = sourceTP.get(ViaVertex, successor);

        // insert into queue
        if (!alreadySeen.contains(successor)) addVertexToQueryGraph(successor);

        bool hasRouteEdge = false;
        bool hasWalkingEdge = false;
        for (const Edge e : queryGraph.edgesFrom(viaSuccessor)) {
          if (queryGraph.get(ToVertex, e) ==
              sourceTP.get(ViaVertex, currentVertex)) {
            if (queryGraph.get(TravelTime, e) == -1)
              hasRouteEdge = true;
            else
              hasWalkingEdge = true;
          }
        }
        if (hasWalkingEdge && hasRouteEdge) continue;
        if ((sourceTP.get(TravelTime, edge) == -1 && !hasRouteEdge) ||
            (!hasWalkingEdge)) {
          queryGraph
              .addEdge(sourceTP.get(ViaVertex, successor),
                       sourceTP.get(ViaVertex, currentVertex))
              .set(TravelTime, sourceTP.get(TravelTime, edge));
          profiler.countMetric(METRIC_NUM_EDGES_QUERY_GRAPH);
        }
      }
    }

    profiler.donePhase(PHASE_EXTRACT_QUERY_GRAPH);
  }

  inline void addVertexToQueryGraph(Vertex vertex) {
    insertIntoQueue(vertex);
    alreadySeen.insert(vertex);
    profiler.countMetric(METRIC_NUM_VERTICES_QUERY_GRAPH);
  }

  // * queue operations for the query graph
  inline void insertIntoQueue(Vertex vertex) {
    AssertMsg(right <= queue.size(), "Right is out of bounds!");
    if (right == queue.size()) [[unlikely]] {
      // the queue is full, so push back resizes
      ++right;
      queue.push_back(vertex);
    } else {
      queue[right++] = vertex;
    }
  }

  inline Vertex popFront() noexcept {
    AssertMsg(left < queue.size(), "Left is out of bounds!");
    return queue[left++];
  }

  inline bool queueIsNotEmpty() noexcept { return left < right; }

  inline void prepBag(const Vertex vertex) {
    AssertMsg(data.raptorData.isStop(StopId(vertex)), "Vertex is not a stop!");
    AssertMsg(vertex < timestampsForBags.size(), "Vertex is out of bounds!");

    if (timestampsForBags[vertex] == currentTimestamp) [[unlikely]]
      return;

    dijkstraBags[vertex] = DijkstraBagType();
    timestampsForBags[vertex] = currentTimestamp;
  }

  // ######

  inline void initializeSourceLabels() {
    profiler.startPhase();

    DijkstraLabel sourceLabel(sourceDepartureTime, StopId(sourceStop));
    // this adds the sourceLabel to the bag of sourceStop
    dijkstraBags[sourceStop].merge(sourceLabel);
    Q.update(&dijkstraBags[sourceStop]);

    profiler.donePhase(PHASE_INIT_SOURCE_LABELS);
  }

  // ######

  // * Evaluate the Query Graph using Multi Criteria Dijkstra
  inline void evaluateQueryGraph() {
    profiler.startPhase();

    while (!Q.empty()) {
      DijkstraBagType *uBag = Q.extractFront();
      const DijkstraLabel &uLabel = uBag->extractFront();
      const Vertex u = Vertex(uBag - &(dijkstraBags[0]));
      size_t parentIndex = dijkstraBags[u].getIndex(uLabel);

      if (!uBag->heapEmpty()) Q.update(uBag);

      for (Edge edge : queryGraph.edgesFrom(u)) {
        const Vertex v = queryGraph.get(ToVertex, edge);

        // parent hop reduction
        if (v == uLabel.parentStop) [[unlikely]]
          continue;

        int travelTime = queryGraph.get(TravelTime, edge);
        // this seems odd, why add -1 to the arrival time?
        // well, if edge uses a route, then it will be overwritten
        int newArrivalTime = uLabel.arrivalTime + travelTime;
        uint8_t newNumberOfTrips = uLabel.numberOfTrips;
        RouteId usedRoute = noRouteId;
        DijkstraLabel vLabel;

        // travelTime == -1 <=> route edge
        if (travelTime == -1) [[likely]] {
          ++newNumberOfTrips;
          std::pair<RouteId, int> pair =
              directConnectionIntersection(u, v, uLabel.arrivalTime);
          newArrivalTime = pair.second;
          usedRoute = pair.first;
        }

        vLabel.set(
            newArrivalTime, uLabel.arrivalTime, newNumberOfTrips, usedRoute,
            StopId(u), parentIndex,
            data.getLowerBoundTravelTime(StopId(targetStop), StopId(v)),
            data.getLowerBoundNumberOfTrips(StopId(targetStop), StopId(v)));

        arrivalByEdge(v, vLabel);
      }
      profiler.countMetric(METRIC_SEETLED_VERTICES);
    }

    profiler.donePhase(PHASE_EVAL_GRAPH);
  }

  inline bool arrivalByEdge(const Vertex vertex, const DijkstraLabel &label) {
    AssertMsg(label.arrivalTime >= sourceDepartureTime,
              "Arriving by route BEFORE departing from the source (source "
              "departure time: "
                  << String::secToTime(sourceDepartureTime) << " ["
                  << sourceDepartureTime
                  << "], arrival time: " << String::secToTime(label.arrivalTime)
                  << " [" << label.arrivalTime << "])!");

    profiler.countMetric(METRIC_RELAXED_TRANSFER_EDGES);
    // Target Pruning - adapted
    // copy label and add the lower bound for trips
    DijkstraLabel skewedCopy(label);

    if (dijkstraBags[targetStop].dominates(skewedCopy)) return false;

    prepBag(vertex);
    // normal (?) pruning... if the label is already dominated by other labels
    // in it's bag, don't add it
    if (!dijkstraBags[vertex].merge(label)) return false;

    profiler.countMetric(METRIC_INCORPERATED_LABELS);

    Q.update(&dijkstraBags[vertex]);
    return true;
  }

  // ######

  // * eval direct connection

  inline std::pair<RouteId, int> directConnectionIntersection(
      const Vertex from, const Vertex to, const int departureTime) {
    AssertMsg(data.raptorData.isStop(StopId(from)),
              "From " << from << " is not a valid stop");
    AssertMsg(data.raptorData.isStop(StopId(to)),
              "To " << to << "is not a valid stop");

    std::vector<RAPTOR::RouteSegment> &fromLookup =
        data.stopLookup[from].incidentLines;
    std::vector<RAPTOR::RouteSegment> &toLookup =
        data.stopLookup[to].incidentLines;

    AssertMsg(std::is_sorted(fromLookup.begin(), fromLookup.end()),
              "StopLookup from From is not sorted!");
    AssertMsg(std::is_sorted(toLookup.begin(), toLookup.end()),
              "StopLookup from From is not sorted!");

    std::pair<RouteId, int> result = std::make_pair(noRouteId, INFTY);

    size_t i(0);
    size_t j(0);

    size_t firstReachableTripIndex(-1);
    int currentArrivalTime(-1);

    while (i < fromLookup.size() && j < toLookup.size()) {
      // if fromLookup[i] and toLookup[j] are on the same line ...
      // ... *and* fromLookup[i] is before toLookup[j] ...
      // ... then evaluate and get the intersection
      if (fromLookup[i].routeId == toLookup[j].routeId &&
          fromLookup[i].stopIndex < toLookup[j].stopIndex) {
        // get the first reachable trip departing >= departureTime
        firstReachableTripIndex = data.earliestTripIndexOfLineByStopIndex(
            fromLookup[i].stopIndex, fromLookup[i].routeId, departureTime);

        // is this firstReachableTripIndex valid? I.e., firstReachableTripIndex
        // != (-1)
        if (firstReachableTripIndex != (size_t)-1) {
          currentArrivalTime = data.getArrivalTime(fromLookup[i].routeId,
                                                   firstReachableTripIndex,
                                                   toLookup[j].stopIndex);
          if (currentArrivalTime < result.second) {
            result = std::make_pair(fromLookup[i].routeId, currentArrivalTime);
          }
        }
        ++i;
        ++j;
      } else {
        if (fromLookup[i] < toLookup[j]) {
          ++i;
        } else {
          ++j;
        }
      }
    }

    return result;
  }

 public:
  inline Profiler &getProfiler() noexcept { return profiler; }

 private:
  Data &data;

  Vertex sourceStop;
  Vertex targetStop;
  int sourceDepartureTime;

  DynamicQueryGraph queryGraph;
  std::vector<Vertex> queue;
  size_t left, right;
  TimestampedAlreadySeen alreadySeen;

  std::vector<DijkstraBagType> dijkstraBags;
  std::vector<uint16_t> timestampsForBags;
  uint16_t currentTimestamp;
  ExternalKHeap<4, DijkstraBagType> Q;

  Profiler profiler;
};

}  // namespace TransferPattern
