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

#include <random>
#include <vector>

#include "../../Helpers/Types.h"

struct VertexQuery {
  VertexQuery(const Vertex source = noVertex, const Vertex target = noVertex,
              const int departureTime = never)
      : source(source), target(target), departureTime(departureTime) {}

  inline friend std::ostream &operator<<(std::ostream &out,
                                         const VertexQuery &query) noexcept {
    return out << query.source << " -> " << query.target << " @ "
               << query.departureTime << std::endl;
  }

  Vertex source;
  Vertex target;
  int departureTime;
};

inline std::vector<VertexQuery>
generateRandomVertexQueries(const size_t numVertices, const size_t numQueries,
                            const int startTime = 0,
                            const int endTime = 24 * 60 * 60) noexcept {
  std::mt19937 randomGenerator(42);
  std::uniform_int_distribution<> vertexDistribution(0, numVertices - 1);
  std::uniform_int_distribution<> timeDistribution(startTime, endTime - 1);
  std::vector<VertexQuery> queries;
  for (size_t i = 0; i < numQueries; i++) {
    queries.emplace_back(Vertex(vertexDistribution(randomGenerator)),
                         Vertex(vertexDistribution(randomGenerator)),
                         timeDistribution(randomGenerator));
  }
  return queries;
}

struct OneToAllQuery {
  OneToAllQuery(const Vertex source = noVertex, const int departureTime = never)
      : source(source), departureTime(departureTime) {}

  Vertex source;
  int departureTime;
};

inline std::vector<OneToAllQuery>
generateRandomOneToAllQueries(const size_t numVertices, const size_t numQueries,
                              const int startTime = 0,
                              const int endTime = 24 * 60 * 60) noexcept {
  std::mt19937 randomGenerator(42);
  std::uniform_int_distribution<> vertexDistribution(0, numVertices - 1);
  std::uniform_int_distribution<> timeDistribution(startTime, endTime - 1);
  std::vector<OneToAllQuery> queries;
  for (size_t i = 0; i < numQueries; i++) {
    queries.emplace_back(Vertex(vertexDistribution(randomGenerator)),
                         timeDistribution(randomGenerator));
  }
  return queries;
}

struct StopQuery {
  StopQuery(const StopId source = noStop, const StopId target = noStop,
            const int departureTime = never)
      : source(source), target(target), departureTime(departureTime) {}

  inline friend std::ostream &operator<<(std::ostream &out,
                                         const StopQuery &query) noexcept {
    return out << query.source << " -> " << query.target << " @ "
               << query.departureTime << std::endl;
  }

  StopId source;
  StopId target;
  int departureTime;
};

inline std::vector<StopQuery>
generateRandomStopQueries(const size_t numStops, const size_t numQueries,
                          const int startTime = 0,
                          const int endTime = 24 * 60 * 60) noexcept {
  std::mt19937 randomGenerator(42);
  std::uniform_int_distribution<> stopDistribution(0, numStops - 1);
  std::uniform_int_distribution<> timeDistribution(startTime, endTime - 1);
  std::vector<StopQuery> queries;
  for (size_t i = 0; i < numQueries; i++) {
    queries.emplace_back(StopId(stopDistribution(randomGenerator)),
                         StopId(stopDistribution(randomGenerator)),
                         timeDistribution(randomGenerator));
  }
  return queries;
}

inline std::vector<StopQuery>
loadFromFileStopQueries(const std::string &fileName) noexcept {
  std::vector<StopQuery> result;
  std::ifstream file(fileName);

  if (!file) {
    std::cerr << "Error: Could not open file " << fileName << "\n";
    return result;
  }

  StopId from, to;
  int departureTime;

  while (file >> from >> to >> departureTime) {
    result.emplace_back(from, to, departureTime);
  }

  if (!file.eof()) {
    std::cerr << "Warning: Some lines in " << fileName
              << " could not be parsed.\n";
  }

  return result; // RVO applies
}
