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

#include <algorithm>
#include <vector>

#include "../TE/Data.h"

namespace PPTL {
/// Packs (path_id, path_pos) into a single 32-bit integer.
/// The upper 16 bits store path_id, the lower 16 bits store path_pos.
inline std::uint32_t packPathHub(std::uint16_t path_id,
                                 std::uint16_t path_pos) noexcept {
  return (static_cast<std::uint32_t>(path_id) << 16) |
         static_cast<std::uint32_t>(path_pos);
}

/// Extracts the path ID (upper 16 bits) from a packed 32-bit hub.
inline std::uint16_t extractPathId(std::uint32_t packed) noexcept {
  return static_cast<std::uint16_t>(packed >> 16);
}

/// Extracts the path position (lower 16 bits) from a packed 32-bit hub.
inline std::uint16_t extractPathPos(std::uint32_t packed) noexcept {
  return static_cast<std::uint16_t>(packed & 0xFFFF);
}

class Data {
 public:
  Data(){};

  Data(TE::Data &teData)
      : teData(teData),
        fwdHubs(teData.numberOfTEVertices()),
        bwdHubs(teData.numberOfTEVertices()){};

  Data(TE::Data &teData, const std::string fileName)
      : teData(teData),
        fwdHubs(teData.numberOfTEVertices()),
        bwdHubs(teData.numberOfTEVertices()) {
    loadPathHub(fileName);
    sortLabels();
  };

  inline static Data FromBinary(const std::string &fileName) noexcept {
    Data data;
    data.deserialize(fileName);
    return data;
  }

  inline bool loadPathHub(const std::string &fileName) {
    std::cout << "Reading Path Hubs from " << fileName << " ... " << std::endl;
    std::ifstream file(fileName);

    if (!file.is_open()) {
      std::cerr << "Error: Could not open the file." << std::endl;
      return false;
    }

    std::string line;
    char type;

    size_t numVertices = 0;
    if (!std::getline(file, line)) {
      std::cerr << "Error: Empty file or missing vertex count." << std::endl;
      return false;
    }

    {
      std::istringstream iss(line);
      char header;
      iss >> header >> numVertices;
      if (header != 'V') {
        std::cerr << "Error: First line must start with 'V'." << std::endl;
        return false;
      }
    }

    fwdHubs.assign(numVertices, {});
    bwdHubs.assign(numVertices, {});

    while (std::getline(file, line)) {
      if (line.empty()) continue;

      std::istringstream iss(line);
      iss >> type;

      if (type != 'o' && type != 'i') {
        std::cerr << "Warning: Unknown line type '" << type
                  << "' in line: " << line << std::endl;
        return false;
      }

      size_t vertex;
      iss >> vertex;
      if (vertex >= numVertices) {
        std::cerr << "Warning: Vertex " << vertex << " is out of bounds (max "
                  << numVertices - 1 << ")" << std::endl;
        return false;
      }

      std::vector<std::uint32_t> packedHubs;
      size_t pathId, pathPos;

      while (iss >> pathId >> pathPos) {
        if (pathId >= (1u << 16) || pathPos >= (1u << 16)) {
          std::cerr << "Warning: pathId or pathPos exceeds 16-bit range."
                    << std::endl;
          return false;
        }
        packedHubs.push_back(packPathHub(pathId, pathPos));
      }

      if (type == 'o')
        fwdHubs[vertex] = std::move(packedHubs);
      else
        bwdHubs[vertex] = std::move(packedHubs);
    }

    file.close();
    std::cout << "Done loading Path Hubs (" << numVertices << " vertices)"
              << std::endl;
    return true;
  }

  inline void clear() noexcept {
    AssertMsg(fwdHubs.size() == (teData.numberOfTEVertices()),
              "Not the same size!");
    AssertMsg(bwdHubs.size() == (teData.numberOfTEVertices()),
              "Not the same size!");

    for (Vertex v = Vertex(0); v < (teData.numberOfTEVertices()); ++v) {
      fwdHubs.clear();
      bwdHubs.clear();
    }
  }

  inline void sortLabels() noexcept {
    AssertMsg(fwdHubs.size() == (teData.numberOfTEVertices()),
              "Not the same size!");
    AssertMsg(bwdHubs.size() == (teData.numberOfTEVertices()),
              "Not the same size!");

    for (Vertex v = Vertex(0); v < (teData.numberOfTEVertices()); ++v) {
      std::sort(fwdHubs[v].begin(), fwdHubs[v].end());
      std::sort(bwdHubs[v].begin(), bwdHubs[v].end());
    }
  }

  inline size_t numberOfStops() const noexcept {
    return teData.numberOfStops();
  }
  inline bool isStop(const StopId stop) const noexcept {
    return stop < numberOfStops();
  }
  inline Range<StopId> stops() const noexcept {
    return Range<StopId>(StopId(0), StopId(numberOfStops()));
  }

  inline size_t numberOfTrips() const noexcept { return teData.numTrips; }
  inline bool isTrip(const TripId route) const noexcept {
    return route < numberOfTrips();
  }
  inline Range<TripId> trips() const noexcept {
    return Range<TripId>(TripId(0), TripId(numberOfTrips()));
  }

  inline size_t numberOfStopEvents() const noexcept {
    return teData.events.size();
  }

  inline bool isEvent(const Vertex event) const noexcept {
    return teData.isEvent(event);
  }
  inline bool isDepartureEvent(const Vertex event) const noexcept {
    return teData.isDepartureEvent(event);
  }
  inline bool isArrivalEvent(const Vertex event) const noexcept {
    return teData.isArrivalEvent(event);
  }

  inline void printInfo() const noexcept {
    size_t minSizeFWD = teData.numberOfTEVertices();
    size_t maxSizeFWD = 0;
    size_t totalSizeFWD = 0;
    Vertex maxFwdVertex = noVertex;

    size_t minSizeBWD = teData.numberOfTEVertices();
    size_t maxSizeBWD = 0;
    size_t totalSizeBWD = 0;
    Vertex maxBwdVertex = noVertex;

    AssertMsg(fwdHubs.size() == (teData.numberOfTEVertices()),
              "Not the same size!");
    AssertMsg(bwdHubs.size() == (teData.numberOfTEVertices()),
              "Not the same size!");

    for (Vertex v = Vertex(0); v < teData.numberOfTEVertices(); ++v) {
      minSizeFWD = std::min(minSizeFWD, fwdHubs[v].size());
      maxSizeFWD = std::max(maxSizeFWD, fwdHubs[v].size());

      if (maxSizeFWD == fwdHubs[v].size()) {
        maxFwdVertex = v;
      }

      totalSizeFWD += fwdHubs[v].size();

      minSizeBWD = std::min(minSizeBWD, bwdHubs[v].size());
      maxSizeBWD = std::max(maxSizeBWD, bwdHubs[v].size());

      if (maxSizeBWD == bwdHubs[v].size()) {
        maxBwdVertex = v;
      }
      totalSizeBWD += bwdHubs[v].size();
    }

    std::cout << "PPTL public transit data:" << std::endl;
    std::cout << "   Number of Stops:           " << std::setw(12)
              << String::prettyInt(teData.numberOfStops()) << std::endl;
    std::cout << "   Number of Trips:           " << std::setw(12)
              << String::prettyInt(teData.numberOfTrips()) << std::endl;
    std::cout << "   Number of TE Vertices:     " << std::setw(12)
              << String::prettyInt(teData.timeExpandedGraph.numVertices())
              << std::endl;
    std::cout << "   Number of TE Edges:        " << std::setw(12)
              << String::prettyInt(teData.timeExpandedGraph.numEdges())
              << std::endl;
    std::cout << "   Forward Labels:" << std::endl;
    std::cout << "      Min # of hubs:          " << std::setw(12)
              << String::prettyInt(minSizeFWD) << std::endl;
    std::cout << "      Avg # of hubs:          " << std::setw(12)
              << String::prettyDouble(static_cast<double>(totalSizeFWD) /
                                      (teData.numberOfTEVertices()))
              << std::endl;
    std::cout << "      Max # of hubs:          " << std::setw(12)
              << String::prettyInt(maxSizeFWD) << std::endl;
    std::cout << "      Max Vertex:             " << std::setw(12)
              << maxFwdVertex << std::endl;
    std::cout << "   Backward Labels:" << std::endl;
    std::cout << "      Min # of hubs:          " << std::setw(12)
              << String::prettyInt(minSizeBWD) << std::endl;
    std::cout << "      Avg # of hubs:          " << std::setw(12)
              << String::prettyDouble(static_cast<double>(totalSizeBWD) /
                                      (teData.numberOfTEVertices()))
              << std::endl;
    std::cout << "      Max # of hubs:          " << std::setw(12)
              << String::prettyInt(maxSizeBWD) << std::endl;
    std::cout << "      Max Vertex:             " << std::setw(12)
              << maxBwdVertex << std::endl;
    std::cout << "   Total size:                " << std::setw(12)
              << String::bytesToString(byteSize()) << std::endl;
  }

  inline void serialize(const std::string &fileName) const noexcept {
    IO::serialize(fileName, fwdHubs, bwdHubs);
    teData.serialize(fileName + ".te");
  }

  inline void deserialize(const std::string &fileName) noexcept {
    IO::deserialize(fileName, fwdHubs, bwdHubs);
    teData.deserialize(fileName + ".te");
  }

  inline long long byteSize() const noexcept {
    long long result = Vector::byteSize(fwdHubs);
    result += Vector::byteSize(bwdHubs);
    result += teData.byteSize();
    return result;
  }

  inline std::vector<std::uint32_t> &getFwdHubs(const Vertex vertex) noexcept {
    AssertMsg(teData.isEvent(vertex), "Vertex is not valid!");

    return fwdHubs[vertex];
  }

  inline std::vector<std::uint32_t> &getBwdHubs(const Vertex vertex) noexcept {
    AssertMsg(teData.isEvent(vertex), "Vertex is not valid!");

    return bwdHubs[vertex];
  }

  TE::Data teData;
  std::vector<std::vector<std::uint32_t>> fwdHubs;
  std::vector<std::vector<std::uint32_t>> bwdHubs;
};
}  // namespace PPTL
