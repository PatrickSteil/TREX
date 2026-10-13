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

namespace PTL {

class Data {
public:
    Data(){};

    Data(TE::Data& teData)
        : teData(teData), fwdVertices(teData.numberOfTEVertices()), bwdVertices(teData.numberOfTEVertices()){};

    Data(TE::Data& teData, const std::string fileName)
        : teData(teData), fwdVertices(teData.numberOfTEVertices()), bwdVertices(teData.numberOfTEVertices()) {
        readViennotLabels(fileName);
        sortLabels();
    };

    inline static Data FromBinary(const std::string& fileName) noexcept {
        Data data;
        data.deserialize(fileName);
        return data;
    }

    inline bool readViennotLabels(const std::string& fileName) {
        std::cout << "Reading Viennot Labels from " << fileName << " ... " << std::endl;
        std::ifstream file;
        file.open(fileName);

        if (!file.is_open()) {
            std::cerr << "Error: Could not open the file." << std::endl;
            return 1;
        }

        size_t n = teData.numberOfTEVertices();

        fwdVertices.resize(n, {});
        bwdVertices.resize(n, {});

        std::string line;
        char type;
        size_t vertex, hub, dummyDist;

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            iss >> type >> vertex >> hub >> dummyDist;

            --vertex;
            --hub;

            AssertMsg(vertex < n, "Vertex is out of bounds!");
            AssertMsg(hub < n, "Hub is out of bounds!");

            switch (type) {
                case 'o':
                    fwdVertices[vertex].push_back(Vertex(hub));
                    break;
                case 'i':
                    bwdVertices[hub].push_back(Vertex(vertex));
                    break;
                default:
                    std::cerr << "Warning: Unknown label type '" << type << "' in line: " << line << std::endl;
                    break;
            }
        }

        file >> std::ws;
        file.close();
        std::cout << "Done!" << std::endl;
        return file.eof() && !file.fail();
    }

    inline bool readSavrusLabels(const std::string& fileName) {
        std::cout << "Reading Savrus Labels from " << fileName << " ... " << std::endl;
        std::ifstream file;
        file.open(fileName);

        if (!file.is_open()) {
            std::cerr << "Error: Could not open the file." << std::endl;
            return 1;
        }

        size_t n;
        file >> n;

        if (n != teData.numberOfTEVertices()) {
            std::cout << "Wrong number of entries!" << std::endl;
            return false;
        }

        fwdVertices.resize(n, {});
        bwdVertices.resize(n, {});

        int dummyDist;

        for (Vertex v = Vertex(0); v < n; ++v) {
            size_t s;

            // bwd
            file >> s;
            bwdVertices[v].resize(s);
            for (size_t i = 0; i < s; ++i) {
                file >> bwdVertices[v][i];
                file >> dummyDist;
            }

            // fwd
            file >> s;
            fwdVertices[v].resize(s);
            for (size_t i = 0; i < s; ++i) {
                file >> fwdVertices[v][i];
                file >> dummyDist;
            }
        }
        file >> std::ws;
        file.close();
        std::cout << "Done!" << std::endl;
        return file.eof() && !file.fail();
    }

    inline void clear() noexcept {
        AssertMsg(fwdVertices.size() == (teData.numberOfTEVertices()), "Not the same size!");
        AssertMsg(bwdVertices.size() == (teData.numberOfTEVertices()), "Not the same size!");

        for (Vertex v = Vertex(0); v < (teData.numberOfTEVertices()); ++v) {
            fwdVertices.clear();
            bwdVertices.clear();
        }
    }

    inline void sortLabels() noexcept {
        AssertMsg(fwdVertices.size() == (teData.numberOfTEVertices()), "Not the same size!");
        AssertMsg(bwdVertices.size() == (teData.numberOfTEVertices()), "Not the same size!");

        for (Vertex v = Vertex(0); v < (teData.numberOfTEVertices()); ++v) {
            std::sort(fwdVertices[v].begin(), fwdVertices[v].end());
            std::sort(bwdVertices[v].begin(), bwdVertices[v].end());
        }
    }

    inline size_t numberOfStops() const noexcept { return teData.numberOfStops(); }
    inline bool isStop(const StopId stop) const noexcept { return stop < numberOfStops(); }
    inline Range<StopId> stops() const noexcept { return Range<StopId>(StopId(0), StopId(numberOfStops())); }

    inline size_t numberOfTrips() const noexcept { return teData.numTrips; }
    inline bool isTrip(const TripId route) const noexcept { return route < numberOfTrips(); }
    inline Range<TripId> trips() const noexcept { return Range<TripId>(TripId(0), TripId(numberOfTrips())); }

    inline size_t numberOfStopEvents() const noexcept { return teData.events.size(); }

    inline bool isEvent(const Vertex event) const noexcept { return teData.isEvent(event); }
    inline bool isDepartureEvent(const Vertex event) const noexcept { return teData.isDepartureEvent(event); }
    inline bool isArrivalEvent(const Vertex event) const noexcept { return teData.isArrivalEvent(event); }

    inline void printInfo() const noexcept {
        size_t minSizeFWD = teData.numberOfTEVertices();
        size_t maxSizeFWD = 0;
        size_t totalSizeFWD = 0;
        Vertex maxFwdVertex = noVertex;

        size_t minSizeBWD = teData.numberOfTEVertices();
        size_t maxSizeBWD = 0;
        size_t totalSizeBWD = 0;
        Vertex maxBwdVertex = noVertex;

        AssertMsg(fwdVertices.size() == (teData.numberOfTEVertices()), "Not the same size!");
        AssertMsg(bwdVertices.size() == (teData.numberOfTEVertices()), "Not the same size!");

        for (Vertex v = Vertex(0); v < teData.numberOfTEVertices(); ++v) {
            minSizeFWD = std::min(minSizeFWD, fwdVertices[v].size());
            maxSizeFWD = std::max(maxSizeFWD, fwdVertices[v].size());

            if (maxSizeFWD == fwdVertices[v].size()) {
                maxFwdVertex = v;
            }

            totalSizeFWD += fwdVertices[v].size();

            minSizeBWD = std::min(minSizeBWD, bwdVertices[v].size());
            maxSizeBWD = std::max(maxSizeBWD, bwdVertices[v].size());

            if (maxSizeBWD == bwdVertices[v].size()) {
                maxBwdVertex = v;
            }
            totalSizeBWD += bwdVertices[v].size();
        }

        std::cout << "PTL public transit data:" << std::endl;
        std::cout << "   Number of Stops:           " << std::setw(12) << String::prettyInt(teData.numberOfStops())
                  << std::endl;
        std::cout << "   Number of Trips:           " << std::setw(12) << String::prettyInt(teData.numberOfTrips())
                  << std::endl;
        std::cout << "   Number of TE Vertices:     " << std::setw(12)
                  << String::prettyInt(teData.timeExpandedGraph.numVertices()) << std::endl;
        std::cout << "   Number of TE Edges:        " << std::setw(12)
                  << String::prettyInt(teData.timeExpandedGraph.numEdges()) << std::endl;
        std::cout << "   Forward Labels:" << std::endl;
        std::cout << "      Min # of hubs:          " << std::setw(12) << String::prettyInt(minSizeFWD) << std::endl;
        std::cout << "      Avg # of hubs:          " << std::setw(12)
                  << String::prettyDouble(static_cast<double>(totalSizeFWD) / (teData.numberOfTEVertices()))
                  << std::endl;
        std::cout << "      Max # of hubs:          " << std::setw(12) << String::prettyInt(maxSizeFWD) << std::endl;
        std::cout << "      Max Vertex:             " << std::setw(12) << maxFwdVertex << std::endl;
        std::cout << "   Backward Labels:" << std::endl;
        std::cout << "      Min # of hubs:          " << std::setw(12) << String::prettyInt(minSizeBWD) << std::endl;
        std::cout << "      Avg # of hubs:          " << std::setw(12)
                  << String::prettyDouble(static_cast<double>(totalSizeBWD) / (teData.numberOfTEVertices()))
                  << std::endl;
        std::cout << "      Max # of hubs:          " << std::setw(12) << String::prettyInt(maxSizeBWD) << std::endl;
        std::cout << "      Max Vertex:             " << std::setw(12) << maxBwdVertex << std::endl;
        std::cout << "   Total size:                " << std::setw(12) << String::bytesToString(byteSize())
                  << std::endl;
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, fwdVertices, bwdVertices);
        teData.serialize(fileName + ".te");
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, fwdVertices, bwdVertices);
        teData.deserialize(fileName + ".te");
    }

    inline long long byteSize() const noexcept {
        long long result = Vector::byteSize(fwdVertices);
        result += Vector::byteSize(bwdVertices);
        result += teData.byteSize();
        return result;
    }

    inline std::vector<Vertex>& getFwdHubs(const Vertex vertex) noexcept {
        AssertMsg(teData.isEvent(vertex), "Vertex is not valid!");

        return fwdVertices[vertex];
    }

    inline std::vector<Vertex>& getBwdHubs(const Vertex vertex) noexcept {
        AssertMsg(teData.isEvent(vertex), "Vertex is not valid!");

        return bwdVertices[vertex];
    }

    TE::Data teData;
    std::vector<std::vector<Vertex>> fwdVertices;
    std::vector<std::vector<Vertex>> bwdVertices;
};
}  // namespace PTL
