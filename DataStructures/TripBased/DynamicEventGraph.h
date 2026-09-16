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

#include "../../Helpers/Assert.h"
#include "../../Helpers/Helpers.h"
#include "../../Helpers/IO/Serialization.h"

namespace TripBased {

// a PackedTransfer (i.e., a uint32_t) is split as follows:
// the next 24 bits are TripID (global TripID)
// the last 8 bits are Position

using TransferVector = std::vector<PackedTransfer>;

class DynamicEventGraph {
public:
    DynamicEventGraph() = default;

    // just clear everything
    inline void clear() noexcept {
        toAdjs.clear();
        transfers.clear();
        numEdges = 0;
    }

    inline std::vector<std::vector<size_t>>& getToAdjs() noexcept { return toAdjs; }

    inline std::vector<TransferVector>& getTransfers() noexcept { return transfers; }

    inline size_t& getNumEdges() noexcept { return numEdges; }

    inline size_t numberOfEdges() const noexcept { return numEdges; }

    inline bool isVertex(TripId trip, StopIndex position) const noexcept {
        return trip < toAdjs.size() && position < toAdjs[trip].size() - 1;
    }

    inline bool isEdge(TripId trip, Edge edge) const noexcept {
        return trip < toAdjs.size() && edge < transfers[trip].size();
    }

    inline size_t beginEdgeFrom(const TripId trip, const StopIndex position) const {
        // note: position can be valid, but also +1
        AssertMsg(isVertex(trip, position) || position == toAdjs[trip].size() - 1,
                  "Trip and / or position is not valid");

        return toAdjs[trip][position];
    }

    inline auto& getToAdjsOfTrip(const TripId trip) {
        AssertMsg(trip < transfers.size(), "Trip is not valid");

        return toAdjs[trip];
    }

    inline auto& getTransfersOfTrip(const TripId trip) {
        AssertMsg(trip < transfers.size(), "Trip is not valid");

        return transfers[trip];
    }

    inline PackedTransfer getTransfer(TripId trip, Edge transferId) const {
        AssertMsg(isEdge(trip, transferId), "Trip and or transferId is not valid");
        return transfers[trip][transferId];
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, toAdjs, transfers, numEdges);
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, toAdjs, transfers, numEdges);
    }

private:
    // maps *per Trip* an event to its start position in the transfers /
    // travelTimes vector
    std::vector<std::vector<size_t>> toAdjs;

    // contains *per Trip* this "packed" transfer
    std::vector<TransferVector> transfers;

    // holds the total number of edges
    size_t numEdges{0};
};

}  // namespace TripBased
