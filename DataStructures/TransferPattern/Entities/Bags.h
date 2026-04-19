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
#include "../../Container/ExternalKHeap.h"

namespace TransferPattern {

template <typename DIJKSTRA_LABEL>
class DijkstraBag : public ExternalKHeapElement {
public:
    using DijkstraLabel = DIJKSTRA_LABEL;
    static constexpr int logK = 1;
    static constexpr int K = 1 << logK;

    DijkstraBag() : labels(), nonHeapLabels() {}

    // Copy Assignment Operator
    DijkstraBag& operator=(const DijkstraBag& other) {
        if (this != &other) {
            labels = other.labels;
            nonHeapLabels = other.nonHeapLabels;
            // If ExternalKHeapElement has any members to copy, ensure they are copied
            // here.
        }
        return *this;
    }

    template <typename OTHER_LABEL>
    inline bool dominates(const OTHER_LABEL& newLabel) const noexcept {
        for (const DijkstraLabel& label : labels) {
            if (label.dominates(newLabel)) return true;
        }
        for (const DijkstraLabel& label : nonHeapLabels) {
            if (label.dominates(newLabel)) return true;
        }
        return false;
    }

    inline size_t size() const noexcept { return labels.size(); }

    inline size_t nonHeapSize() const noexcept { return nonHeapLabels.size(); }

    inline bool empty() const noexcept { return labels.empty(); }

    inline bool heapEmpty() const noexcept { return labels.size() == 0; }

    inline const DijkstraLabel& front() const noexcept {
        AssertMsg(!empty(), "An empty heap has no front!");
        return labels[0];
    }

    inline int getKey() const noexcept { return front().getKey(); }

    inline bool hasSmallerKey(const DijkstraBag* const other) const noexcept { return getKey() < other->getKey(); }

    inline const DijkstraLabel& extractFront() noexcept {
        AssertMsg(!empty(), "An empty heap has no front!");

        nonHeapLabels.push_back(labels[0]);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
        labels[0] = labels.back();
#pragma GCC diagnostic pop
        labels.pop_back();
        // fix heap prop
        if (labels.size() > 1) siftDown(0);
        return nonHeapLabels.back();
    }

    inline bool merge(const DijkstraLabel& newLabel) noexcept {
        size_t removedLabels = 0;
        for (size_t i = 0; i < labels.size(); i++) {
            if (labels[i].dominates(newLabel)) return false;
            if (newLabel.dominates(labels[i])) {
                ++removedLabels;
                continue;
            }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
            labels[i - removedLabels] = labels[i];
#pragma GCC diagnostic pop
        }
        for (size_t i = 0; i < nonHeapLabels.size(); ++i) {
            if (nonHeapLabels[i].dominates(newLabel)) return false;
        }
        labels.resize(labels.size() - removedLabels + 1);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
        labels.back() = newLabel;
#pragma GCC diagnostic pop
        heapify();
        return true;
    }

    inline void initialize(const DijkstraLabel& label) noexcept {
        AssertMsg(labels.empty(), "Trying to initialize non-empty bag!");
        labels.emplace_back(label);
    }

    inline DijkstraLabel& access(const size_t index) {
        AssertMsg(index < nonHeapLabels.size(), "Index out of bounds");
        return nonHeapLabels[index];
    }

    inline size_t getIndex(const DijkstraLabel& label) { return (size_t)(&label - &nonHeapLabels[0]); }

private:
    inline void heapify() noexcept {
        if (labels.size() <= 1) return;
        for (size_t i = parent(labels.size() - 1); i != size_t(-1); --i) {
            siftDown(i);
        }
    }

    inline void siftDown(size_t i) noexcept {
        AssertMsg(i < labels.size(), "siftDown index out of range!");
        while (true) {
            size_t minIndex = i;
            const size_t childrenStart = firstChild(i);
            const size_t childrenEnd = std::min(childrenStart + K, labels.size());
            for (size_t j = childrenStart; j < childrenEnd; j++) {
                if (labels[j].hasSmallerKey(&labels[minIndex])) {
                    minIndex = j;
                }
            }
            if (minIndex == i) break;
            std::swap(labels[i], labels[minIndex]);
            i = minIndex;
        }
    }

    inline size_t parent(const size_t i) const noexcept { return (i - 1) >> logK; }

    inline size_t firstChild(const size_t i) const noexcept { return (i << logK) + 1; }

    friend std::ostream& operator<<(std::ostream& out, const DijkstraBag& r) {
        out << "Heap Labels:\n";
        for (auto& l : r.labels) out << l << "\n";
        out << "Non Heap Labels:\n";
        for (auto& l : r.nonHeapLabels) out << l << "\n";
        return out;
    }

    std::vector<DijkstraLabel> labels;
    std::vector<DijkstraLabel> nonHeapLabels;
};

}  // namespace TransferPattern
