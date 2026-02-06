#pragma once

#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/TimeTable/Data.h"
#include <vector>

namespace TimeTable {

FlatStopEventGraph FlattenTimeTable(const Data &data) {
  std::vector<std::size_t> nrEventsPerRoute(data.numberOfRoutes(), 0);

#pragma omp parallel for
  for (std::size_t i = 0; i < data.routes.size(); ++i) {
    nrEventsPerRoute[i] = data.routes[i].size();
    nrEventsPerRoute[i] *=
        (data.routes[i].size() > 0 ? data.routes[i].front().numberOfEvents()
                                   : 0);
  }
}
} // namespace TimeTable
