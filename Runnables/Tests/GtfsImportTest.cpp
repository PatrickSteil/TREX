// Repros for GTFS import: frequency end_time is exclusive, blank stop times are
// interpolated, in-seat transfer types (4, 5) are not footpaths, and
// validate() keeps the trips sorted after removing dominated duplicates.
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "../../DataStructures/Intermediate/Data.h"

namespace fs = std::filesystem;
static int failures = 0;

#define CHECK(cond, msg)                                         \
  do {                                                           \
    if (!(cond)) {                                               \
      std::cout << "FAIL: " << msg << std::endl;                 \
      ++failures;                                                \
    }                                                            \
  } while (0)

static void write(const fs::path& file, const std::string& content) {
  std::ofstream(file) << content;
}

// Three stops A, B, C far apart, one route, one service on 2024-01-01.
static Intermediate::Data load(const std::string& name,
                               const std::string& stopTimes,
                               const std::string& transfers = "",
                               const std::string& frequencies = "") {
  const fs::path dir = fs::temp_directory_path() / ("trex_test_" + name);
  fs::create_directories(dir);
  write(dir / "agency.txt",
        "agency_id,agency_name,agency_timezone\na,A,Europe/Berlin\n");
  write(dir / "calendar.txt",
        "service_id,sunday,monday,tuesday,wednesday,thursday,friday,saturday,"
        "start_date,end_date\ns,1,1,1,1,1,1,1,20240101,20240101\n");
  write(dir / "routes.txt",
        "route_id,agency_id,route_short_name,route_long_name,route_type,"
        "route_color,route_text_color\nr,a,R,Route,3,,\n");
  write(dir / "stops.txt",
        "stop_id,stop_name,stop_lat,stop_lon\nA,A,49.00,8.00\nB,B,49.05,8.00\n"
        "C,C,49.10,8.00\n");
  write(dir / "trips.txt",
        "route_id,service_id,trip_id,trip_short_name\nr,s,t1,t1\n");
  write(dir / "stop_times.txt",
        "trip_id,arrival_time,departure_time,stop_id,stop_sequence\n" +
            stopTimes);
  if (!transfers.empty())
    write(dir / "transfers.txt",
          "from_stop_id,to_stop_id,min_transfer_time,transfer_type\n" +
              transfers);
  if (!frequencies.empty())
    write(dir / "frequencies.txt",
          "trip_id,start_time,end_time,headway_secs\n" + frequencies);
  GTFS::Data gtfs = GTFS::Data::FromGTFS(dir.string() + "/", false);
  const int day = stringToDay("20240101");
  return Intermediate::Data::FromGTFS(gtfs, day, day);
}

int main() {
  const std::string plain =
      "t1,08:00:00,08:00:00,A,1\nt1,08:10:00,08:10:00,B,2\n";

  {  // 1. end_time is exclusive: 08:00 and 08:30 only, not 09:00.
    auto d = load("freq", plain, "", "t1,08:00:00,09:00:00,1800\n");
    CHECK(d.numberOfTrips() == 2,
          "frequency trips: got " << d.numberOfTrips() << ", expected 2");
  }
  {  // 2. blank times at B are interpolated, B stays in the trip.
    auto d = load("interp",
                  "t1,08:00:00,08:00:00,A,1\nt1,,,B,2\n"
                  "t1,08:20:00,08:20:00,C,3\n");
    CHECK(d.numberOfTrips() == 1, "interp: expected 1 trip");
    if (d.numberOfTrips() == 1) {
      const auto& se = d.trips[0].stopEvents;
      CHECK(se.size() == 3, "interp: stop events " << se.size() << " != 3");
      if (se.size() == 3) {
        const int expected = 8 * 3600 + 600;
        CHECK(se[1].arrivalTime == expected && se[1].departureTime == expected,
              "interp: B at " << se[1].arrivalTime << "/" << se[1].departureTime
                              << ", expected " << expected);
      }
    }
  }
  {  // 3. type 0 is a footpath, types 4 and 5 are not.
    auto d = load("transfers",
                  "t1,08:00:00,08:00:00,A,1\nt1,08:10:00,08:10:00,B,2\n"
                  "t1,08:20:00,08:20:00,C,3\n",
                  "A,B,60,0\nB,C,60,4\nA,C,60,5\n");
    CHECK(d.transferGraph.numEdges() == 2,
          "transfers: " << d.transferGraph.numEdges() << " edges, expected 2");
  }
  {  // 4. validate() keeps trips sorted when removing a dominated trip.
    using namespace Intermediate;
    Data d;
    const Geometry::Point p[3] = {
        Geometry::Point(Construct::LatLong, 49.0, 8.0),
        Geometry::Point(Construct::LatLong, 49.1, 8.0),
        Geometry::Point(Construct::LatLong, 49.2, 8.0)};
    d.transferGraph.addVertices(3);
    for (int i = 0; i < 3; i++) {
      d.stops.emplace_back("s", p[i]);
      d.transferGraph.set(Coordinates, Vertex(i), p[i]);
    }
    auto trip = [&](int to, int arr0, int dep0, int arr1, int dep1) {
      d.trips.emplace_back("t", "r", 3);
      d.trips.back().stopEvents.emplace_back(StopId(0), arr0, dep0);
      d.trips.back().stopEvents.emplace_back(StopId(to), arr1, dep1);
    };
    trip(1, 99, 100, 300, 301);  // dominated by the next one
    trip(1, 99, 110, 290, 301);
    trip(2, 99, 100, 300, 301);
    d.validate();
    CHECK(d.numberOfTrips() == 2, "sorted: " << d.numberOfTrips() << " trips");
    CHECK(std::is_sorted(d.trips.begin(), d.trips.end()),
          "sorted: trips are not sorted after validate()");
  }
  {  // 5. GTFS allows single-digit hours ("8:00:00").
    auto d = load("shorttime", "t1,8:00:00,8:00:00,A,1\nt1,8:10:00,8:10:00,B,2\n");
    CHECK(d.numberOfTrips() == 1 &&
              d.trips[0].stopEvents.back().arrivalTime == 8 * 3600 + 600,
          "shorttime: wrong trip");
  }
  std::cout << (failures ? "FAILED" : "ok") << std::endl;
  return failures ? 1 : 0;
}
