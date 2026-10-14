// Tests for Intermediate::Data::splitLongTrips and the `splitTrips` flag of
// Intermediate::Data::FromGTFS: trips with more than 255 stop events (as
// produced by nigiri unrolling block ids) are cut into pieces of at most 255
// stop events, preferably where the vehicle is back at its first stop.
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include "../../DataStructures/Intermediate/Data.h"

namespace fs = std::filesystem;
static int failures = 0;

#define CHECK(cond, msg)                         \
  do {                                           \
    if (!(cond)) {                               \
      std::cout << "FAIL: " << msg << std::endl; \
      ++failures;                                \
    }                                            \
  } while (0)

using Intermediate::Data;
using Intermediate::Trip;

// Stop i lies at (lat(i), lon(i)); the caller picks positions that are far
// apart (> 1 km) or deliberately close (< 85 m).
template <typename LAT, typename LON>
static Data makeData(const size_t numStops, LAT lat, LON lon) {
  Data d;
  d.transferGraph.addVertices(numStops);
  for (size_t i = 0; i < numStops; i++) {
    const Geometry::Point p(Construct::LatLong, lat(i), lon(i));
    d.stops.emplace_back("s" + std::to_string(i), p);
    d.transferGraph.set(Coordinates, Vertex(i), p);
  }
  return d;
}

static Data farApart(const size_t numStops) {
  return makeData(
      numStops, [](size_t i) { return 40.0 + 0.01 * (i % 100); },
      [](size_t i) { return 8.0 + 0.02 * (i / 100); });
}

// Adds a trip that visits `sequence` with one minute between stop events.
static void addTrip(Data& d, const std::vector<size_t>& sequence,
                    const int start = 8 * 3600) {
  d.trips.emplace_back("t", "r", 3);
  int time = start;
  for (const size_t stop : sequence) {
    d.trips.back().stopEvents.emplace_back(StopId(stop), time, time + 10);
    time += 60;
  }
}

static std::vector<size_t> stopsOf(const Trip& t) {
  std::vector<size_t> result;
  for (const auto& se : t.stopEvents) result.push_back(size_t(se.stopId));
  return result;
}

// Block of `runs` runs of the loop 0 -> 1 -> ... -> length-1.
static std::vector<size_t> loops(const size_t length, const size_t runs) {
  std::vector<size_t> result;
  for (size_t r = 0; r < runs; r++)
    for (size_t i = 0; i < length; i++) result.push_back(i);
  return result;
}

// All pieces of a split must be in order, adjacent, and cover the input.
static std::vector<size_t> concatenated(const Data& d,
                                        const std::vector<size_t>& original) {
  // Pieces are sorted by stop sequence by splitLongTrips, so find the one that
  // starts the original and chain the rest by their first stop events' times.
  std::vector<const Trip*> pieces;
  for (const Trip& t : d.trips) pieces.push_back(&t);
  std::sort(pieces.begin(), pieces.end(), [](const Trip* a, const Trip* b) {
    return a->stopEvents.front().departureTime <
           b->stopEvents.front().departureTime;
  });
  std::vector<size_t> result;
  for (const Trip* t : pieces) {
    const auto s = stopsOf(*t);
    result.insert(result.end(), s.begin(), s.end());
  }
  (void)original;
  return result;
}

static size_t maxLength(const Data& d) {
  size_t m = 0;
  for (const Trip& t : d.trips) m = std::max(m, t.stopEvents.size());
  return m;
}

static void write(const fs::path& file, const std::string& content) {
  std::ofstream(file) << content;
}

static std::string hms(const int seconds) {
  std::ostringstream ss;
  ss << seconds / 3600 << ":" << (seconds / 60) % 60 / 10 << (seconds / 60) % 10
     << ":00";
  return ss.str();
}

// One trip, 300 stop events: 10 far-apart stops, driven 30 times in a row.
static Data loadGtfs(const bool split) {
  const fs::path dir = fs::temp_directory_path() / "trex_test_split";
  fs::create_directories(dir);
  write(dir / "agency.txt",
        "agency_id,agency_name,agency_timezone\na,A,Europe/Berlin\n");
  write(dir / "calendar.txt",
        "service_id,sunday,monday,tuesday,wednesday,thursday,friday,saturday,"
        "start_date,end_date\ns,1,1,1,1,1,1,1,20240101,20240101\n");
  write(dir / "routes.txt",
        "route_id,agency_id,route_short_name,route_long_name,route_type,"
        "route_color,route_text_color\nr,a,R,Route,3,,\n");
  std::string stops = "stop_id,stop_name,stop_lat,stop_lon\n";
  for (int i = 0; i < 10; i++)
    stops += "S" + std::to_string(i) + ",S" + std::to_string(i) + "," +
             std::to_string(49.0 + 0.02 * i) + ",8.0\n";
  write(dir / "stops.txt", stops);
  write(dir / "trips.txt",
        "route_id,service_id,trip_id,trip_short_name\nr,s,t1,t1\n");
  std::string times = "trip_id,arrival_time,departure_time,stop_id,stop_sequence\n";
  for (int i = 0; i < 300; i++) {
    const std::string t = hms(8 * 3600 + 60 * i);
    times += "t1," + t + "," + t + ",S" + std::to_string(i % 10) + "," +
             std::to_string(i + 1) + "\n";
  }
  write(dir / "stop_times.txt", times);
  GTFS::Data gtfs = GTFS::Data::FromGTFS(dir.string() + "/", false);
  const int day = stringToDay("20240101");
  return Data::FromGTFS(gtfs, day, day, false, false, split);
}

int main() {
  {  // 1. Loops of 4 stops, 100 runs: cut only at run boundaries.
    Data d = farApart(4);
    const auto original = loops(4, 100);
    addTrip(d, original);
    CHECK(d.warnAboutLongTrips() == 1, "warn: expected one long trip");
    CHECK(d.splitLongTrips() == 1, "loops: expected one split trip");
    CHECK(d.numberOfTrips() == 2, "loops: " << d.numberOfTrips() << " pieces");
    CHECK(maxLength(d) <= 255, "loops: piece longer than 255");
    for (const Trip& t : d.trips)
      CHECK(t.stopEvents.front().stopId == StopId(0) &&
                t.stopEvents.size() % 4 == 0,
            "loops: piece does not consist of whole runs");
    CHECK(concatenated(d, original) == original,
          "loops: pieces do not add up to the original trip");
    CHECK(d.warnAboutLongTrips() == 0, "loops: still long trips afterwards");
  }
  {  // 2. 255 stop events are fine, 256 are not.
    Data d = farApart(5);
    addTrip(d, loops(5, 51));  // 255
    CHECK(d.splitLongTrips() == 0 && d.numberOfTrips() == 1,
          "255 events must stay untouched");
    Data e = farApart(4);
    addTrip(e, loops(4, 64));  // 256
    CHECK(e.splitLongTrips() == 1 && maxLength(e) <= 255,
          "256 events must be split");
  }
  {  // 3. Runs use fresh stops, but stop j of every run is next to stop j of the
     // first run (different tracks). Only neighbours make a loop out of it.
    const size_t length = 4, runs = 80;  // 320 events, 255 is no run boundary
    auto build = [&](const int neighbourSeconds) {
      Data d = makeData(
          length * runs,
          [&](size_t i) { return 40.0 + 0.0001 * (i / length); },
          [&](size_t i) { return 8.0 + 0.02 * (i % length); });
      std::vector<size_t> seq;
      for (size_t i = 0; i < length * runs; i++) seq.push_back(i);
      addTrip(d, seq);
      d.splitLongTrips(255, neighbourSeconds);
      return d;
    };
    Data with = build(60);
    CHECK(maxLength(with) <= 255, "neighbours: piece longer than 255");
    for (const Trip& t : with.trips)
      CHECK(size_t(t.stopEvents.front().stopId) % length == 0,
            "neighbours: piece starts in the middle of a run (stop "
                << size_t(t.stopEvents.front().stopId) << ")");
    Data without = build(0);
    bool midRun = false;
    for (const Trip& t : without.trips)
      midRun |= size_t(t.stopEvents.front().stopId) % length != 0;
    CHECK(midRun, "without neighbours the cut should not be at a run boundary");
  }
  {  // 4. A run boundary beats loop cuts: first stop 0 returns at 150, stop 1
     // occurs at 100 and 200, which would also give two pieces.
    Data d = farApart(300);
    std::vector<size_t> seq;
    for (size_t i = 0; i < 300; i++) seq.push_back(i + 2);
    seq[0] = 0;
    seq[150] = 0;
    seq[100] = 1;
    seq[200] = 1;
    addTrip(d, seq);
    d.splitLongTrips();
    CHECK(d.numberOfTrips() == 2, "boundary: " << d.numberOfTrips() << " pieces");
    for (const Trip& t : d.trips)
      CHECK(t.stopEvents.size() == 150, "boundary: piece of size "
                                            << t.stopEvents.size());
  }
  {  // 5. No place repeats: still split (hard cut), and nothing is lost.
    Data d = farApart(300);
    std::vector<size_t> seq;
    for (size_t i = 0; i < 300; i++) seq.push_back(i);
    addTrip(d, seq);
    d.splitLongTrips();
    CHECK(maxLength(d) <= 255 && d.numberOfTrips() == 2,
          "hard cut: " << d.numberOfTrips() << " pieces");
    for (const Trip& t : d.trips)
      CHECK(t.stopEvents.size() >= 2, "hard cut: piece with one stop event");
    CHECK(concatenated(d, seq) == seq, "hard cut: events lost or reordered");
  }
  {  // 6. Trips of one line are cut alike; times are kept except at the cuts.
    Data d = farApart(4);
    const auto seq = loops(4, 100);
    addTrip(d, seq, 8 * 3600);
    addTrip(d, seq, 9 * 3600);
    d.splitLongTrips();
    CHECK(d.numberOfTrips() == 4, "line: " << d.numberOfTrips() << " pieces");
    std::vector<size_t> sizes;
    for (const Trip& t : d.trips) sizes.push_back(t.stopEvents.size());
    std::sort(sizes.begin(), sizes.end());
    CHECK(sizes[0] == sizes[1] && sizes[2] == sizes[3],
          "line: trips of one line were cut differently");
    for (const Trip& t : d.trips) {
      const auto& se = t.stopEvents;
      CHECK(se.front().arrivalTime == se.front().departureTime - 1,
            "times: first arrival");
      CHECK(se.back().departureTime == se.back().arrivalTime + 1,
            "times: last departure");
      for (size_t i = 1; i + 1 < se.size(); i++)
        CHECK(se[i].departureTime == se[i].arrivalTime + 10 &&
                  se[i].arrivalTime == se[i - 1].departureTime + 50,
              "times: interior stop event changed");
    }
  }
  {  // 7. FromGTFS splits by default and keeps the long trip when asked to.
    Data split = loadGtfs(true);
    CHECK(split.numberOfTrips() >= 2 && maxLength(split) <= 255,
          "gtfs: " << split.numberOfTrips() << " trips, longest "
                   << maxLength(split));
    size_t events = 0;
    for (const Trip& t : split.trips) events += t.stopEvents.size();
    CHECK(events == 300, "gtfs: " << events << " stop events, expected 300");
    Data keep = loadGtfs(false);
    CHECK(keep.numberOfTrips() == 1 && maxLength(keep) == 300,
          "gtfs: splitTrips=false must keep the trip");
  }
  std::cout << (failures ? "FAILED" : "ok") << std::endl;
  return failures ? 1 : 0;
}
