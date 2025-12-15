#include <chrono>
#include <iostream>
#include <random>

#include "../Algorithms/TripBased/Query/ProfileReachedIndexSIMD.h"
#include "../Algorithms/TripBased/Query/ReachedIndex.h"
#include "../DataStructures/TripBased/Data.h"

using Clock = std::chrono::high_resolution_clock;

template <typename F>
double benchmark(const char *name, F &&f, std::size_t iters) {
  for (std::size_t i = 0; i < iters / 10; ++i) {
    f();
  }

  auto start = Clock::now();
  for (std::size_t i = 0; i < iters; ++i)
    f();
  auto end = Clock::now();

  double ms = std::chrono::duration<double, std::milli>(end - start).count();
  std::cout << name << ": " << ms << " ms\n";
  return ms;
}

void benchAlreadyReached(const TripBased::Data &data) {
  TripBased::ReachedIndex scalar(data);
  TripBased::ProfileReachedIndexSIMD simd(data);

  constexpr std::size_t ITERS = 50'000'000;
  TripId trip = data.trips().front();
  volatile bool sink;

  benchmark(
      "ReachedIndex::alreadyReached",
      [&] { sink = scalar.alreadyReached(trip, 5); }, ITERS);

  benchmark(
      "ProfileReachedIndexSIMD::alreadyReached",
      [&] { sink = simd.alreadyReached(trip, 5, 1); }, ITERS);
}

void benchUpdate(const TripBased::Data &data) {
  TripBased::ReachedIndex scalar(data);
  TripBased::ProfileReachedIndexSIMD simd(data);

  constexpr std::size_t ITERS = 1'000'000;
  TripId trip = data.trips().front();

  benchmark(
      "ReachedIndex::update", [&] { scalar.update(trip, StopIndex(7)); },
      ITERS);

  benchmark(
      "ProfileReachedIndexSIMD::update", [&] { simd.update(trip, 7, 1); },
      ITERS);
}

void benchClear(const TripBased::Data &data) {
  TripBased::ReachedIndex scalar(data);
  TripBased::ProfileReachedIndexSIMD simd(data);

  constexpr std::size_t ITERS = 1000;

  benchmark(
      "ReachedIndex::clear", [&] { scalar.clear(); }, ITERS);

  benchmark(
      "ProfileReachedIndexSIMD::clear", [&] { simd.clear(); }, ITERS);
}

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <tripbased data filname>\n";
    return 1;
  }

  const std::string filename = argv[1];

  std::cout << "Loading TripBased data from: " << filename << std::endl;
  const TripBased::Data tripBasedData(filename);
  tripBasedData.printInfo();

  benchAlreadyReached(tripBasedData);
  benchUpdate(tripBasedData);
  benchClear(tripBasedData);

  return 0;
}
