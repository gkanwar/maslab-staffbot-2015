#ifndef RANDOM_H
#define RANDOM_H

#include <chrono>
#include <random>

using namespace std;

namespace {
// Generator seeded with current time
default_random_engine generator(
    chrono::system_clock::now().time_since_epoch().count());
}  // anonymous namespace

// Return 0-centered gaussian noise with given stdDev
double gaussianNoise(double stdDev) {
  normal_distribution<double> dist(0.0, stdDev);
  return dist(generator);
}

// Uniform sample in the range [min, max)
double uniformSample(double min, double max) {
  uniform_real_distribution<double> dist(min, max);
  return dist(generator);
}

#endif
