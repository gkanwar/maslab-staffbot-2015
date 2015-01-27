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
double gaussianSample(double stdDev);
// Uniform sample in the range [min, max)
double uniformSample(double min, double max);
// Return gaussian probability density at dev from center, given stdDev
double gaussianPDF(double stdDev, double dev);

#endif
