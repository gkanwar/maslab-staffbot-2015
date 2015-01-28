#include "random.h"

#include "util.h"

double gaussianSample(double stdDev) {
  normal_distribution<double> dist(0.0, stdDev);
  return dist(generator);
}

double uniformSample(double min, double max) {
  uniform_real_distribution<double> dist(min, max);
  return dist(generator);
}

double gaussianPDF(double stdDev, double dev) {
  return exp(-dev*dev / (2*stdDev*stdDev));
}
