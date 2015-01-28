#ifndef UTIL_H
#define UTIL_H

#include <cmath>

#include "error.h"

#define PI 3.14159265358979323846

// NOTE: All thetas are CCW

// Normalize theta
inline double normTheta(double theta) {
  if (theta < 0) {
    return theta + 2*PI;
  }
  else if (theta >= 2*PI) {
    return theta - 2*PI;
  }
  return theta;
}

// Add theta and keep resulting value in range [0, 2*PI)
inline double addTheta(double t1, double t2) {
  double total = t1 + t2;
  return normTheta(total);
}

// Get theta for a line segment
inline double getTheta(double x1, double y1, double x2, double y2) {
  return normTheta(atan2(y2-y1, x2-x1));
}

// Return whether the given angle is contained between first and second, inclusive
inline bool isThetaContained(double first, double second, double theta) {
  rassert(0.0 <= first && first < 2*PI) << first;
  rassert(0.0 <= second && second < 2*PI) << second;
  if (first < second) {
    return (theta >= first && theta <= second);
  }
  else {
    return (theta >= first || theta <= second);
  }
}

inline double distSq(double x1, double y1, double x2, double y2) {
  return (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
}

inline double dist(double x1, double y1, double x2, double y2) {
  return sqrt(distSq(x1, y1, x2, y2));
}

// 2-D vector in global space
class Vector {
 public:
  // Meters, in global coords
  double x, y;

  Vector() : Vector(0, 0) {}
  Vector(double x, double y) : x(x), y(y) {}

  friend ostream& operator<<(ostream& os, const Vector& value) {
    os << "<" << value.x << "," << value.y << ">";
    return os;
  }
};

// Get a ray endpoint given an origin, theta, and distance
inline Vector getEndpoint(Vector origin, double theta, double d) {
  return Vector(origin.x + d*cos(theta), origin.y + d*sin(theta));
}

#endif
