#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>

#include "util.h"

using namespace std;

class RobotPose;
// Deltas are represented in the same way as a pose
typedef RobotPose RobotPoseDelta;

class RobotPose {
 public:
  // Meters
  double x, y;
  // Radians
  double theta;

  RobotPose(double x, double y, double theta) : x(x), y(y), theta(theta) {}

  void addDelta(RobotPoseDelta delta) {
    x += delta.x;
    y += delta.y;
    theta += delta.theta;
    if (theta < 0) {
      theta += 2*PI;
    }
    else if (theta >= 2*PI) {
      theta -= 2*PI;
    }
  }

  friend ostream& operator<<(ostream& os, const RobotPose& rp) {
    os << "(" << rp.x << "," << rp.y << "," << rp.theta << ")";
    return os;
  }
};

#endif
