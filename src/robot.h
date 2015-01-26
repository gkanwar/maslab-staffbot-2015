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
    theta = addTheta(theta, delta.theta);
  }

  friend ostream& operator<<(ostream& os, const RobotPose& rp) {
    os << "(" << rp.x << "," << rp.y << "," << rp.theta << ")";
    return os;
  }
};

// Vector to an object on the robot, represented as (dist,theta), with
// additional orientation information for the object itself
class RobotVector {
 public:
  // Position on robot, robotTheta == 0 corresponds to "forward"
  double dist, robotTheta;
  // Theta relative to robot, theta == 0 corresponds to robot "forward"
  double theta;

  RobotVector(double dist, double robotTheta, double theta)
      : dist(dist), robotTheta(robotTheta), theta(theta) {}

  Vector getGlobalPos(RobotPose pose) {
    double totalTheta = addTheta(robotTheta, pose.theta);
    return Vector(pose.x + sin(totalTheta)*dist, pose.y + cos(totalTheta)*dist);
  }

  double getGlobalTheta(RobotPose pose) {
    return addTheta(addTheta(theta, robotTheta), pose.theta);
  }
};

#endif
