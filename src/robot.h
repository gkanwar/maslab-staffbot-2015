#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>

#include "util.h"

#define ROBOT_RADIUS 0.088

using namespace std;

class RobotPose;
// Deltas are represented in the same way as a pose
typedef RobotPose RobotPoseDelta;

class RobotMotionDelta {
 public:
  // Wheel dist in meters
  double forward, rot;
  RobotMotionDelta(double forward, double rot) : forward(forward), rot(rot) {}

  friend ostream& operator<<(ostream& os, const RobotMotionDelta& value) {
    os << "<RobotMotionDelta>(" << value.forward << "," << value.rot << ")";
    return os;
  }
};

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

  void addDelta(RobotMotionDelta delta) {
    double deltaTheta = delta.rot / ROBOT_RADIUS;
    theta = addTheta(theta, deltaTheta);
    double deltaX = cos(theta) * delta.forward;
    double deltaY = sin(theta) * delta.forward;
    x += deltaX;
    y += deltaY;
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

  Vector getGlobalPos(RobotPose pose) const {
    double totalTheta = addTheta(robotTheta, pose.theta);
    return Vector(pose.x + cos(totalTheta)*dist, pose.y + sin(totalTheta)*dist);
  }

  double getGlobalTheta(RobotPose pose) const {
    return addTheta(theta, pose.theta);
  }

  friend ostream& operator<<(ostream& os, const RobotVector& value) {
    os << "(" << value.dist << "@" << value.robotTheta
       << ", pointing " << value.theta << ")";
    return os;
  }
};

#endif
