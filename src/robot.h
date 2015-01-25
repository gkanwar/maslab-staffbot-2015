#ifndef ROBOT_H
#define ROBOT_H

class RobotPose {
  // Meters
  double x, y;
  // Radians
  double theta;

 public:
  RobotPose(double x, double y, double theta) : x(x), y(y), theta(theta) {}
};

#endif
