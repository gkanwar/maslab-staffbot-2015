#ifndef ROBOT_H
#define ROBOT_H

class RobotPose {
 public:
  // Meters
  double x, y;
  // Radians
  double theta;

  RobotPose(double x, double y, double theta) : x(x), y(y), theta(theta) {}
};

#endif
