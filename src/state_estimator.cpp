#include "state_estimator.h"

#include <cmath>

#include "robot.h"
#include "util.h"

#define ROBOT_RADIUS 0.20

RobotPoseDelta StateEstimator::tick(TimePoint time, RobotPose* pose) {
  RobotPoseDelta deltaPose(0, 0, 0);
  if (init) {
    DurationMicro diff = chrono::duration_cast<DurationMicro>(time - lastUpdate);
    double deltaT = 0.000001 * diff.count();
    double rot = deltaT * (rightMotorSpeed - leftMotorSpeed) / 2.0;
    double deltaTheta = rot / ROBOT_RADIUS;
    double forward = deltaT * (rightMotorSpeed + leftMotorSpeed) / 2.0;
    double deltaX = cos(lastEst.theta) * forward;
    double deltaY = sin(lastEst.theta) * forward;
    deltaPose = RobotPoseDelta(deltaX, deltaY, deltaTheta);
    lastEst.addDelta(deltaPose);
  }
  lastUpdate = time;
  init = true;
  *pose = lastEst;
  return deltaPose;
}

void StateEstimator::updateMotorSpeeds(double left, double right) {
  leftMotorSpeed = left;
  rightMotorSpeed = right;
}
