#include "state_estimator.h"

#include <cmath>

#include "robot.h"
#include "util.h"

RobotMotionDelta StateEstimator::tick(TimePoint time, RobotPose* pose) {
  RobotMotionDelta deltaPose(0, 0);
  if (init) {
    DurationMicro diff = chrono::duration_cast<DurationMicro>(time - lastUpdate);
    double deltaT = 0.000001 * diff.count();
    double rot = deltaT * (rightMotorSpeed - leftMotorSpeed) / 2.0;
    double forward = deltaT * (rightMotorSpeed + leftMotorSpeed) / 2.0;
    deltaPose = RobotMotionDelta(forward, rot);
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
