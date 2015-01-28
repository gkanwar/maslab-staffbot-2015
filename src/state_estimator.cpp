#include "state_estimator.h"

#include <cmath>

#include "robot.h"
#include "util.h"

#define MOTOR_SPEED_PER_POWER 0.1

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

void StateEstimator::updateMotorSpeeds(double leftPower, double rightPower) {
  leftMotorSpeed = leftPower*MOTOR_SPEED_PER_POWER;
  rightMotorSpeed = rightPower*MOTOR_SPEED_PER_POWER;
}
