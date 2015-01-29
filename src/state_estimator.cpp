#include "state_estimator.h"

#include <cmath>

#include "control.h"
#include "robot.h"
#include "util.h"

RobotMotionDelta StateEstimator::tick(TimePoint time, RobotPose* pose) {
  RobotMotionDelta deltaPose(0, 0);
  int encR = control.getRightEncoder();
  int encL = control.getLeftEncoder();
  if (init) {
    int deltaR = encR - rightEncoder;
    int deltaL = encL - leftEncoder;
    double rot = (deltaR - deltaL) / (MOTOR_TICKS_PER_SPEED * 2.0);
    double forward = (deltaR + deltaL) / (MOTOR_TICKS_PER_SPEED * 2.0);
    deltaPose = RobotMotionDelta(forward, rot);
  }
  
  init = true;
  leftEncoder = encL;
  rightEncoder = encR;
  lastEst.addDelta(deltaPose);
  if (pose != nullptr) {
    *pose = lastEst;
  }
  return deltaPose;
}
