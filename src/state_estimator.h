#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <chrono>

#include "control.h"
#include "robot.h"

using namespace std;

typedef chrono::time_point<chrono::system_clock> TimePoint;
typedef chrono::duration<double, micro> DurationMicro;

// Class that wraps hardware interface and provides a full estimate of the robot
// on top of raw sensor values. Ex: should contain raw odometry values plus a
// method to access a delta estimate of odometry.
class StateEstimator {
 private:
  const Control& control;
  TimePoint lastUpdate;
  int leftEncoder, rightEncoder;
  RobotPose lastEst;
  bool init;

 public:
  StateEstimator(RobotPose initPose, const Control& control)
      : lastEst(initPose), control(control), init(false) {}

  // TODO: For now, just return robot pose and delta
  RobotMotionDelta tick(TimePoint time, RobotPose* pose);
};

#endif
