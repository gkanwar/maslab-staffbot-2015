#ifndef SIM_CONTROL_H
#define SIM_CONTROL_H

#include "control.h"
#include "state_estimator.h"

class SimControl : public Control {
 private:
  bool init = false;
  TimePoint lastTime;
  int leftEncoder, rightEncoder;
  double leftSpeed, rightSpeed;
 public:
  SimControl() : leftEncoder(0), rightEncoder(0) {}
  void setMotorSpeeds(double leftPower, double rightPower);
  void tick(TimePoint time);
  int getLeftEncoder() const {
    return leftEncoder;
  }
  int getRightEncoder() const {
    return rightEncoder;
  }
};

#endif
