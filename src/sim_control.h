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
  virtual void setLeftSpeed(double leftPower) override;
  virtual void setRightSpeed(double rightPower) override;
  void tick(TimePoint time);
  virtual int getLeftEncoder() const override{
    return leftEncoder;
  }
  virtual int getRightEncoder() const override{
    return rightEncoder;
  }
};

#endif
