#ifndef REAL_CONTROL_H
#define REAL_CONTROL_H

#if EDISON

#include "control.h"
#include "lidar.h"
#include "encoder.h"
#include "motor.h"
#include "pinDef.h"

class RealControl : public Control {
 private:
  Encoder encLeft;
  Encoder encRight;
  Motor motLeft;
  Motor motRight;
 public:
  RealControl() : encLeft(ENC_LEFT_A_PIN, ENC_LEFT_B_PIN),
                  encRight(ENC_RIGHT_A_PIN, ENC_RIGHT_B_PIN),
                  motLeft(MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN),
                  motRight(MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN) {
    lidar_init();
  }
  virtual int getLeftEncoder() const override {
    return -1 * encLeft.getCount();
  }
  virtual int getRightEncoder() const override {
    return encRight.getCount();
  }
  virtual void setLeftSpeed(double leftPower) override {
    rassert(-1.0 <= leftPower && leftPower <= 1.0);
    motLeft.setSpeed(fabs(leftPower), leftPower >= 0);
  }
  virtual void setRightSpeed(double rightPower) override {
    rassert(-1.0 <= rightPower && rightPower <= 1.0);
    motRight.setSpeed(fabs(rightPower), rightPower >= 0);
  }
};

#endif

#endif
