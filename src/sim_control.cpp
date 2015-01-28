#include "sim_control.h"

#include "error.h"

void SimControl::setMotorSpeeds(double leftPower, double rightPower) {
  rassert(-1.0 <= leftPower && leftPower <= 1.0)
      << "Left motor speed " << leftPower << " is outside [-1, 1]";
  rassert(-1.0 <= rightPower && rightPower <= 1.0)
      << "Right motor speed " << rightPower << " is outside [-1, 1]";
  estimator.updateMotorSpeeds(leftPower, rightPower);
}
