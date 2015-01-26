#include "sim_control.h"

#include "error.h"

void SimControl::setMotorSpeeds(double left, double right) {
  rassert(-1.0 <= left && left <= 1.0) << "Left motor speed "
                                       << left << " is outside [-1, 1]";
  rassert(-1.0 <= right && right <= 1.0) << "Right motor speed "
                                         << right << " is outside [-1, 1]";
  estimator.updateMotorSpeeds(left, right);
}
