#ifndef SIM_CONTROL_H
#define SIM_CONTROL_H

#include "state_estimator.h"

class SimControl {
 private:
  StateEstimator& estimator;
 public:
  // Accepts state estimator 
  SimControl(StateEstimator& estimator) : estimator(estimator) {}
  void setMotorSpeeds(double leftPower, double rightPower);
};

#endif
