#include "sim_control.h"

#include "error.h"

// Random numbers...
#define MOTOR_SPEED_PER_POWER 10.0

void SimControl::setLeftSpeed(double leftPower) {
  rassert(-1.0 <= leftPower && leftPower <= 1.0)
      << "Left motor speed " << leftPower << " is outside [-1, 1]";
  leftSpeed = leftPower * MOTOR_SPEED_PER_POWER;
}

void SimControl::setRightSpeed(double rightPower) {
  rassert(-1.0 <= rightPower && rightPower <= 1.0)
      << "Right motor speed " << rightPower << " is outside [-1, 1]";
  rightSpeed = rightPower * MOTOR_SPEED_PER_POWER;
}

void SimControl::tick(TimePoint time) {
  if (init) {
    DurationMicro diff = chrono::duration_cast<DurationMicro>(time - lastTime);
    double deltaT = 0.000001 * diff.count();
    leftEncoder += deltaT * leftSpeed * MOTOR_TICKS_PER_SPEED;
    rightEncoder += deltaT * rightSpeed * MOTOR_TICKS_PER_SPEED;
  }
  lastTime = time;
  init = true;
}
