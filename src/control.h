#ifndef CONTROL_H
#define CONTROL_H

#define MOTOR_TICKS_PER_SPEED 7500

class Control {
 public:
  virtual int getLeftEncoder() const = 0;
  virtual int getRightEncoder() const = 0;
  virtual void setLeftSpeed(double) = 0;
  virtual void setRightSpeed(double) = 0; 
};

#endif
