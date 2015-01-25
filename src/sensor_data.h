#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <cassert>

// Interface for a sensor data bundle
class SensorData {
 public:
  // Compute probability of this sensor reading given pose and map
  virtual double computeProb(RobotPose pose, Map map) {
    // Subclasses should implement
    assert(false);
  }
};

#endif
