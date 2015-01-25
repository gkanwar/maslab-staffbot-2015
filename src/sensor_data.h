#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include "error.h"

// Interface for a sensor data bundle
class SensorData {
 public:
  // Compute probability of this sensor reading given pose and map
  virtual double computeProb(RobotPose pose, Map map) {
    // Subclasses should implement
    rassert(false)
        << "computeProb must be implemented by SensorData subclasses";
  }
};

#endif
