#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <vector>

#include "map.h"
#include "prob.h"
#include "robot.h"
#include "sensor_data.h"

using namespace std;

// Localization-specific classes (should not be used externally)
namespace loc {

class Particle {
 public:
  RobotPose pose;
  Prob weight;

  Particle(RobotPose pose, Prob weight) : pose(pose), weight(weight) {}
};

class ParticleFilter {
  vector<Particle> particles;
  Map map;

  ParticleFilter(double startX, double startY, Map map);

  // Update given input and return the highest likelihood particle
  Particle update(SensorData reading);

  // Renormalize all probabilities
  void renormalize();
};

}  // namespace loc

#endif
