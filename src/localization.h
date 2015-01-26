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
 private:
  vector<Particle> particles;
  Map map;

 public:
  ParticleFilter(double startX, double startY, Map map);

  // Update given input and return the highest likelihood particle
  Particle update(const SensorData& reading);

  // Step the estimates given a delta motion
  void step(RobotMotionDelta robotDelta);

  // Renormalize all probabilities, such that the highest probability particle
  // is probability 1, and everything else is proportional.
  void renormalize();

  // Resample particles
  void resample();

  // Draw the particles
  void renderLoc();
};

}  // namespace loc

#endif
