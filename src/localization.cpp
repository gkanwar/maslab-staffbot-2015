#include "localization.h"

#include "random.h"
#include "sensor_data.h"
#include "util.h"

namespace loc {

ParticleFilter::ParticleFilter(double startX, double startY, Map map) : map(map) {
  // TODO: How many particles do we actually want to create?
  Prob eachProb = Prob::makeFromLinear(1.0/50000.0);
  for (int i = 0; i < 50000; ++i) {
    // Just guessing at some reasonable noise
    double partX = startX + gaussianNoise(0.5);
    double partY = startY + gaussianNoise(0.5);
    double theta = uniformSample(0.0, 2*PI);
    particles.emplace_back(RobotPose(partX, partY, theta),
                           Prob::makeFromLinear(1.0));
  }
  renormalize();
}

Particle ParticleFilter::update(SensorData reading) {
  bool init = false;
  Prob greatest;
  const Particle *mostLikely;
  for (Particle &p : particles) {
    Prob readingProb = reading.computeProb(p.pose, map);
    p.weight = Prob::andProb(p.weight, readingProb);
    if (!init || p.weight > greatest) {
      greatest = p.weight;
      mostLikely = &p;
      init = true;
    }
  }

  renormalize();

  return *mostLikely;
}

void ParticleFilter::renormalize() {
  bool init = false;
  Prob greatest;
  for (Particle &p : particles) {
    if (!init || p.weight > greatest) {
      greatest = p.weight;
      init = true;
    }
  }

  for (Particle &p : particles) {
    p.weight = Prob::normProb(p.weight, greatest);
  }
}

}  // namespace loc
