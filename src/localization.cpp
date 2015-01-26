#include "localization.h"

#include <algorithm>

#include "random.h"
#include "render.h"
#include "sensor_data.h"
#include "util.h"

using namespace std;

#define NUM_PARTS 500

namespace loc {

ParticleFilter::ParticleFilter(double startX, double startY, Map map) : map(map) {
  for (int i = 0; i < NUM_PARTS; ++i) {
    // Just guessing at some reasonable noise
    double partX = startX + gaussianNoise(0.5);
    double partY = startY + gaussianNoise(0.5);
    double theta = uniformSample(0.0, 2*PI);
    particles.emplace_back(RobotPose(partX, partY, theta),
                           Prob::makeFromLinear(1.0));
  }
  renormalize();
}

Particle ParticleFilter::update(const SensorData& reading) {
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

void ParticleFilter::renderLoc() {
  for (Particle &p : particles) {
    double red = max(0.2, 1.0 - p.weight.getNegLogProb() / 10.0);
    drawRect(p.pose.x*(RENDER_WIDTH/(double)(GRID_SIZE*TILE_SIZE)),
             p.pose.y*(RENDER_HEIGHT/(double)(GRID_SIZE*TILE_SIZE)), 3, 3,
             red, 0.0, 0.0);
  }
}

}  // namespace loc
