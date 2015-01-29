#include "localization.h"

#include <algorithm>
#include <cmath>

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
    double partX = startX + gaussianSample(0.1);
    double partY = startY + gaussianSample(0.1);
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

  Particle out = *mostLikely;

  // Possibly resample
  double weightSqTotal = 0.0;
  for (const Particle &p : particles) {
    weightSqTotal += Prob::andProb(p.weight, p.weight).getProb();
  }
  double numEffParticles = 1.0 / weightSqTotal;
  cout << "Effective particles: " << numEffParticles << endl;
  // Less than 1% effective particles
  if (numEffParticles < NUM_PARTS/100.0) {
    resample();
  }

  return out;
}

void ParticleFilter::step(RobotMotionDelta robotDelta) {
  for (Particle &p : particles) {
    p.pose.addDelta(robotDelta);
  }
}

void ParticleFilter::renormalize() {
  double totalProb = 0.0;
  for (Particle &p : particles) {
    totalProb += p.weight.getProb();
  }

  double negLogTotalProb = -log(totalProb);
  for (Particle &p : particles) {
    p.weight = Prob::normProb(p.weight, negLogTotalProb);
  }
}

void ParticleFilter::resample() {
  cout << "Resampling." << endl;
  // TODO: Is simple resampling enough?
  vector<Particle> resampled;
  vector<double> cumulativeProb;
  double curProb = 0.0;
  for (Particle &p : particles) {
    curProb += p.weight.getProb();
    cumulativeProb.push_back(curProb);
  }
  rassert(abs(curProb - 1.0) < 0.0000001)
      << "Total probability too far off of 1: " << curProb;

  int i;
  // TUNE: If we run into kidnapped robot problems, allow for some completely
  // random resampling.
  int randomResample = 0; // NUM_PARTS/10;
  bool lowestInit = false;
  Prob lowest;
  for (i = 0; i < NUM_PARTS - randomResample; ++i) {
    double rand = uniformSample(0.0, 1.0);
    for (int j = 0; j < cumulativeProb.size(); ++j) {
      double prob = cumulativeProb[j];
      if (prob >= rand) {
        Particle newPart = particles[j];
        newPart.pose.addDelta(RobotPoseDelta(gaussianSample(0.1),
                                             gaussianSample(0.1),
                                             gaussianSample(0.1)));
        if (!lowestInit || lowest >= newPart.weight) {
          lowest = newPart.weight;
          lowestInit = true;
        }
        resampled.push_back(newPart);
        break;
      }
    }
    rassert(resampled.size() == i+1)
        << "Resampled size is: " << resampled.size() << ", not: " << i+1;
  }
  for (; i < NUM_PARTS; ++i) {
    // Match the lowest particle prob, for stability
    resampled.push_back(Particle(RobotPose(
        uniformSample(0.0, GRID_SIZE*TILE_SIZE),
        uniformSample(0.0, GRID_SIZE*TILE_SIZE),
        uniformSample(0.0, 2*PI)), lowest));
  }

  particles = resampled;
}

void ParticleFilter::renderLoc() {
  for (Particle &p : particles) {
    if (p.weight.getNegLogProb() > 10.0) continue;
    double red = max(0.2, 1.0 - p.weight.getNegLogProb() / 10.0);
    drawRect(p.pose.x-0.05, p.pose.y-0.05, 0.1, 0.1, red, 0.0, 0.0);
  }
}

}  // namespace loc
