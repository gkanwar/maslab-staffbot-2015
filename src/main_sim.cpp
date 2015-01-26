#include "localization.h"

#include <thread>
#include <chrono>

#include "render.h"
#include "sensor_data.h"
#include "sim_control.h"
#include "state_estimator.h"
#include "util.h"

using namespace std;

Map getTestMap() {
  return Map({
      Wall(1.0, 1.0, 1.0, 10.0),
      Wall(1.0, 10.0, 10.0, 10.0),
      Wall(10.0, 10.0, 10.0, 1.0)},
    { Wall(10.0, 1.0, 1.0, 1.0) });
}

// Basic "sensor data" which computes probabilities based on closeness to
// ground truth pose.
class SimSensorData : public SensorData {
 private:
  RobotPose truePose;

 public:
  SimSensorData(RobotPose truePose) : truePose(truePose) {}

  Prob computeProb(RobotPose pose, Map map) const override {
    // Quick check: Just return probability based on closeness to existing pose
    double distSq = (pose.x - truePose.x)*(pose.x - truePose.x) +
        (pose.y - truePose.y)*(pose.y - truePose.y);
    double thetaDist1 = pose.theta - truePose.theta;
    double thetaDist2 = truePose.theta - pose.theta;
    if (thetaDist1 < 0) {
      rassert(thetaDist2 >= 0) << thetaDist2;
      thetaDist1 += 2*PI;
      rassert(thetaDist1 >= 0) << thetaDist2;
    }
    else {
      rassert(thetaDist2 <= 0) << thetaDist2;
      thetaDist2 += 2*PI;
      rassert(thetaDist2 >= 0) << thetaDist2;
    }
    double thetaDist = min(thetaDist1, thetaDist2);

    return Prob::makeFromLinear((0.5*thetaDist/PI) *
                                max(1.0 - (distSq / 25.0), 0.0));
  }
};

class SimRangeSensorData : public SensorData {
 private:
  static vector<RobotVector> sensors;
  // TODO: Update this to be reasonable
  static constexpr double rangeThresh = 100.0;

  RobotPose truePose;

  vector<double> buildRangeSignature(const RobotPose& pose, const Map& map) {
    vector<double> sig;
    for (RobotVector sensorPos : sensors) {
      Vector origin = sensorPos.getGlobalPos(pose);
      double theta = sensorPos.getGlobalTheta(pose);

      // TODO: This is slow, we can do this smarter if needed
      vector< pair<int, int> > intersected;
      for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
          double centerX = i*TILE_SIZE;
          double centerY = j*TILE_SIZE;
          // For each of the 4 border lines, does the ray intersect?
          double thetaP1 = getTheta(origin.x, origin.y, centerX-0.5*TILE_SIZE,
                                    centerY-0.5*TILE_SIZE);
          double thetaP2 = getTheta(origin.x, origin.y, centerX+0.5*TILE_SIZE,
                                    centerY-0.5*TILE_SIZE);
          double thetaP3 = getTheta(origin.x, origin.y, centerX+0.5*TILE_SIZE,
                                    centerY+0.5*TILE_SIZE);
          double thetaP4 = getTheta(origin.x, origin.y, centerX-0.5*TILE_SIZE,
                                    centerY+0.5*TILE_SIZE);
          if (isThetaContained(thetaP1, thetaP2, theta) ||
              isThetaContained(thetaP2, thetaP3, theta) ||
              isThetaContained(thetaP3, thetaP4, theta) ||
              isThetaContained(thetaP4, thetaP1, theta)) {
            intersected.push_back(make_pair(i, j));
          }
        }
      }
      double minDist = -1;
      for (auto point : intersected) {
        double d = dist(point.first, point.second, origin.x, origin.y);
        if (minDist < 0 ||
            minDist > d) {
          minDist = d;
        }
      }

      // minDist == -1 indicates no wall
      if (minDist > rangeThresh) {
        minDist = -1;
      }
      sig.push_back(minDist);
    }
  }

 public:
  SimRangeSensorData(RobotPose truePose) : truePose(truePose) {
    sensors.emplace_back(0.1, 0, 0);
    sensors.emplace_back(0.1, 0.5*PI, 0.5*PI);
    sensors.emplace_back(0.1, PI, PI);
    sensors.emplace_back(0.1, 1.5*PI, 1.5*PI);
  }

  Prob computeProb(RobotPose pose, Map map) const override {
    // TODO
    return Prob::makeFromLinear(1.0);
  }
};

int main() {
  Map testMap = getTestMap();
  RobotPose truePose(5.0, 5.0, 0.0);
  StateEstimator estimator(truePose);
  SimControl control(estimator);

  loc::ParticleFilter pf(5.0, 5.0, testMap);
  while (true) {
    testMap.renderMap();
    pf.renderLoc();
    drawFrame();
    loc::Particle best = pf.update(SimSensorData(truePose));
    cout << "Best particle: " << best.pose.x << "," << best.pose.y
         << "," << best.pose.theta << endl;
    cout << "Weight: " << best.weight.getProb() << endl;
    control.setMotorSpeeds(0.5, -0.3);

    // Sim-only step: maintain true info
    RobotPoseDelta poseDelta = estimator.tick(
        chrono::system_clock::now(), &truePose);
    cout << "Pose: " << truePose << endl;

    pf.step(poseDelta);

    this_thread::sleep_for(chrono::milliseconds(50));
  }

  joinRenderThread();
  return 0;
}
