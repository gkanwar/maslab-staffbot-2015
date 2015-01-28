#include "localization.h"

#include <thread>
#include <chrono>
#include <cmath>
#include <vector>

#include "render.h"
#include "sensor_data.h"
#include "sim_control.h"
#include "state_estimator.h"
#include "random.h"
#include "util.h"

using namespace std;

Map getTestMap() {
  return Map({
      Wall(1.0, 1.0, 1.0, 7.0),
      Wall(1.0, 7.0, 10.0, 7.0),
      Wall(10.0, 7.0, 10.0, 1.0)},
    { Wall(10.0, 1.0, 1.0, 1.0) });
}

// Basic "sensor data" which computes probabilities based on closeness to
// ground truth pose.
class SimSensorData : public SensorData {
 private:
  RobotPose truePose;

 public:
  SimSensorData(RobotPose truePose) : truePose(truePose) {}

  Prob computeProb(const RobotPose& pose, const Map& map) const override {
    // Quick check: Just return probability based on closeness to existing pose
    double distSquared = distSq(pose.x, pose.y, truePose.x, truePose.y);
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

    return Prob::makeFromLinear(gaussianPDF(0.5*PI, thetaDist) *
                                gaussianPDF(0.5, distSquared));
  }
};

class SimRangeSensorData : public SensorData {
 public:
  static vector<RobotVector> sensors;
 private:
  // TODO: Update this to be reasonable
  static constexpr double rangeThresh = 100.0;

  RobotPose truePose;

  vector<double> buildRangeSignature(RobotPose pose, const Map& map) const {
    vector<double> sig;
    for (const RobotVector& sensorPos : sensors) {
      Vector origin = sensorPos.getGlobalPos(pose);
      double theta = sensorPos.getGlobalTheta(pose);
      rassert(map.isValidPoint(origin.x, origin.y))
          << "Invalid sensor origin: " << origin;

      // TODO: This is slow, we can do this smarter if needed
      // vector< pair<int, int> > intersected;
      bool found = false;
      if (theta <= PI) {
        // y is pos
        double err = 0.0;
        int x = map.toGridCoord(origin.x);
        for (int y = map.toGridCoord(origin.y); y < GRID_SIZE; ++y) {
          err += abs(1.0/tan(theta));
          while (err >= 0.5) {
            if (x < 0 || x >= GRID_SIZE) break;
            if (map.isObstacle(x, y)) {
              sig.push_back(dist(TILE_SIZE*x, TILE_SIZE*y, origin.x, origin.y));
              found = true;
              break;
            }
            // x is pos
            if (theta <= 0.5*PI || theta >= 1.5*PI) x++;
            else x--;
            err -= 1.0;
          }
          if (found) break;
        }
      }
      else {
        // y is neg
        double err = 0.0;
        int x = map.toGridCoord(origin.x);
        for (int y = map.toGridCoord(origin.y); y >= 0; --y) {
          err += abs(1.0/tan(theta));
          while (err >= 0.5) {
            if (x < 0 || x >= GRID_SIZE) break;
            if (map.isObstacle(x, y)) {
              sig.push_back(dist(TILE_SIZE*x, TILE_SIZE*y, origin.x, origin.y));
              found = true;
              break;
            }
            // x is pos
            if (theta <= 0.5*PI || theta >= 1.5*PI) x++;
            else x--;
            err -= 1.0;
          }
          if (found) break;
        }
      }

      // Not found
      if (!found) {
        sig.push_back(-1);
      }
    }
    rassert(sig.size() == sensors.size());
    return sig;
  }

 public:
  SimRangeSensorData(RobotPose truePose) : truePose(truePose) {}

  Prob computeProb(RobotPose pose, Map map) const override {
    vector<double> rangeSig = buildRangeSignature(pose, map);
    // cout << "sig done" << endl;
    Prob out = Prob::makeFromLinear(1.0);
    rassert(rangeSig.size() == sensors.size());
    for (int i = 0; i < rangeSig.size(); ++i) {
      double range = rangeSig[i];
      // Skip invalid readings
      // TODO: Perhaps use a low-weighted correlation to expected 
      if (range < 0) continue;

      Vector origin = sensors[i].getGlobalPos(pose);
      double sensorTheta = sensors[i].getGlobalTheta(pose);
      Vector endpoint = getEndpoint(origin, sensorTheta);

      Vector closest = map.getClosest(endpoint.x, endpoint.y);
      double d = dist(closest.x, closest.y, endpoint.x, endpoint.y);
      // Thresh for erroneous read
      if (d > 0.5) {
        // Penalize misplaced sensor reading
        // TODO: Is this the right penalty?
        out = Prob::andProb(out, Prob::makeFromLinear(0.1));
      }
      else {
        out = Prob::andProb(out, Prob::makeFromLinear(gaussianPDF(0.5, d)));
      }
    }
    return out;
  }
};

vector<RobotVector> SimRangeSensorData::sensors = {
  RobotVector(0.1, 0, 0),
  RobotVector(0.1, 0.5*PI, 0.5*PI),
  RobotVector(0.1, PI, PI),
  RobotVector(0.1, 1.5*PI, 1.5*PI)
};

int main() {
  Map testMap = getTestMap();
  RobotPose truePose(5.0, 5.0, 0.0);
  StateEstimator estimator(truePose);
  SimControl control(estimator);

  loc::ParticleFilter pf(5.0, 5.0, testMap);
  // Fake time for sim
  TimePoint curTime;
  const chrono::duration<long int, milli> deltaTime = chrono::milliseconds(50);
  while (true) {
    testMap.renderMap();
    pf.renderLoc();
    drawRect(truePose.x-0.05, truePose.x+0.05, 0.1, 0.1, 0.0, 1.0, 0.0);
    drawFrame();
    loc::Particle best = pf.update(SimRangeSensorData(truePose));
    cout << "Best particle: " << best.pose.x << "," << best.pose.y
         << "," << best.pose.theta << endl;
    cout << "Weight: " << best.weight.getProb() << endl;
    control.setMotorSpeeds(0.5, -0.3);

    // Sim-only step: maintain true info
    RobotMotionDelta robotDelta = estimator.tick(
        curTime, &truePose);
    cout << "Pose: " << truePose << endl;

    pf.step(robotDelta);

    // Step the sim at a fixed rate
    curTime = curTime + deltaTime;
  }

  joinRenderThread();
  return 0;
}
