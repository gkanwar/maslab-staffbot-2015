#include <thread>
#include <chrono>
#include <cmath>
#include <vector>

#include "localization.h"
#include "render.h"
#include "sensor_data.h"
#include "sim_control.h"
#include "state_estimator.h"
#include "random.h"
#include "util.h"

using namespace std;

Map getTestMap() {
  return Map("maps/test.map");
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
 private:
  RobotPose truePose;

  vector<double> buildRangeSignature(RobotPose pose, const Map& map) const {
    vector<double> sig;
    for (const RobotVector& sensorPos : sensors) {
      Vector origin = sensorPos.getGlobalPos(pose);
      double theta = sensorPos.getGlobalTheta(pose);
      if (!map.isValidPoint(origin.x, origin.y)) {
        sig.push_back(-1);
      }

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

 protected:
  vector<double> trueRangeSig;

 public:
  static vector<RobotVector> sensors;

  SimRangeSensorData(RobotPose truePose, const Map& map) : truePose(truePose) {
    trueRangeSig = buildRangeSignature(truePose, map);
  }

  Prob computeProb(const RobotPose& pose, const Map &map) const override {
    // cout << "sig done" << endl;
    Prob out = Prob::makeFromLinear(1.0);
    rassert(trueRangeSig.size() == sensors.size());
    for (int i = 0; i < trueRangeSig.size(); ++i) {
      double range = trueRangeSig[i];
      // Skip invalid readings
      // TODO: Perhaps use a low-weighted correlation to expected 
      if (range < 0) continue;

      Vector origin = sensors[i].getGlobalPos(pose);
      double sensorTheta = sensors[i].getGlobalTheta(pose);
      Vector endpoint = getEndpoint(origin, sensorTheta, range);
      Vector endpointClamped = map.clampPoint(endpoint);
      Vector closest = map.getClosest(endpointClamped.x, endpointClamped.y);
      double d = dist(closest.x, closest.y, endpoint.x, endpoint.y);

      // TUNE: stddev of PDF should be increased for more stability, reduced for
      // a more dynamic response.
      out = Prob::andProb(out, Prob::makeFromLinear(gaussianPDF(1.0, d)));
    }
    return out;
  }
};

vector<RobotVector> SimRangeSensorData::sensors = {
  RobotVector(0.1, 0, 0),
  RobotVector(0.1, 0.1*PI, 0.1*PI),
  RobotVector(0.1, 0.2*PI, 0.2*PI),
  RobotVector(0.1, 0.3*PI, 0.3*PI),
  RobotVector(0.1, 0.4*PI, 0.4*PI),
  RobotVector(0.1, 0.5*PI, 0.5*PI),
  RobotVector(0.1, 0.6*PI, 0.6*PI),
  RobotVector(0.1, 0.7*PI, 0.7*PI),
  RobotVector(0.1, 0.8*PI, 0.8*PI),
  RobotVector(0.1, 0.9*PI, 0.9*PI),
  RobotVector(0.1, 1.0*PI, 1.0*PI),
  RobotVector(0.1, 1.1*PI, 1.1*PI),
  RobotVector(0.1, 1.2*PI, 1.2*PI),
  RobotVector(0.1, 1.3*PI, 1.3*PI),
  RobotVector(0.1, 1.4*PI, 1.4*PI),
  RobotVector(0.1, 1.5*PI, 1.5*PI),
  RobotVector(0.1, 1.6*PI, 1.6*PI),
  RobotVector(0.1, 1.7*PI, 1.7*PI),
  RobotVector(0.1, 1.8*PI, 1.8*PI),
  RobotVector(0.1, 1.9*PI, 1.9*PI)
};

class SimRangeSensorDataNoisy : public SimRangeSensorData {
 public:
  SimRangeSensorDataNoisy(RobotPose truePose, const Map& map)
      : SimRangeSensorData(truePose, map) {
    for (int i = 0; i < trueRangeSig.size(); ++i) {
      double noise = gaussianSample(0.3);
      trueRangeSig[i] += noise;
    }
  }
};

int main(int argc, char** argv) {
  rassert(argc == 2) << "Must pass map name as argument";
  Map testMap(argv[1]);
  RobotPose truePose = testMap.getInitPose();
  SimControl control;
  StateEstimator estimator(truePose, control);

  loc::ParticleFilter pf(truePose.x, truePose.y, testMap);
  // Fake time for sim
  TimePoint curTime;
  const chrono::duration<long int, milli> deltaTime = chrono::milliseconds(10);
  while (true) {
    testMap.renderMap();
    pf.renderLoc();
    drawRect(truePose.x-0.05, truePose.y-0.05, 0.1, 0.1, 0.0, 1.0, 0.0);
    drawFrame();
    loc::Particle best = pf.update(SimRangeSensorDataNoisy(truePose, testMap));
    cout << "Pose: " << truePose << endl;
    cout << "Best particle: " << best.pose << endl;
    cout << "Weight: " << best.weight.getProb() << endl;
    control.setLeftSpeed(0.2);
    control.setRightSpeed(0.1);
    control.tick(curTime);
    
    // Sim-only step: maintain true info
    RobotMotionDelta robotDelta = estimator.tick(curTime, &truePose);

    pf.step(robotDelta);

    // Step the sim at a fixed rate
    curTime = curTime + deltaTime;
  }

  joinRenderThread();
  return 0;
}
