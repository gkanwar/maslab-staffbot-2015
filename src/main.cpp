#include <chrono>
#include <cmath>
#include <csignal>
#include <vector>

#include "lidar.h"
#include "localization.h"
#include "real_control.h"
#include "render.h"
#include "sensor_data.h"
#include "state_estimator.h"
#include "random.h"
#include "util.h"

using namespace std;

int running = 1;

void sig_handler(int signo) {
  if (signo == SIGINT) {
    running = 0;
  }
}

Map getTestMap() {
  return Map({
      Wall(1.0, 1.0, 1.0, 7.0),
      Wall(1.0, 7.0, 10.0, 7.0),
      Wall(10.0, 7.0, 10.0, 1.0)},
    { Wall(10.0, 1.0, 1.0, 1.0) });
}

class LidarRangeSensorData : public SensorData {
 public:
  static vector<RobotVector> sensors;

  LidarRangeSensorData() {
    if (sensors.empty()) {
      for (int i = 0; i < 360; ++i) {
        sensors.emplace_back(0.076, i*(2*PI/360.0), i*(2*PI/360.0));
      }
    }
  }

  Prob computeProb(const RobotPose& pose, const Map &map) const override {
    // cout << "sig done" << endl;
    Prob out = Prob::makeFromLinear(1.0);
    for (int i = 0; i < 360; ++i) {
      uint32_t rangeInt = lidarSamples[i];
      if (rangeInt == 0xFFFFFFFF) continue;
      double range = rangeInt/1000.0;
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

vector<RobotVector> LidarRangeSensorData::sensors = {};

int main() {
  Map testMap = getTestMap();
  RobotPose initPose(5.0, 5.0, 0.0);
  RealControl control;
  StateEstimator estimator(initPose, control);

  loc::ParticleFilter pf(5.0, 5.0, testMap);
  while (running) {
    TimePoint curTime = chrono::system_clock::now();
    loc::Particle best = pf.update(LidarRangeSensorData());
    // cout << "Pose: " << truePose << endl;
    cout << "Best particle: " << best.pose << endl;
    cout << "Weight: " << best.weight.getProb() << endl;
    control.setLeftSpeed(0.2);
    control.setRightSpeed(0.1);
    
    // Update our estimate
    RobotMotionDelta robotDelta = estimator.tick(curTime, &initPose);

    pf.step(robotDelta);
  }

  return 0;
}
