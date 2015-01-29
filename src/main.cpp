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

#if EDISON

using namespace std;

int running = 1;

void sig_handler(int signo) {
  if (signo == SIGINT) {
    running = 0;
  }
}

Map getTestMap() {
  return Map("maps/test.map");
}

class LidarRangeSensorData : public SensorData {
 public:
  static vector<RobotVector> sensors;

  LidarRangeSensorData() {
    if (sensors.empty()) {
      for (int i = 0; i < 360; ++i) {
        if (i % 10 != 0) continue;
        sensors.emplace_back(0.076, i*(2*PI/360.0), i*(2*PI/360.0));
      }
    }
  }

  Prob computeProb(const RobotPose& pose, const Map &map) const override {
    // cout << "sig done" << endl;
    Prob out = Prob::makeFromLinear(1.0);
    for (int i = 0; i < 360; ++i) {
      if (i % 10 != 0) continue;
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

int main(int argc, char** argv) {
  rassert(argc == 2) << "Must give map as argument";
  signal(SIGINT, sig_handler);

  Map testMap(argv[1]);
  RealControl control;
  StateEstimator estimator(testMap.getInitPose(), control);

  // TODO: Fix interface
  loc::ParticleFilter pf(testMap.getInitPose().x, testMap.getInitPose().y, testMap);
  while (running) {
    TimePoint curTime = chrono::system_clock::now();
    loc::Particle best = pf.update(LidarRangeSensorData());
    // cout << "Pose: " << truePose << endl;
    cout << "Best particle: " << best.pose << endl;
    cout << "Weight: " << best.weight.getProb() << endl;
    //control.setLeftSpeed(0.2);
    //control.setRightSpeed(0.1);
    
    // Update our estimate
    RobotMotionDelta robotDelta = estimator.tick(curTime, nullptr);

    pf.step(robotDelta);
  }

  cout << "Exiting" << endl;

  return 0;
}

#endif
