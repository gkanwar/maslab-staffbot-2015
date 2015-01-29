#include <chrono>
#include <cmath>
#include <vector>

#include "lidar.h"
#include "localization.h"
#include "render.h"
#include "sensor_data.h"
#include "state_estimator.h"
#include "random.h"
#include "util.h"

using namespace std;

Map getTestMap() {
  return Map("maps/test.map");
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
      double range = lidarSamples[i];
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

int main() {
  Map testMap = getTestMap();
  RealControl control;
  StateEstimator estimator(testMap.getInitPose(), control);

  loc::ParticleFilter pf(5.0, 5.0, testMap);
  while (true) {
    TimePoint curTime = chrono::system_clock::now();
    loc::Particle best = pf.update(LidarRangeSensorData());
    // cout << "Pose: " << truePose << endl;
    cout << "Best particle: " << best.pose << endl;
    cout << "Weight: " << best.weight.getProb() << endl;
    control.setLeftSpeed(0.5);
    control.setRightSpeed(0.3);
    control.tick(curTime);
    
    // Update our estimate
    RobotMotionDelta robotDelta = estimator.tick(curTime, nullptr);

    pf.step(robotDelta);
  }

  return 0;
}
