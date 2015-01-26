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
