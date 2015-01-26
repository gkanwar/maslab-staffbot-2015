#include "localization.h"

#include <thread>
#include <chrono>

#include "render.h"
#include "sensor_data.h"
#include "util.h"

using namespace std;

Map getTestMap() {
  return Map({
      Wall(1.0, 1.0, 1.0, 10.0),
      Wall(1.0, 10.0, 10.0, 10.0),
      Wall(10.0, 10.0, 10.0, 1.0)},
    { Wall(10.0, 1.0, 1.0, 1.0) });
}

// Dummy sensor data class that biases towards orientation with theta = PI
class TestSensorData : public SensorData {
 public:
  Prob computeProb(RobotPose pose, Map map) const override {
    return Prob::makeFromLinear(1.0 - (abs(pose.theta - PI) / PI));
  }
};

int main() {
  Map testMap = getTestMap();

  loc::ParticleFilter pf(5.0, 5.0, testMap);
  while (true) {
    testMap.renderMap();
    pf.renderLoc();
    drawFrame();
    pf.update(TestSensorData());
    this_thread::sleep_for(chrono::milliseconds(50));
  }

  joinRenderThread();
  return 0;
}
