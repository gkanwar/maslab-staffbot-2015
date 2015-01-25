#include "localization.h"

#include "sensor_data.h"

Map getTestMap() {
  return Map({
      Wall(1.0, 1.0, 1.0, 10.0),
      Wall(1.0, 10.0, 10.0, 10.0),
      Wall(10.0, 10.0, 10.0, 1.0)},
    { Wall(10.0, 1.0, 1.0, 1.0) });
}

class TestSensorData : public SensorData {

};

int main() {
  Map testMap = getTestMap();
  testMap.renderMap();
  while (true) {}
  return 0;
}
