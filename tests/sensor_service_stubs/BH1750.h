#pragma once
#include "Wire.h"
class BH1750 {
 public:
  enum Mode { ONE_TIME_HIGH_RES_MODE };
  explicit BH1750(uint8_t) {}
  bool begin(Mode, uint8_t, TwoWire *) { return sensor_mock::lightsHealthy; }
  bool measurementReady(bool) { return sensor_mock::lightsHealthy; }
  float readLightLevel() { ++sensor_mock::lightReads; return 150.0f; }
  bool configure(Mode) { return sensor_mock::lightsHealthy; }
};
