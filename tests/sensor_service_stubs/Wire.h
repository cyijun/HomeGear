#pragma once
#include "Arduino.h"
class TwoWire {
 public:
  void begin() { ++sensor_mock::wireBegins; }
};
extern TwoWire Wire;
