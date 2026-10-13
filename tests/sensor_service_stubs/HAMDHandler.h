#pragma once
#include "Arduino.h"
class HAMqttDiscoveryHandler {
 public:
  String getStateTopic() { return String("homegear/state"); }
};
class HAMqttDiscoverySensor {
 public:
  explicit HAMqttDiscoverySensor(HAMqttDiscoveryHandler &) {}
  void setDeviceNameMin(const char *key) { key_ = String(key); }
  void setDeviceClass(const char *) {}
  void setStateClass(const char *) {}
  void setUnitOfMeasurement(const char *) {}
  void setValueTemplate(String) {}
  void construct() {}
  String getMqttDiscoveryConfigTopic() { return key_; }
  String getMqttDiscoveryMesg() { return String("{}"); }
 private:
  String key_;
};
