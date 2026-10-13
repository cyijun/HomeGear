#pragma once
#include "Arduino.h"
#include <ArduinoJson.h>

class HAMqttDiscoveryHandler {
 public:
  HAMqttDiscoveryHandler() = default;
  HAMqttDiscoveryHandler(String platform, String serial, String, String, String, HAMqttDiscoveryHandler &)
      : prefix_(platform + "/" + serial) {}
  String getCtrlTopic() { return prefix_ + "/ctrl"; }
  String getStateTopic() { return prefix_ + "/state"; }
  String getMqttDiscoveryMesgBase() { return String("{}"); }
 private:
  String prefix_ = String("homegear/base");
};

class HAMqttDiscoveryClimate {
 public:
  explicit HAMqttDiscoveryClimate(HAMqttDiscoveryHandler &device) : stateTopic_(device.getStateTopic()), ctrlTopic_(device.getCtrlTopic()) {}
  void setDeviceNameMin(String name) { name_ = name; }
  void setModeCommandTemplate(String value) { modeCommand_ = value; }
  void setModeStateTemplate(String value) { modeState_ = value; }
  void setTemperatureCommandTemplate(String value) { temperatureCommand_ = value; }
  void setTemperatureStateTemplate(String value) { temperatureState_ = value; }
  void setTemperatureUnit(String value) { unit_ = value; }
  void setMaxTemp(float value) { max_ = value; }
  void setMinTemp(float value) { min_ = value; }
  void setModes(String modes[10]) { for (int i = 0; i < 10; ++i) modes_[i] = modes[i]; }
  void construct() {
    StaticJsonDocument<1024> json;
    json["mode_cmd_t"] = ctrlTopic_;
    json["mode_stat_t"] = stateTopic_;
    json["mode_cmd_tpl"] = modeCommand_;
    json["mode_stat_tpl"] = modeState_;
    if (!temperatureCommand_.isEmpty()) json["temp_cmd_tpl"] = temperatureCommand_;
    if (!temperatureState_.isEmpty()) json["temp_stat_tpl"] = temperatureState_;
    json["min_temp"] = min_; json["max_temp"] = max_; json["temp_unit"] = unit_;
    for (const auto &mode : modes_) if (!mode.isEmpty()) json["modes"].add(mode);
    payload_ = String();
    serializeJson(json, payload_);
  }
  String getMqttDiscoveryConfigTopic() { return String("homeassistant/climate/") + name_ + "/config"; }
  String getMqttDiscoveryMesg() { return payload_; }
 private:
  String name_, stateTopic_, ctrlTopic_, modeCommand_, modeState_, temperatureCommand_, temperatureState_, payload_;
  String modes_[10];
  String unit_ = String("C");
  float min_ = 16, max_ = 31;
};
