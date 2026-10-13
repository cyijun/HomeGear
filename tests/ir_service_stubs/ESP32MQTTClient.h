#pragma once
#include "Arduino.h"
#include <functional>
#include <map>
#include <vector>
class ESP32MQTTClient {
 public:
  struct Message { std::string topic, payload; int qos; bool retain; };
  bool connected = true;
  std::vector<Message> messages;
  std::map<std::string, std::function<void(const String &)>> subscriptions;
  bool isConnected() const { return connected; }
  bool publish(const String &topic, const String &payload, int qos, bool retain) {
    messages.push_back({topic.c_str(), std::string(payload.c_str(), payload.length()), qos, retain});
    return connected;
  }
  bool subscribe(const String &topic, std::function<void(const String &)> callback) {
    subscriptions[topic.c_str()] = callback;
    return true;
  }
};
