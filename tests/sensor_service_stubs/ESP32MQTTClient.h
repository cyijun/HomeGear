#pragma once
#include "Arduino.h"
#include <vector>
class ESP32MQTTClient {
 public:
  struct Message { std::string topic, payload; int qos; bool retain, accepted; };
  bool connected = false;
  bool accept = true;
  void (*onPublish)() = nullptr;
  std::vector<Message> messages;
  bool isConnected() const { return connected; }
  bool publish(const String &topic, const String &payload, int qos, bool retain) {
    messages.push_back({topic.c_str(), payload.c_str(), qos, retain, accept});
    if (onPublish) onPublish();
    return accept;
  }
};
