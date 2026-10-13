#pragma once

#include <cstdint>

class ESP32MQTTClient;
class HAMqttDiscoveryHandler;

namespace homegear {
namespace sensors {

void begin();
void tick(uint32_t now);
// Safe from MQTT callbacks; all sampling and publishing remains in tick().
void requestPublish();
void publishDiscovery(ESP32MQTTClient &client, HAMqttDiscoveryHandler &device);
void printStatus();

}  // namespace sensors
}  // namespace homegear
