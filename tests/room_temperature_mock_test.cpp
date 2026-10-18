// The real publisher runs with the existing MQTT/Arduino host stubs.
#include "../Software/HomeGear/RoomTemperature.cpp"
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>

namespace sensor_mock { uint32_t now = 0; }
namespace homegear { namespace mqtt {
ESP32MQTTClient &client() { static ESP32MQTTClient client; return client; }
HAMqttDiscoveryHandler &discovery() { static HAMqttDiscoveryHandler device; return device; }
}}

namespace room = homegear::sensors::room;

static ESP32MQTTClient &reset() {
  room::reset();
  ESP32MQTTClient &client = homegear::mqtt::client();
  client.connected = true;
  client.accept = true;
  client.onPublish = nullptr;
  client.messages.clear();
  return client;
}

static void startupAndValidity() {
  ESP32MQTTClient &client = reset();
  // No sensor attached (or THI_MODULE disabled) still clears a retained value.
  assert(!room::reading(0).available && std::isnan(room::reading(0).celsius));
  room::tick(0);
  assert(client.messages.size() == 1 && client.messages.back().payload == "None");
  assert(client.messages.back().topic == "homegear/state/room_temperature");
  assert(client.messages.back().retain && client.messages.back().qos == 0);
  room::tick(1);
  assert(client.messages.size() == 1);
  room::update(0.0f, 2);
  room::tick(2);
  assert(room::reading(2).available && room::reading(2).celsius == 0.0f);
  assert(std::strcmp(room::reading(2).source, "shtc3") == 0);
  assert(client.messages.back().payload == "0.00");
  const float invalid[] = {NAN, INFINITY, -INFINITY, -40.01f, 125.01f};
  uint32_t now = 3;
  for (float value : invalid) {
    room::update(value, now);
    room::tick(now++);
    assert(!room::reading(now).available && client.messages.back().payload == "None");
    room::update(24.0f, now);
    room::tick(now++);
    assert(room::reading(now).available && client.messages.back().payload == "24.00");
  }
  room::invalidate();
  room::tick(now);
  assert(!room::reading(now).available && client.messages.back().payload == "None");
}

static void cadenceAndHeartbeat() {
  ESP32MQTTClient &client = reset();
  room::update(23.0f, 0);
  room::tick(0);
  room::update(23.2f, 2000);
  room::tick(2000);
  assert(client.messages.size() == 1);
  room::update(23.2f, 14999);
  room::tick(14999);
  assert(client.messages.size() == 1);
  room::tick(15000);
  assert(client.messages.size() == 2 && client.messages.back().payload == "23.20");
  room::update(23.25f, 30000);
  room::tick(30000);
  assert(client.messages.size() == 2);  // Compare with the last successful value.
  room::update(23.25f, 314999);
  room::tick(314999);
  assert(client.messages.size() == 2);
  room::tick(315000);
  assert(client.messages.size() == 3 && client.messages.back().payload == "23.25");
}

static void stalenessAndRollover() {
  ESP32MQTTClient &client = reset();
  room::update(22.0f, 1000);
  room::tick(1000);
  assert(room::reading(11000).available);
  room::tick(11001);
  assert(!room::reading(11001).available && client.messages.back().payload == "None");
  room::update(22.0f, 11002);
  room::tick(11002);
  assert(client.messages.back().payload == "22.00");  // Recovery bypasses 15 seconds.

  reset();
  const uint32_t sampledAt = UINT32_MAX - 5000;
  room::update(-10.0f, sampledAt);
  room::tick(sampledAt);
  assert(room::reading(sampledAt + uint32_t(10000)).available);
  room::tick(sampledAt + uint32_t(10001));
  assert(!room::reading(sampledAt + uint32_t(10001)).available);
  assert(client.messages.back().payload == "None");
}

static void forceAndRetry() {
  ESP32MQTTClient &client = reset();
  client.connected = false;
  room::update(24.0f, 0);
  room::tick(0, true);
  assert(client.messages.empty());
  client.connected = true;
  room::tick(1);
  assert(client.messages.size() == 1 && client.messages.back().payload == "24.00");
  room::tick(2, true);  // Reconnect/pubdata synchronizes unchanged readings.
  assert(client.messages.size() == 2);
  client.connected = false;
  room::tick(3, true);
  client.connected = true;
  room::tick(4);
  assert(client.messages.size() == 3);

  reset();
  room::update(24.0f, 0);
  client.accept = false;
  room::tick(0, true);
  assert(client.messages.size() == 1 && !client.messages.back().accepted);
  room::update(25.0f, 1);
  room::tick(1, true);
  client.accept = true;
  room::tick(999);
  assert(client.messages.size() == 1);
  room::tick(1000);
  assert(client.messages.size() == 2 && client.messages.back().accepted);
  assert(client.messages.back().payload == "25.00");
  room::tick(1001);
  assert(client.messages.size() == 2);
  room::update(26.0f, 16000);
  client.accept = false;
  room::tick(16000);
  assert(client.messages.size() == 3 && !client.messages.back().accepted);
  room::update(27.0f, 17000);
  client.accept = true;
  room::tick(17000);
  assert(client.messages.size() == 4 && client.messages.back().payload == "27.00");
  room::update(27.05f, 32000);
  room::tick(32000);
  assert(client.messages.size() == 4);  // A failed attempt never moved the baseline.

  room::invalidate();
  client.accept = false;
  room::tick(32001);
  assert(client.messages.back().payload == "None" && !client.messages.back().accepted);
  client.accept = true;
  room::tick(33000);
  assert(!client.messages.back().accepted);
  room::tick(33001);
  assert(client.messages.back().payload == "None" && client.messages.back().accepted);
}

int main() {
  startupAndValidity();
  cadenceAndHeartbeat();
  stalenessAndRollover();
  forceAndRetry();
  std::cout << "room_temperature_mock_test: PASS\n";
}
