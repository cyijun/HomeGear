// Real SensorService still publishes an explicit unknown with both modules off.
#define THI_MODULE false
#define IAQ_MODULE false
#include "../Software/HomeGear/RoomTemperature.cpp"
#include "../Software/HomeGear/SensorService.cpp"
#include <cassert>
#include <iostream>

namespace sensor_mock {
uint32_t now = 0;
unsigned wireBegins = 0;
}
MockSerial Serial;
TwoWire Wire;
namespace homegear { namespace mqtt {
ESP32MQTTClient &client() { static ESP32MQTTClient client; return client; }
HAMqttDiscoveryHandler &discovery() { static HAMqttDiscoveryHandler device; return device; }
}}

int main() {
  using namespace homegear::sensors;
  ESP32MQTTClient &client = homegear::mqtt::client();
  client.connected = true;
  begin();
  tick(0);
  assert(sensor_mock::wireBegins == 0);
  assert(client.messages.size() == 1 && client.messages.back().payload == "None");
  assert(client.messages.back().topic == "homegear/state/room_temperature");
  assert(client.messages.back().retain && !room::reading(0).available);
  publishDiscovery(client, homegear::mqtt::discovery());
  assert(client.messages.size() == 1);  // No phantom hardware entities.
  requestPublish();
  tick(1);
  assert(client.messages.size() == 2 && client.messages.back().payload == "None");
  client.connected = false;
  requestPublish();
  tick(2);
  assert(client.messages.size() == 2);
  client.connected = true;
  tick(3);
  assert(client.messages.size() == 3 && client.messages.back().payload == "None");
  std::cout << "room_temperature_no_sensors_test: PASS\n";
}
