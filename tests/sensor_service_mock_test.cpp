// Compile with tests/sensor_service_stubs, locked ArduinoJson/src and BSEC/src
// include paths. The real SensorService.cpp and JSON serializer run on the host.
#include "../Software/HomeGear/TrackedBsec.cpp"
#include "../Software/HomeGear/RoomTemperature.cpp"
#include "../Software/HomeGear/SensorService.cpp"
#include <cassert>
#include <iostream>

namespace sensor_mock {
uint32_t now = 0;
bool shtHealthy = true, crcHealthy = true, lightsHealthy = false, bsecHealthy = true;
float temperature = 23.0f;
unsigned shtReads = 0, lightReads = 0, wireBegins = 0, bsecConfigurations = 0;
uint32_t bsecOutputMask = UINT32_MAX;
bool bsecZeroSignals = false;
int bsecAlgorithmStatus = BSEC_OK;
}

extern "C" bsec_library_return_t bsec_do_steps(const bsec_input_t *const, const uint8_t,
    bsec_output_t *outputs, uint8_t *count) {
  const uint8_t capacity = *count;
  *count = 0;
  const bsec_output_t available[] = {
    {sensor_mock::bsecZeroSignals ? 0.0f : 26.0f, BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE, 0},
    {sensor_mock::bsecZeroSignals ? 0.0f : 48.0f, BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY, 0},
    {101325.0f, BSEC_OUTPUT_RAW_PRESSURE, 0},
    {sensor_mock::bsecZeroSignals ? 0.0f : 100.0f, BSEC_OUTPUT_STATIC_IAQ, 2},
    {600.0f, BSEC_OUTPUT_CO2_EQUIVALENT, 2},
    {sensor_mock::bsecZeroSignals ? 0.0f : 0.5f, BSEC_OUTPUT_BREATH_VOC_EQUIVALENT, 2},
    {1.0f, BSEC_OUTPUT_STABILIZATION_STATUS, 0},
    {1.0f, BSEC_OUTPUT_RUN_IN_STATUS, 0}
  };
  for (const auto &output : available) {
    if ((sensor_mock::bsecOutputMask & (uint32_t(1) << output.sensor_id)) && *count < capacity)
      outputs[(*count)++] = output;
  }
  return sensor_mock::bsecAlgorithmStatus;
}
MockSerial Serial;
TwoWire Wire;
namespace homegear { namespace mqtt {
ESP32MQTTClient &client() { static ESP32MQTTClient client; return client; }
HAMqttDiscoveryHandler &discovery() { static HAMqttDiscoveryHandler device; return device; }
}}

static void tickAt(uint32_t now) { sensor_mock::now = now; homegear::sensors::tick(now); }

static std::vector<ESP32MQTTClient::Message> snapshotMessages(const ESP32MQTTClient &client) {
  std::vector<ESP32MQTTClient::Message> result;
  for (const auto &message : client.messages)
    if (message.topic == "homegear/state") result.push_back(message);
  return result;
}

static std::vector<ESP32MQTTClient::Message> roomMessages(const ESP32MQTTClient &client) {
  std::vector<ESP32MQTTClient::Message> result;
  for (const auto &message : client.messages)
    if (message.topic == "homegear/state/room_temperature") result.push_back(message);
  return result;
}

int main() {
  using namespace homegear::sensors;
  ESP32MQTTClient &client = homegear::mqtt::client();
  begin();
  assert(sensor_mock::wireBegins == 1);
  assert(sensor_mock::bsecConfigurations == 1);
  assert(bme680.requestedRate == BSEC_SAMPLE_RATE_LP);
  const uint8_t expectedConfig[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
  };
  assert(std::memcmp(bme680.config, expectedConfig, sizeof(expectedConfig)) == 0);
  tickAt(0);
  assert(snapshotMessages(client).empty());  // Offline startup cannot discard the forced snapshot.
  client.connected = true;
  tickAt(2000);
  assert(snapshotMessages(client).size() == 1);
  StaticJsonDocument<1024> json;
  assert(!deserializeJson(json, snapshotMessages(client).back().payload));
  assert(json["thim_temperature"].as<std::string>() == "23.00");
  assert(json.containsKey("thim_humidity"));  // Both BH1750 failures cannot block SHTC3.
  assert(json.containsKey("iaqm_pressure"));
  assert(!json.containsKey("thim_illuminance"));
  assert(!json.containsKey("iaqm_iaq"));  // Six-minute gas warmup remains in force.
  assert(snapshotMessages(client).back().qos == 0 && snapshotMessages(client).back().retain);
  assert(roomMessages(client).size() == 1 && roomMessages(client).back().payload == "23.00");
  assert(roomMessages(client).back().qos == 0 && roomMessages(client).back().retain);
  assert(room::reading(2000).available && std::string(room::reading(2000).source) == "shtc3");

  sensor_mock::temperature = 25.0f;
  tickAt(4000);
  assert(snapshotMessages(client).size() == 1);  // Normal changes are coalesced for 15 seconds.
  requestPublish();
  tickAt(4001);
  assert(snapshotMessages(client).size() == 2);
  json.clear();
  assert(!deserializeJson(json, snapshotMessages(client).back().payload));
  assert(json["thim_temperature"].as<std::string>() == "25.00");
  assert(json.containsKey("thim_humidity") && json.containsKey("iaqm_pressure"));

  client.accept = false;
  requestPublish();
  tickAt(5000);
  assert(snapshotMessages(client).size() == 3 && !snapshotMessages(client).back().accepted);
  client.accept = true;
  tickAt(5999);
  assert(snapshotMessages(client).size() == 3);
  sensor_mock::temperature = 27.0f;
  tickAt(6000);
  assert(snapshotMessages(client).size() == 4);
  json.clear();
  assert(!deserializeJson(json, snapshotMessages(client).back().payload));
  assert(json["thim_temperature"].as<std::string>() == "27.00");

  // A callback during the synchronous publish must remain pending after commit.
  const size_t roomCountBeforeCallback = roomMessages(client).size();
  client.onPublish = requestPublish;
  requestPublish();
  tickAt(6001);
  assert(snapshotMessages(client).size() == 5);
  assert(roomMessages(client).size() == roomCountBeforeCallback + 1);
  client.onPublish = nullptr;
  tickAt(6002);
  assert(snapshotMessages(client).size() == 6);
  assert(roomMessages(client).size() == roomCountBeforeCallback + 2);

  sensor_mock::lightsHealthy = true;
  tickAt(8000);  // Independently initialize the lights.
  tickAt(10000); // Then read their one-shot results.
  assert(sensor_mock::lightReads == 2);
  requestPublish();
  tickAt(360000);
  json.clear();
  assert(!deserializeJson(json, snapshotMessages(client).back().payload));
  assert(json.containsKey("thim_illuminance") && json.containsKey("iaqm_iaq"));
  assert(json["iaqm_breathVocEquivalent"].as<std::string>() == "500.00");
  assert(json["iaqm_accuracy"].as<int>() == 2);

  // GAS_VALID=false can yield only pressure/status. Cleared-but-absent fields
  // must preserve their last known readings and calibration, never become 0.
  sensor_mock::bsecOutputMask = uint32_t(1) << BSEC_OUTPUT_RAW_PRESSURE;
  requestPublish();
  tickAt(363000);
  assert(bme680.temperature == 0.0f && bme680.staticIaq == 0.0f && bme680.breathVocEquivalent == 0.0f);
  assert(trackedBsec::hasOutput(BSEC_OUTPUT_RAW_PRESSURE));
  assert(!trackedBsec::hasOutput(BSEC_OUTPUT_STATIC_IAQ));
  json.clear();
  assert(!deserializeJson(json, snapshotMessages(client).back().payload));
  assert(json["iaqm_temperature"].as<std::string>() == "26.00");
  assert(json["iaqm_humidity"].as<std::string>() == "48.00");
  assert(json["iaqm_iaq"].as<std::string>() == "100.00");
  assert(json["iaqm_breathVocEquivalent"].as<std::string>() == "500.00");
  assert(json["iaqm_accuracy"].as<int>() == 2);
  assert(json["iaqm_stabilizationStatus"].as<int>() == 1);

  // Presence disambiguates real, newly measured zeros from absent outputs.
  sensor_mock::bsecOutputMask = UINT32_MAX;
  sensor_mock::bsecZeroSignals = true;
  requestPublish();
  tickAt(366000);
  json.clear();
  assert(!deserializeJson(json, snapshotMessages(client).back().payload));
  assert(json["iaqm_temperature"].as<std::string>() == "0.00");
  assert(json["iaqm_humidity"].as<std::string>() == "0.00");
  assert(json["iaqm_iaq"].as<std::string>() == "0.00");
  assert(json["iaqm_breathVocEquivalent"].as<std::string>() == "0.00");
  assert(json["iaqm_accuracy"].as<int>() == 2);

  // No gateway invocation and a failed algorithm call both clear presence.
  tickAt(366001);
  assert(trackedBsec::outputMask() == 0);
  sensor_mock::bsecAlgorithmStatus = -1;
  tickAt(369000);
  assert(trackedBsec::outputMask() == 0);
  sensor_mock::bsecAlgorithmStatus = BSEC_OK;
  sensor_mock::bsecZeroSignals = false;

  client.messages.clear();
  publishDiscovery(client, homegear::mqtt::discovery());
  assert(client.messages.size() == 9);
  for (const auto &message : client.messages) assert(message.qos == 0 && message.retain);
  printStatus();
  assert(Serial.output.find("SENSORS BEGIN") != std::string::npos);
  assert(Serial.output.find("SENSORS bsec_config=33v_3s_4d") != std::string::npos);
  assert(Serial.output.find("SENSORS END") != std::string::npos);

  // No valid first sample must produce no empty payload or initialized fake value.
  client.messages.clear();
  sensor_mock::now = 400000;
  sensor_mock::crcHealthy = false;
  sensor_mock::lightsHealthy = false;
  sensor_mock::bsecHealthy = false;
  begin();
  tickAt(400000);
  assert(snapshotMessages(client).empty());
  assert(roomMessages(client).size() == 1 && roomMessages(client).back().payload == "None");
  assert(!room::reading(400000).available);
  sensor_mock::crcHealthy = true;
  tickAt(402000);
  assert(snapshotMessages(client).size() == 1);
  json.clear();
  assert(!deserializeJson(json, snapshotMessages(client).back().payload));
  assert(json.size() == 2 && json.containsKey("thim_temperature") && json.containsKey("thim_humidity"));
  assert(roomMessages(client).size() == 2 && roomMessages(client).back().payload == "27.00");
  // Failed hardware or CRC readings immediately clear the independent room
  // topic, even while the existing sensor snapshot keeps its historical fields.
  sensor_mock::shtHealthy = false;
  tickAt(404000);
  assert(roomMessages(client).back().payload == "None" && !room::reading(404000).available);
  sensor_mock::shtHealthy = true;
  sensor_mock::temperature = 0.0f;
  tickAt(406000);
  assert(roomMessages(client).back().payload == "0.00" && room::reading(406000).available);
  sensor_mock::temperature = NAN;
  tickAt(408000);
  assert(roomMessages(client).back().payload == "None" && !room::reading(408000).available);
  sensor_mock::temperature = 24.0f;
  tickAt(410000);
  assert(roomMessages(client).back().payload == "24.00");
  sensor_mock::crcHealthy = false;
  tickAt(412000);
  assert(roomMessages(client).back().payload == "None");

  // On startup a partial frame must not create never-measured zero T/RH/gas.
  client.messages.clear();
  sensor_mock::now = 500000;
  sensor_mock::crcHealthy = false;
  sensor_mock::bsecHealthy = true;
  sensor_mock::bsecOutputMask = uint32_t(1) << BSEC_OUTPUT_RAW_PRESSURE;
  begin();
  tickAt(500000);
  assert(snapshotMessages(client).size() == 1);
  json.clear();
  assert(!deserializeJson(json, snapshotMessages(client).back().payload));
  assert(json.containsKey("iaqm_pressure"));
  assert(roomMessages(client).size() == 1 && roomMessages(client).back().payload == "None");
  assert(!room::reading(500000).available);  // Never select the heated BME680 fallback.
  assert(!json.containsKey("iaqm_temperature") && !json.containsKey("iaqm_humidity"));
  assert(!json.containsKey("iaqm_accuracy") && !json.containsKey("iaqm_stabilizationStatus"));
  requestPublish();
  tickAt(860000);  // Warmup ending must not manufacture absent gas values.
  json.clear();
  assert(!deserializeJson(json, snapshotMessages(client).back().payload));
  assert(!json.containsKey("iaqm_iaq") && !json.containsKey("iaqm_breathVocEquivalent"));
  sensor_mock::bsecOutputMask = UINT32_MAX;
  requestPublish();
  tickAt(863000);
  json.clear();
  assert(!deserializeJson(json, snapshotMessages(client).back().payload));
  assert(json["iaqm_temperature"].as<std::string>() == "26.00");
  assert(roomMessages(client).back().payload == "None" && !room::reading(863000).available);
  std::cout << "sensor_service_mock_test: PASS\n";
}
