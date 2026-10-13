// Runs the real IR service, control parser/encoder and pinned JSON serializer.
// Only queues, MQTT transport, HA helper, clock and IR hardware are substituted.
#include "../Software/HomeGear/IrService.cpp"
#include <cassert>
#include <iostream>

MockSerial Serial;
namespace ir_mock {
uint32_t now = 0;
bool clockReady = true;
unsigned clockDepth = 0;
unsigned clockUses = 0;
}
namespace homegear { namespace mqtt {
ESP32MQTTClient &client() { static ESP32MQTTClient client; return client; }
HAMqttDiscoveryHandler &discovery() { static HAMqttDiscoveryHandler device; return device; }
}}
namespace homegear { namespace device {
const String &chipId() { static String id("TESTCHIP"); return id; }
}}
namespace homegear { namespace power {
ScopedIrClock::ScopedIrClock() : previousMhz_(160), ready_(ir_mock::clockReady) { ++ir_mock::clockDepth; ++ir_mock::clockUses; }
ScopedIrClock::~ScopedIrClock() { assert(previousMhz_ == 160 && ir_mock::clockDepth == 1); --ir_mock::clockDepth; }
}}

static std::string controlTopic() {
  auto &subscriptions = homegear::mqtt::client().subscriptions;
  assert(subscriptions.size() == 1);
  return subscriptions.begin()->first;
}

static bool deliver(const std::string &payload, bool retained = false) {
  const std::string topic = controlTopic();
  esp_mqtt_event_t event;
  event.topic = const_cast<char *>(topic.data()); event.topic_len = static_cast<int>(topic.size());
  event.data = const_cast<char *>(payload.data()); event.data_len = event.total_data_len = static_cast<int>(payload.size());
  event.retain = retained;
  if (!homegear::ir::acceptMqttEvent(&event)) return false;
  homegear::mqtt::client().subscriptions.begin()->second(String(payload.data(), payload.size()));
  return true;
}

static StaticJsonDocument<512> state() {
  const auto &messages = homegear::mqtt::client().messages;
  for (auto it = messages.rbegin(); it != messages.rend(); ++it) {
    if (it->topic.find("/state") == std::string::npos) continue;
    StaticJsonDocument<512> result;
    assert(!deserializeJson(result, it->payload));
    assert(it->qos == 0 && it->retain);
    return result;
  }
  assert(false && "Expected a state publication");
  return StaticJsonDocument<512>();
}

static void checkFrame(bool on, int target) {
  const auto &frame = homegear::ir::sender().frames.back();
  assert(frame.size() == 23);
  assert(frame[15] == static_cast<uint8_t>((target - 7) * 2 + 1));
  assert(frame[16] == static_cast<uint8_t>(~frame[15]));
  assert(frame[17] == 0x14 && frame[18] == 0xEB); // Dry, low fan.
  assert(frame[21] == (on ? 0x90 : 0xA0));
  assert(frame[22] == static_cast<uint8_t>(~frame[21]));
}

int main() {
  using namespace homegear::ir;
  auto &transport = homegear::mqtt::client();
  auto &output = sender();
  begin();
  assert(output.beginCalls == 1 && output.frames.empty());
  onConnected();
  assert(output.frames.empty());
  bool discoverySeen = false;
  std::string initialDiscoveryTopic;
  for (const auto &message : transport.messages) {
    assert(message.qos == 0 && message.retain);
    if (message.topic.find("/config") != std::string::npos) {
      StaticJsonDocument<4096> discovery;
      assert(!deserializeJson(discovery, message.payload));
      assert(discovery["min_temp"].as<int>() == 17 && discovery["max_temp"].as<int>() == 30);
      assert(discovery["initial"].as<int>() == 25);
      assert(discovery["temp_step"].as<int>() == 1 && discovery["precision"].as<int>() == 1);
      assert(discovery.containsKey("optimistic") && !discovery["optimistic"].as<bool>());
      assert(discovery.containsKey("current_temperature_topic"));
      assert(discovery["current_temperature_template"].as<std::string>().find("None") != std::string::npos);
      initialDiscoveryTopic = message.topic;
      discoverySeen = true;
    }
  }
  assert(discoverySeen);
  tick();
  auto snapshot = state();
  assert(snapshot["achitachi_mode"].as<std::string>() == "unknown");
  assert(snapshot["achitachi_power"].isNull());
  assert(snapshot["achitachi_temperature"].as<int>() == 25);
  assert(snapshot.containsKey("achitachi_current_temperature") && snapshot["achitachi_current_temperature"].isNull());
  assert(snapshot["state_source"].as<std::string>() == "unknown");
  assert(output.frames.empty() && ir_mock::clockUses == 0);

  assert(deliver("{\"achitachi_temperature\":24.0}"));
  assert(output.frames.empty() && state()["achitachi_temperature"].as<int>() == 25);
  tick();
  assert(output.frames.empty());
  snapshot = state();
  assert(snapshot["achitachi_temperature"].as<int>() == 24);
  assert(snapshot["achitachi_mode"].as<std::string>() == "unknown" && snapshot["achitachi_power"].isNull());
  assert(ir_mock::clockUses == 0); // Staging must not switch the hardware clock either.

  assert(deliver("{\"achitachi_mode\":\"dry\"}"));
  assert(output.frames.empty()); // Callback enqueues only.
  tick();
  assert(output.frames.size() == 1); checkFrame(true, 24);
  assert(temperature(23));
  assert(output.frames.size() == 1 && state()["achitachi_temperature"].as<int>() == 24);
  tick();
  assert(output.frames.size() == 2); checkFrame(true, 23);
  assert(state()["achitachi_mode"].as<std::string>() == "dry");
  assert(state()["achitachi_power"].as<bool>());

  assert(command(false)); tick(); checkFrame(false, 23);
  assert(deliver("{\"achitachi_temperature\":\"22.0\"}"));
  const size_t beforeOffTemperature = output.frames.size();
  tick();
  assert(output.frames.size() == beforeOffTemperature + 1); checkFrame(false, 22);
  assert(state()["achitachi_mode"].as<std::string>() == "off" && !state()["achitachi_power"].as<bool>());
  assert(command(true)); tick(); checkFrame(true, 22);

  // HomeKit's fractional targets are rounded by the real parser, then both the
  // feedback and actual IR frame must report the same integral setpoint.
  assert(deliver("{\"achitachi_temperature\":24.4}")); tick(); checkFrame(true, 24);
  assert(state()["achitachi_temperature"].as<int>() == 24);
  assert(deliver("{\"achitachi_temperature\":\"24.5\"}")); tick(); checkFrame(true, 25);
  assert(state()["achitachi_temperature"].as<int>() == 25);
  assert(temperature(22)); tick(); checkFrame(true, 22);

  // Clock failure must not commit either power or target temperature changes.
  ir_mock::clockReady = false;
  const size_t beforeClockFailure = output.frames.size();
  assert(command(false)); tick();
  assert(temperature(21)); tick();
  assert(output.frames.size() == beforeClockFailure);
  onConnected(); tick();
  assert(output.frames.size() == beforeClockFailure);
  snapshot = state();
  assert(snapshot["achitachi_mode"].as<std::string>() == "dry");
  assert(snapshot["achitachi_temperature"].as<int>() == 22);
  ir_mock::clockReady = true;

  // SDK gate rejects whole messages before subscription dispatch.
  const size_t beforeInvalid = output.frames.size();
  assert(!deliver("{\"achitachi_mode\":\"off\"}", true));
  std::string nulPayload = "{\"achitachi_mode\":\"off\"}";
  nulPayload.push_back('\0'); nulPayload += "trailing";
  assert(!deliver(nulPayload));
  assert(!deliver(std::string(HitachiAc3Control::kMaxCommandLength + 1, ' ')));
  esp_mqtt_event_t fragment;
  const std::string topic = controlTopic();
  std::string first = "{\"achitachi_mode\":";
  fragment.topic = const_cast<char *>(topic.data()); fragment.topic_len = static_cast<int>(topic.size());
  fragment.data = const_cast<char *>(first.data()); fragment.data_len = static_cast<int>(first.size());
  fragment.total_data_len = fragment.data_len + 6;
  assert(!acceptMqttEvent(&fragment));
  fragment.topic = nullptr; fragment.topic_len = 0;
  fragment.current_data_offset = fragment.data_len; fragment.data_len = 6;
  assert(!acceptMqttEvent(&fragment));
  tick();
  assert(output.frames.size() == beforeInvalid);
  const std::string wrongTopic = "unrelated/control";
  fragment = esp_mqtt_event_t();
  fragment.topic = const_cast<char *>(wrongTopic.data()); fragment.topic_len = static_cast<int>(wrongTopic.size());
  fragment.data = const_cast<char *>(first.data()); fragment.data_len = fragment.total_data_len = static_cast<int>(first.size());
  fragment.retain = true;
  assert(acceptMqttEvent(&fragment)); // Filter does not swallow unrelated traffic.

  // Strict parser failures are exercised through the actual subscribed callback.
  for (const char *invalid : {"{\"achitachi_temperature\":16}", "{\"achitachi_temperature\":31}",
       "{\"achitachi_temperature\":16.9}", "{\"achitachi_temperature\":30.1}", "{\"achitachi_mode\":\"cool\"}",
       "{\"achitachi_mode\":\"dry\"}trailing", "{\"achitachi_mode\":\"off\",\"achitachi_temperature\":22}"}) {
    assert(deliver(invalid)); tick();
  }
  assert(!temperature(16) && !temperature(31));
  assert(output.frames.size() == beforeInvalid);

  // Typed queue must preserve mixed-event targets and power when all eight slots fill.
  assert(command(false)); assert(temperature(17)); assert(command(true)); assert(temperature(18));
  assert(command(false)); assert(temperature(19)); assert(command(true)); assert(temperature(20));
  assert(!command(false) && !temperature(21));
  assert(output.frames.size() == beforeInvalid);
  const bool expectedPower[] = {false, false, true, true, false, false, true, true};
  const int expectedTarget[] = {22, 17, 17, 18, 18, 19, 19, 20};
  for (size_t i = 0; i < 8; ++i) { tick(); checkFrame(expectedPower[i], expectedTarget[i]); }
  assert(output.frames.size() == beforeInvalid + 8);
  assert(state()["achitachi_temperature"].as<int>() == 20);

  // A command arriving during send is deferred to the following loop turn.
  output.duringSend = [] { assert(homegear::ir::temperature(21)); };
  assert(command(false));
  const size_t beforeReentrantArrival = output.frames.size();
  tick();
  assert(output.frames.size() == beforeReentrantArrival + 1); checkFrame(false, 20);
  output.duringSend = nullptr;
  tick();
  assert(output.frames.size() == beforeReentrantArrival + 2); checkFrame(false, 21);
  assert(output.activeSends == 0 && ir_mock::clockDepth == 0);

  const size_t beforeReconnect = output.frames.size();
  onConnected(); tick();
  assert(output.frames.size() == beforeReconnect);
  bool reconnectDiscoverySeen = false;
  for (auto it = transport.messages.rbegin(); it != transport.messages.rend(); ++it) {
    if (it->topic.find("/config") != std::string::npos) {
      assert(it->topic == initialDiscoveryTopic);
      reconnectDiscoverySeen = true;
      break;
    }
  }
  assert(reconnectDiscoverySeen);
  assert(state()["achitachi_mode"].as<std::string>() == "off");
  assert(state()["achitachi_temperature"].as<int>() == 21);
  std::cout << "ir_service_mock_test: PASS\n";
}
