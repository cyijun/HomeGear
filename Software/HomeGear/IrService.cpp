#include "IrService.h"
#include "MqttService.h"
#include "DeviceInfo.h"
#include "PowerPolicy.h"
#include <ArduinoJson.h>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#if (IR_MODULE && IR_HITACHI_AC3)
#include <ir_Hitachi.h>
#include "HitachiAc3Control.h"
#endif
namespace homegear { namespace ir {
#if IR_MODULE
IRsend &sender() { static IRsend output(kIrLed); return output; }
#endif
namespace {
#if (IR_MODULE && IR_HITACHI_AC3)
ESP32MQTTClient &mqttClient = mqtt::client();
HAMqttDiscoveryHandler hamdhHitachi("homegear_ir", device::chipId() + "_hitachi_ac3", "Hitachi", "Hitachi AC3 Dry", SOFTWARE_VERSION, mqtt::discovery());
HitachiAc3Control hitachiAc3;
QueueHandle_t hitachiAc3Queue = nullptr;
const uint8_t kHitachiAc3OnEvent = 1;
const uint8_t kHitachiAc3OffEvent = 2;
const uint8_t kHitachiAc3PublishEvent = 3;
const uint8_t kHitachiAc3TemperatureEvent = 4;
struct HitachiEvent { uint8_t type; uint8_t temperature; };
IRsend &irsend = sender();
// Only loop() reads or changes command state, sends IR and publishes snapshots.
String hitachiAc3Feedback()
{
  StaticJsonDocument<384> feedback;
  feedback["achitachi_mode"] = hitachiAc3.getMode();
  feedback["achitachi_temperature"] = hitachiAc3.getTemperature();
  feedback["achitachi_fan_mode"] = hitachiAc3.getFanMode();
  if (hitachiAc3.isKnown())
    feedback["achitachi_power"] = hitachiAc3.getPower();
  else
    feedback["achitachi_power"] = nullptr;
  feedback["state_source"] = hitachiAc3.getStateSource();
  // IR reports the requested setpoint, never an observed room temperature.
  feedback["achitachi_current_temperature"] = nullptr;
  feedback["current_temperature_source"] = "unavailable";
  String payload;
  serializeJson(feedback, payload);
  return payload;
}

void publishHitachiAc3State()
{
  if (mqttClient.isConnected())
    mqttClient.publish(hamdhHitachi.getStateTopic(), hitachiAc3Feedback(), 0, true);
}

bool enqueueHitachiAc3Event(uint8_t type, uint8_t temperature = 0)
{
  if (!hitachiAc3Queue)
  {
    Serial.println("HITACHI ERROR NOT_INITIALIZED");
    return false;
  }
  const HitachiEvent event = {type, temperature};
  if (xQueueSend(hitachiAc3Queue, &event, 0) != pdTRUE)
  {
    Serial.println("HITACHI ERROR COMMAND_QUEUE_FULL");
    return false;
  }
  return true;
}

bool commandHitachiAc3(bool power)
{
  return enqueueHitachiAc3Event(power ? kHitachiAc3OnEvent : kHitachiAc3OffEvent);
}

bool temperatureHitachiAc3(int degrees)
{
  if (degrees < HitachiAc3Control::kMinTemperature || degrees > HitachiAc3Control::kMaxTemperature)
  {
    Serial.println("HITACHI ERROR TEMPERATURE_OUT_OF_RANGE");
    return false;
  }
  return enqueueHitachiAc3Event(kHitachiAc3TemperatureEvent, static_cast<uint8_t>(degrees));
}

void processHitachiAc3Event()
{
  HitachiEvent event;
  if (!hitachiAc3Queue || xQueueReceive(hitachiAc3Queue, &event, 0) != pdTRUE) return;
  if (event.type == kHitachiAc3PublishEvent)
  {
    publishHitachiAc3State();
    return;
  }
  if (event.type == kHitachiAc3TemperatureEvent && !hitachiAc3.isKnown())
  {
    // An unknown power state must not become OFF merely to set a temperature.
    // Stage the target until an explicit mode command supplies absolute power.
    hitachiAc3.setTemperature(event.temperature);
  }
  else if (event.type == kHitachiAc3OnEvent || event.type == kHitachiAc3OffEvent ||
           event.type == kHitachiAc3TemperatureEvent)
  {
    power::ScopedIrClock clock;
    if (!clock.ready()) { Serial.println("HITACHI ERROR IR_CLOCK_UNAVAILABLE"); return; }
    if (event.type == kHitachiAc3TemperatureEvent)
      hitachiAc3.setTemperature(event.temperature);
    else
      hitachiAc3.setPower(event.type == kHitachiAc3OnEvent);
    irsend.sendHitachiAc3(hitachiAc3.getRaw(), HitachiAc3Control::kStateLength, 0);
  }
  else return;
  publishHitachiAc3State();
#if SERIAL_DEBUG
  Serial.print("HITACHI ");
  Serial.println(hitachiAc3Feedback());
#endif
}

void printHitachiAc3State()
{
  Serial.println(hitachiAc3Feedback());
}

void handleHitachiAc3Command(const String &payload)
{
  HitachiAc3Control::Command command;
  if (!HitachiAc3Control::parseCommand(payload.c_str(), payload.length(), command))
  {
    Serial.println("HITACHI ERROR EXPECTED_SINGLE_MODE_OR_TEMPERATURE");
    return;
  }
  if (command.type == HitachiAc3Control::Command::Type::Mode)
    commandHitachiAc3(command.power);
  else
    temperatureHitachiAc3(command.temperature);
}
#endif
}
void begin() {
#if IR_MODULE
  sender().begin();
#endif
#if (IR_MODULE && IR_HITACHI_AC3)
  hitachiAc3Queue = xQueueCreate(8, sizeof(HitachiEvent));
  if (!hitachiAc3Queue) Serial.println("HITACHI ERROR QUEUE_ALLOCATION_FAILED");
#endif
}
void tick() {
#if (IR_MODULE && IR_HITACHI_AC3)
  processHitachiAc3Event();
#endif
}
bool command(bool power) {
#if (IR_MODULE && IR_HITACHI_AC3)
  return commandHitachiAc3(power);
#else
  (void)power;
  return false;
#endif
}
bool temperature(int degrees) {
#if (IR_MODULE && IR_HITACHI_AC3)
  return temperatureHitachiAc3(degrees);
#else
  (void)degrees;
  return false;
#endif
}
void printStatus() {
#if (IR_MODULE && IR_HITACHI_AC3)
  printHitachiAc3State();
#else
  Serial.println("HITACHI DISABLED");
#endif
}
void onConnected() {
#if (IR_MODULE && IR_HITACHI_AC3)
  HAMqttDiscoveryClimate hamdhHitachiAC(hamdhHitachi);
  // Keep the existing discovery topic and unique ID when adding capabilities.
  hamdhHitachiAC.setDeviceNameMin("hitachi_ac3_25c_dry");
  hamdhHitachiAC.setModeCommandTemplate("{\"achitachi_mode\": \"{{ value }}\"}");
  hamdhHitachiAC.setModeStateTemplate("{{ value_json.achitachi_mode if value_json.achitachi_mode in ['off', 'dry'] else 'None' }}");
  hamdhHitachiAC.setTemperatureCommandTemplate("{\"achitachi_temperature\": {{ value }}}");
  hamdhHitachiAC.setTemperatureStateTemplate("{{ value_json.achitachi_temperature }}");
  hamdhHitachiAC.setMinTemp(HitachiAc3Control::kMinTemperature);
  hamdhHitachiAC.setMaxTemp(HitachiAc3Control::kMaxTemperature);
  hamdhHitachiAC.setTemperatureUnit("C");
  String hitachiModes[10] = {"off", "dry", "", "", "", "", "", "", "", ""};
  hamdhHitachiAC.setModes(hitachiModes);
  hamdhHitachiAC.construct();
  // Extend the pinned discovery library without editing its vendor files.
  DynamicJsonDocument discovery(3072);
  if (deserializeJson(discovery, hamdhHitachiAC.getMqttDiscoveryMesg()) == DeserializationError::Ok)
  {
    discovery["name"] = "Hitachi AC3 Dry";
    discovery["initial"] = static_cast<int>(HitachiAc3Control::kDefaultTemperature);
    discovery["temp_step"] = 1;
    discovery["precision"] = 1.0;
    discovery["optimistic"] = false;
    discovery["current_temperature_topic"] = hamdhHitachi.getStateTopic();
    // HA resets on the literal 'None'; an empty string or 'null' is ignored.
    discovery["current_temperature_template"] = "{{ 'None' }}";
    String payload;
    if (!discovery.overflowed() && serializeJson(discovery, payload))
      mqttClient.publish(hamdhHitachiAC.getMqttDiscoveryConfigTopic(), payload, 0, true);
    else Serial.println("HITACHI ERROR DISCOVERY_SERIALIZATION");
  }
  else Serial.println("HITACHI ERROR DISCOVERY_JSON");
  mqttClient.subscribe(hamdhHitachi.getCtrlTopic(), [](const String &payload) { handleHitachiAc3Command(payload); });
  // Schedule a loop-owned snapshot; reconnect never resets state or sends IR.
  enqueueHitachiAc3Event(kHitachiAc3PublishEvent);
#endif
}
bool acceptMqttEvent(esp_mqtt_event_handle_t event) {
#if (IR_MODULE && IR_HITACHI_AC3)
    // Reject the whole Hitachi message before the MQTT library can dispatch a
    // partial payload. The SDK reports the topic only on the first fragment.
    static bool discardHitachiMessage = false;
    if (event && event->event_id == MQTT_EVENT_DATA)
    {
      if (event->current_data_offset == 0)
      {
        discardHitachiMessage = false;
        String topic = hamdhHitachi.getCtrlTopic();
        bool isHitachiTopic = event->topic &&
            event->topic_len == static_cast<int>(topic.length()) &&
            memcmp(event->topic, topic.c_str(), topic.length()) == 0;
        if (isHitachiTopic &&
            (event->retain || event->total_data_len > static_cast<int>(HitachiAc3Control::kMaxCommandLength) ||
             event->data_len != event->total_data_len ||
             (event->data && event->data_len > 0 && memchr(event->data, '\0', event->data_len))))
        {
          discardHitachiMessage = true;
          Serial.println("HITACHI ERROR RETAINED_FRAGMENTED_OR_INVALID_COMMAND_IGNORED");
        }
      }
      if (discardHitachiMessage)
      {
        if (static_cast<int64_t>(event->current_data_offset) + event->data_len >= event->total_data_len)
          discardHitachiMessage = false;
        return false;
      }
    }
#endif
  (void)event;
  return true;
}
}}
