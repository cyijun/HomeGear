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
HAMqttDiscoveryHandler hamdhHitachi("homegear_ir", device::chipId() + "_hitachi_ac3", "Hitachi", "Hitachi AC3 25 C Dry", SOFTWARE_VERSION, mqtt::discovery());
HitachiAc3Control hitachiAc3;
QueueHandle_t hitachiAc3Queue = nullptr;
const uint8_t kHitachiAc3OnEvent = 1;
const uint8_t kHitachiAc3OffEvent = 2;
const uint8_t kHitachiAc3PublishEvent = 3;
IRsend &irsend = sender();
// Only loop() reads or changes command state, sends IR and publishes snapshots.
String hitachiAc3Feedback()
{
  StaticJsonDocument<256> feedback;
  feedback["achitachi_mode"] = hitachiAc3.getMode();
  feedback["achitachi_temperature"] = hitachiAc3.getTemperature();
  feedback["achitachi_fan_mode"] = hitachiAc3.getFanMode();
  if (hitachiAc3.isKnown())
    feedback["achitachi_power"] = hitachiAc3.getPower();
  else
    feedback["achitachi_power"] = nullptr;
  feedback["state_source"] = hitachiAc3.getStateSource();
  String payload;
  serializeJson(feedback, payload);
  return payload;
}

void publishHitachiAc3State()
{
  if (hitachiAc3.isKnown() && mqttClient.isConnected())
    mqttClient.publish(hamdhHitachi.getStateTopic(), hitachiAc3Feedback(), 0, true);
}

bool enqueueHitachiAc3Event(uint8_t event)
{
  if (!hitachiAc3Queue)
  {
    Serial.println("HITACHI ERROR NOT_INITIALIZED");
    return false;
  }
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

void processHitachiAc3Event()
{
  uint8_t event;
  if (!hitachiAc3Queue || xQueueReceive(hitachiAc3Queue, &event, 0) != pdTRUE) return;
  if (event == kHitachiAc3PublishEvent)
  {
    publishHitachiAc3State();
    return;
  }
  if (event != kHitachiAc3OnEvent && event != kHitachiAc3OffEvent) return;
  {
    power::ScopedIrClock clock;
    if (!clock.ready()) { Serial.println("HITACHI ERROR IR_CLOCK_UNAVAILABLE"); return; }
    hitachiAc3.setPower(event == kHitachiAc3OnEvent);
    irsend.sendHitachiAc3(hitachiAc3.getRaw(), HitachiAc3Control::kStateLength, 0);
  }
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
  bool power;
  if (!HitachiAc3Control::parseModeCommand(payload.c_str(), payload.length(), power))
  {
    Serial.println("HITACHI ERROR EXPECTED_ONLY_LITERAL_MODE_DRY_OR_OFF");
    return;
  }
  commandHitachiAc3(power);
}
#endif
}
void begin() {
#if IR_MODULE
  sender().begin();
#endif
#if (IR_MODULE && IR_HITACHI_AC3)
  hitachiAc3Queue = xQueueCreate(8, sizeof(uint8_t));
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
  hamdhHitachiAC.setDeviceNameMin("hitachi_ac3_25c_dry");
  hamdhHitachiAC.setModeCommandTemplate("{\"achitachi_mode\": \"{{ value }}\"}");
  hamdhHitachiAC.setModeStateTemplate("{{ value_json.achitachi_mode }}");
  String hitachiModes[10] = {"off", "dry", "", "", "", "", "", "", "", ""};
  hamdhHitachiAC.setModes(hitachiModes);
  // Fixed 25 C and low fan are attributes, not adjustable HA capabilities.
  hamdhHitachiAC.construct();
  mqttClient.publish(hamdhHitachiAC.getMqttDiscoveryConfigTopic(), hamdhHitachiAC.getMqttDiscoveryMesg(), 0, true);
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
