#include "MqttService.h"
#include "HomeGearConfig.h"
#include "DeviceInfo.h"
#include "ConfigStore.h"
#include "SensorService.h"
#include "IrService.h"
#include "LegacyRemotes.h"
#include <WiFi.h>
#include <SPIFFS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
namespace homegear { namespace mqtt {
ESP32MQTTClient &client() { static ESP32MQTTClient value; return value; }
HAMqttDiscoveryHandler &discovery() { static HAMqttDiscoveryHandler value(PLATFORM, device::chipId(), MANUFACTURER, DEVICE_MODEL, SOFTWARE_VERSION); return value; }
namespace {
String mqttUri;
bool mqttStarted = false;
bool initialized = false;
QueueHandle_t systemQueue = nullptr;
const String &availability() { static const String value = discovery().getAvailabilityTopic(); return value; }
void queueSystem(uint8_t command) {
  if (!systemQueue || xQueueSend(systemQueue, &command, 0) != pdTRUE) Serial.println("MQTT ERROR SYSTEM_QUEUE_FULL");
}
void mqttStart()
{
  client().setMaxPacketSize(2048);
#if SERIAL_DEBUG
  client().enableDebuggingMessages();
#endif
  // esp32mqttclient doesn't have OTA function
  // client().enableHTTPWebUpdater("admin", "homegearota", "/ota");
  client().enableLastWillMessage(availability().c_str(), "offline", true);
  const auto &activeConfig = config::active();
  // Set the WiFi and MQTT information that we loaded from global param
  client().setMqttClientName(device::name());
  mqttUri = "mqtt://" + String(activeConfig.mqtt_server) + ":" + String(activeConfig.mqtt_port);
  client().setURI(mqttUri.c_str(), activeConfig.mqtt_user, activeConfig.mqtt_pwd);
  // Legacy ESP32MQTTClient 0.1.0 always returns false even after starting.
  // Guard the start attempt once; its internal reconnect handles network loss.
  mqttStarted = true;
  client().loopStart();
}

}
void begin() {
  if (!initialized) {
    initialized = true;
    systemQueue = xQueueCreate(8, sizeof(uint8_t));
    if (!systemQueue) Serial.println("MQTT ERROR SYSTEM_QUEUE_ALLOCATION_FAILED");
  }
  if (!mqttStarted && WiFi.status() == WL_CONNECTED && config::active().mqtt_server[0]) mqttStart();
}
void tick() {
  begin();
  uint8_t command;
  if (systemQueue && xQueueReceive(systemQueue, &command, 0) == pdTRUE) {
    if (command == 1) ESP.restart();
    else if (command == 2) { SPIFFS.format(); WiFi.disconnect(true); ESP.restart(); }
    else if (command == 3) sensors::requestPublish();
    else if (command == 4) legacy::setRfBaud();
  }
}
void connected() {
  client().publish(availability().c_str(), "online", 0, true);
  client().subscribe(discovery().getCmdTopic(), [](const String &payload) {
    if (payload == "reboot") queueSystem(1);
    else if (payload == "restore") queueSystem(2);
    else if (payload == "pubdata") queueSystem(3);
#if RF_MODULE
    else if (payload == "setrfmodbaud115200") queueSystem(4);
#endif
  });
  sensors::publishDiscovery(client(), discovery());
  sensors::requestPublish();
  ir::onConnected();
  legacy::onConnected();
}
}}
// These global callbacks are required by the pinned ESP32MQTTClient API.
void onConnectionEstablishedCallback(esp_mqtt_client_handle_t client) { (void)client; homegear::mqtt::connected(); }
esp_err_t handleMQTT(esp_mqtt_event_handle_t event) {
  if (!event || !homegear::ir::acceptMqttEvent(event)) return ESP_OK;
  homegear::mqtt::client().onEventCallback(event);
  return ESP_OK;
}
