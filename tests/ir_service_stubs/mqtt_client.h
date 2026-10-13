#pragma once
#include <cstdint>
enum esp_mqtt_event_id_t { MQTT_EVENT_CONNECTED = 1, MQTT_EVENT_DATA = 2 };
struct esp_mqtt_event_t {
  esp_mqtt_event_id_t event_id = MQTT_EVENT_DATA;
  char *topic = nullptr;
  int topic_len = 0;
  char *data = nullptr;
  int data_len = 0;
  int total_data_len = 0;
  int current_data_offset = 0;
  bool retain = false;
};
using esp_mqtt_event_handle_t = esp_mqtt_event_t *;
