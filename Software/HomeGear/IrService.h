#pragma once
#include "HomeGearConfig.h"
#include <Arduino.h>
#include <mqtt_client.h>
#if IR_MODULE
#include <IRsend.h>
#endif
namespace homegear { namespace ir {
void begin();
void tick();
void onConnected();
bool command(bool power);
void printStatus();
bool acceptMqttEvent(esp_mqtt_event_handle_t event);
#if IR_MODULE
constexpr uint16_t kIrLed = 4;
IRsend &sender();
#endif
}}
