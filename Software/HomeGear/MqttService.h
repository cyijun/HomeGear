#pragma once
#include "ESP32MQTTClient.h"
#include "HAMDHandler.h"
namespace homegear { namespace mqtt {
ESP32MQTTClient &client();
HAMqttDiscoveryHandler &discovery();
uint32_t connectionCount();
const char *clientId();
void begin();
void tick();
}}
