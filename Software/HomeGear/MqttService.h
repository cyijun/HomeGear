#pragma once
#include "ESP32MQTTClient.h"
#include "HAMDHandler.h"
namespace homegear { namespace mqtt {
ESP32MQTTClient &client();
HAMqttDiscoveryHandler &discovery();
void begin();
void tick();
}}
