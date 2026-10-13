#pragma once
#include <Arduino.h>
namespace homegear { namespace wifi {
void begin();
void tick();
void syncConfig();
void printStatus();
}}
