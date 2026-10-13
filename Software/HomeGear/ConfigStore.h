#pragma once
#include "SerialConfig.h"
namespace homegear { namespace config {
using Values = HomeGearSerialConfig::Values;
using Key = HomeGearSerialConfig::Key;
void begin();
const Values &active();
const Values &pending();
bool stage(Key key, const char *value, std::size_t length);
bool stagePortal(const char *const values[]);
bool save();
bool dirty();
}}
