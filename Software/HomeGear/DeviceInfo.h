#pragma once
#include <Arduino.h>
namespace homegear { namespace device {
void begin();
const String &chipId();
const char *name();
String moduleList();
}}
