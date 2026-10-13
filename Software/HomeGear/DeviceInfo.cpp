#include "DeviceInfo.h"
#include "HomeGearConfig.h"
namespace homegear { namespace device {
namespace {
char nodeName[32] = {};
String uint64ToString(uint64_t input)
{
  String result = "";
  uint8_t base = 16;

  do
  {
    char c = input % base;
    input /= base;

    if (c < 10)
      c += '0';
    else
      c += 'A' - 10;
    result = c + result;
  } while (input);
  return result;
}
}
const String &chipId() { static const String id = uint64ToString(ESP.getEfuseMac()); return id; }
void begin() { String value = "HomeGear_" + chipId(); value.toUpperCase(); value.toCharArray(nodeName, sizeof(nodeName)); }
const char *name() { return nodeName; }
String moduleList()
{
  String modList = " / ";
#if THI_MODULE
  modList += "Temperature-Humidity-Illuminance Module / ";
#endif
#if IAQ_MODULE
  modList += "IAQ Module / ";
#endif
#if IR_MODULE
  modList += "IR Module / ";
#endif
#if RF_MODULE
  modList += "RF Module / ";
#endif
#if ZGB_MODULE
  modList += "ZigBee Module / ";
#endif
  return modList;
}

}}
