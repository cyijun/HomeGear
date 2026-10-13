#include "SerialConsole.h"
#include "HomeGearConfig.h"
#include "DeviceInfo.h"
#include "ConfigStore.h"
#include "WifiService.h"
#include "MqttService.h"
#include "IrService.h"
#include "SensorService.h"
#include "ZigbeeBridge.h"
#include "PowerPolicy.h"
#include <WiFi.h>
#include <SPIFFS.h>
namespace homegear { namespace serial {
namespace {
void showSerialConfig()
{
  auto pendingConfig = config::pending();
  const auto &activeConfig = config::active();
  Serial.println("CONFIG BEGIN");
  Serial.printf("CONFIG firmware_version=%s\n", SOFTWARE_VERSION);
  for (int i = 0; i < static_cast<int>(HomeGearSerialConfig::Key::Invalid); ++i)
  {
    HomeGearSerialConfig::Key key = static_cast<HomeGearSerialConfig::Key>(i);
    const char *value = HomeGearSerialConfig::field(pendingConfig, key);
    Serial.printf("CONFIG %s=%s\n", HomeGearSerialConfig::keyName(key),
        HomeGearSerialConfig::isSecret(key) ? (value[0] ? "set" : "empty") : value);
  }
  Serial.printf("CONFIG unsaved=%s\n", config::dirty() ? "yes" : "no");
  Serial.printf("CONFIG active_mqtt_server=%s\n", activeConfig.mqtt_server);
  Serial.printf("CONFIG active_mqtt_port=%s\n", activeConfig.mqtt_port);
  Serial.printf("CONFIG active_homeassistant_server=%s\n", activeConfig.homeassistant_server);
  Serial.printf("CONFIG active_homeassistant_port=%s\n", activeConfig.homeassistant_port);
  wifi::printStatus();
  Serial.printf("CONFIG mqtt_connected=%s\n", mqtt::client().isConnected() ? "true" : "false");
  Serial.println("CONFIG END");
}

void dispatchSerialCommand(const char *line, size_t length)
{
  size_t prefix = 0;
  while (prefix < length && line[prefix] == ' ') ++prefix;
  if (length - prefix >= 6 && memcmp(line + prefix, "config", 6) == 0)
  {
    HomeGearSerialConfig::Command command;
    if (!HomeGearSerialConfig::parseCommand(line, length, command))
    {
      Serial.println("CONFIG ERROR INVALID_COMMAND_OR_VALUE");
      return;
    }
    if (command.operation == HomeGearSerialConfig::Operation::Show) showSerialConfig();
    else if (command.operation == HomeGearSerialConfig::Operation::Set)
    {
      if (!config::stage(command.key, command.value, command.valueLength))
      {
        Serial.println("CONFIG ERROR INVALID_VALUE");
        return;
      }
      wifi::syncConfig();
      Serial.printf("CONFIG OK STAGED %s\n", HomeGearSerialConfig::keyName(command.key));
    }
    else if (config::save() && command.operation == HomeGearSerialConfig::Operation::Apply)
    {
      Serial.println("CONFIG OK APPLY_REBOOT");
      Serial.flush();
      ESP.restart();
    }
    return;
  }
  String cmd(line);
  cmd.trim();
#if (IR_MODULE && IR_HITACHI_AC3)
  if (cmd == "hitachi_on") ir::command(true);
  else if (cmd == "hitachi_off") ir::command(false);
  else if (cmd == "hitachi_status") ir::printStatus();
  else
#endif
  if (cmd == "power status") power::printStatus();
  else if (cmd == "sensors status") sensors::printStatus();
  else if (cmd == "zigbee status") zigbee::printStatus();
  else if (cmd == "lsmod") Serial.println(device::moduleList());
  else if (cmd == "restore")
  {
    SPIFFS.format();
    WiFi.disconnect(true, true);
    ESP.restart();
  }
  else if (cmd == "reboot") ESP.restart();
  else if (length) Serial.println("SERIAL ERROR UNKNOWN_COMMAND");
}

void processSerialCommands()
{
  static char line[HomeGearSerialConfig::kMaxLineLength + 1];
  static size_t length = 0;
  static bool invalid = false;
  unsigned budget = 256;
  while (budget-- && Serial.available())
  {
    char c = static_cast<char>(Serial.read());
    if (c == '\n')
    {
      if (length && line[length - 1] == '\r') --length;
      line[length] = '\0';
      if (invalid) Serial.println("CONFIG ERROR INVALID_OR_OVERLONG_LINE");
      else dispatchSerialCommand(line, length);
      length = 0;
      invalid = false;
    }
    else if (c == '\0' || length == HomeGearSerialConfig::kMaxLineLength) invalid = true;
    else if (!invalid) line[length++] = c;
  }
}

}
void tick() { processSerialCommands(); }
}}
