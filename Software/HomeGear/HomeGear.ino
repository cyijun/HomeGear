/**
 *  @file HomeGear.ino
 *  @author Chen Yijun
 *  @brief
 *  @version 1.0
 *  @date 2022-02-23
 *
 *  Copyright 2022 Chen Yijun
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <Arduino.h>
#include <assert.h>
#include <Wire.h>
#include <FS.h>
#include <WiFiManager.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "SerialConfig.h"
#include "HAMDHandler.h"
#include "ESP32MQTTClient.h"

//***** Config *****

#define PLATFORM "HomeGear"
#define HOMEGEAR_SERIAL_NUMBER chipId
#define MANUFACTURER "Chen Yijun"
#define DEVICE_MODEL "HomeGear Scalable Hardware Stack"
#define SOFTWARE_VERSION "HomeGear 1.0"

#define THI_MODULE true
#define IAQ_MODULE true
#define IR_MODULE true
#define RF_MODULE false
#define ZGB_MODULE true

#define SERIAL_DEBUG true

#define IR_GREE_AC false
#define IR_SINGFUN_FAN false
#define IR_MIDEA_AC false
#define IR_COOLIX_AC false
#define IR_HAIER_AC false
#define IR_HITACHI_AC3 true
#define RF_PHILIPS_FAN_LIGHT false

//***** Basic *****
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
#define ESP_getChipId() uint64ToString(ESP.getEfuseMac())
const String chipId = ESP_getChipId();
char nodename[32];

unsigned int timeDrivenPubInterval = 300;
unsigned long lastPubMs;
unsigned long startLoopMs;

void dataShift(float data[2])
{
  data[0] = data[1];
}

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

//***** Modules *****

HAMqttDiscoveryHandler hamdhHG(PLATFORM, HOMEGEAR_SERIAL_NUMBER, MANUFACTURER, DEVICE_MODEL, SOFTWARE_VERSION);
String avtStr = hamdhHG.getAvailabilityTopic();
const char *avt = avtStr.c_str();

#if THI_MODULE
bool thimEvent = false;
bool thimHasNewData = false;
unsigned long lastReadTHIMs = 0;
// SHTC3
#include "SparkFun_SHTC3.h"
SHTC3 shtc3;

// BH1750*2
#include <BH1750.h>
BH1750 bh1750_1;
BH1750 bh1750_2;

float thim_temperature[2] = {-100, 1};
float thim_humidity[2] = {-100, 1};
float thim_illuminance[2] = {-100, 1};
#endif

#if IAQ_MODULE // BME680
bool bme680HasNewData = false;
bool iaqIniting = true;
bool iaqmEvent = false;
#include "bsec.h"
Bsec bme680;

// use 2 elements arrays to calculate the differences of new data
float iaqm_temperature[2] = {-100, 1};
float iaqm_humidity[2] = {-100, 1};
float iaqm_pressure[2];
float iaqm_iaq[2] = {-100, 1};
float iaqm_co2Equivalent[2];
float iaqm_breathVocEquivalent[2];
#endif

#if IR_MODULE
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include "IR_Code.h"
const uint16_t kIrLed = 4;
IRsend irsend(kIrLed);

#if IR_HITACHI_AC3
#include <ir_Hitachi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "HitachiAc3Control.h"
HAMqttDiscoveryHandler hamdhHitachi("homegear_ir", chipId + "_hitachi_ac3", "Hitachi", "Hitachi AC3 25 C Dry", SOFTWARE_VERSION, hamdhHG);
HitachiAc3Control hitachiAc3;
QueueHandle_t hitachiAc3Queue = nullptr;
const uint8_t kHitachiAc3OnEvent = 1;
const uint8_t kHitachiAc3OffEvent = 2;
const uint8_t kHitachiAc3PublishEvent = 3;
#endif

#if IR_SINGFUN_FAN
HAMqttDiscoveryHandler hamdhSF("homegear_ir", "00001", "SingFun", "SingFun Floor Fan", SOFTWARE_VERSION, hamdhHG);
#endif

#if IR_GREE_AC
#include <ir_Gree.h>
HAMqttDiscoveryHandler hamdhGree("homegear_ir", "ac001", "Gree", "Gree AC", SOFTWARE_VERSION, hamdhHG);
IRGreeAC greeAC(kIrLed);

void setGreeACDefaultState()
{
  greeAC.off();
  greeAC.setFan(1);
  // kGreeAuto, kGreeDry, kGreeCool, kGreeFan, kGreeHeat
  greeAC.setMode(kGreeCool);
  greeAC.setTemp(26); // 16-30C
  greeAC.setSwingVertical(false, kGreeSwingAuto);
  greeAC.setSwingHorizontal(false);
  greeAC.setXFan(false);
  greeAC.setLight(false);
  greeAC.setSleep(false);
  greeAC.setTurbo(false);
}
#endif

#if IR_MIDEA_AC
#include <ir_Midea.h>
HAMqttDiscoveryHandler hamdhMidea("homegear_ir", "ac001", "Midea", "Midea AC", SOFTWARE_VERSION, hamdhHG);
IRMideaAC mideaAC(kIrLed);

void setMideaACDefaultState()
{
  mideaAC.off();
  mideaAC.setFan(0);
  mideaAC.setMode(kMideaACCool);
  mideaAC.setTemp(26); // 16-30C
  mideaAC.setSwingVToggle(false);
  mideaAC.setTurboToggle(false);
  mideaAC.setLightToggle(false);
  mideaAC.setQuiet(false);
  mideaAC.setSleep(false);
}
#endif

#if IR_COOLIX_AC
#include <ir_Coolix.h>
HAMqttDiscoveryHandler hamdhCoolix("homegear_ir", "ac001", "Coolix", "Coolix AC", SOFTWARE_VERSION, hamdhHG);
IRCoolixAC coolixAC(kIrLed);

void setCoolixACDefaultState()
{
  coolixAC.off();
  coolixAC.setTemp(26); // 16-30C
}
#endif

#if IR_HAIER_AC
#include <ir_Haier.h>
HAMqttDiscoveryHandler hamdhHaier("homegear_ir", "ac001", "Haier", "Haier AC", SOFTWARE_VERSION, hamdhHG);
IRHaierAC176 haierAC(kIrLed);

void setHaierACDefaultState()
{
  haierAC.off();
  haierAC.setTemp(26); // 16-30C
}
#endif

#endif

#if RF_MODULE

HardwareSerial rfSerial(1);

#if RF_PHILIPS_FAN_LIGHT
HAMqttDiscoveryHandler hamdhPFL("homegear_rf", "00001", "Philips", "Philips RF Remote Fan Light", SOFTWARE_VERSION, hamdhHG);
#endif

#include "RF_Code.h"
void sendRfSig(const unsigned char data[6])
{
  rfSerial.write(rf_module_uart_protocol[0]);
  for (int i = 0; i < 6; i++)
  {
    rfSerial.write(data[i]);
  }
  rfSerial.write(rf_module_uart_protocol[1]);
}
#endif

#if ZGB_MODULE

#include <WiFiClient.h>
#include <WiFiServer.h>
#define ZGB_BAUD_RATE 115200
#define TCP_LISTEN_PORT 1234
// serial and ethernet buffer size
#define BUFFER_SIZE 256
WiFiServer ser2NetServer(TCP_LISTEN_PORT);
WiFiClient wClient;

HardwareSerial zgbSerial(2);
#endif

//***** network communication *****

// MQTT
ESP32MQTTClient mqttClient;

#if (IR_MODULE && IR_HITACHI_AC3)
// Only loop() reads or changes command state, sends IR and publishes snapshots.
String hitachiAc3Feedback()
{
  StaticJsonDocument<256> feedback;
  feedback["achitachi_mode"] = hitachiAc3.getMode();
  feedback["achitachi_temperature"] = hitachiAc3.getTemperature();
  feedback["achitachi_fan_mode"] = hitachiAc3.getFanMode();
  if (hitachiAc3.isKnown())
    feedback["achitachi_power"] = hitachiAc3.getPower();
  else
    feedback["achitachi_power"] = nullptr;
  feedback["state_source"] = hitachiAc3.getStateSource();
  String payload;
  serializeJson(feedback, payload);
  return payload;
}

void publishHitachiAc3State()
{
  if (hitachiAc3.isKnown() && mqttClient.isConnected())
    mqttClient.publish(hamdhHitachi.getStateTopic(), hitachiAc3Feedback(), 0, true);
}

bool enqueueHitachiAc3Event(uint8_t event)
{
  if (!hitachiAc3Queue)
  {
    Serial.println("HITACHI ERROR NOT_INITIALIZED");
    return false;
  }
  if (xQueueSend(hitachiAc3Queue, &event, 0) != pdTRUE)
  {
    Serial.println("HITACHI ERROR COMMAND_QUEUE_FULL");
    return false;
  }
  return true;
}

bool commandHitachiAc3(bool power)
{
  return enqueueHitachiAc3Event(power ? kHitachiAc3OnEvent : kHitachiAc3OffEvent);
}

void processHitachiAc3Event()
{
  uint8_t event;
  if (!hitachiAc3Queue || xQueueReceive(hitachiAc3Queue, &event, 0) != pdTRUE) return;
  if (event == kHitachiAc3PublishEvent)
  {
    publishHitachiAc3State();
    return;
  }
  if (event != kHitachiAc3OnEvent && event != kHitachiAc3OffEvent) return;
  hitachiAc3.setPower(event == kHitachiAc3OnEvent);
  irsend.sendHitachiAc3(hitachiAc3.getRaw(), HitachiAc3Control::kStateLength, 0);
  publishHitachiAc3State();
#if SERIAL_DEBUG
  Serial.print("HITACHI ");
  Serial.println(hitachiAc3Feedback());
#endif
}

void printHitachiAc3State()
{
  Serial.println(hitachiAc3Feedback());
}

void handleHitachiAc3Command(const String &payload)
{
  bool power;
  if (!HitachiAc3Control::parseModeCommand(payload.c_str(), payload.length(), power))
  {
    Serial.println("HITACHI ERROR EXPECTED_ONLY_LITERAL_MODE_DRY_OR_OFF");
    return;
  }
  commandHitachiAc3(power);
}
#endif

// define your default values here, if there are different values in config.json, they are overwritten.
#define SERVER_ADDR_LEN 64
#define NW_PORT_LEN 6
#define USER_PWD_LEN 32
#define TOKEN_LEN 256
#define FLAG_LEN 6

// Active values stay unchanged until reboot; serial and portal edits are staged.
HomeGearSerialConfig::Values activeConfig;
HomeGearSerialConfig::Values pendingConfig;
char (&mqtt_server)[SERVER_ADDR_LEN] = activeConfig.mqtt_server;
char (&mqtt_port)[NW_PORT_LEN] = activeConfig.mqtt_port;
char (&mqtt_user)[USER_PWD_LEN] = activeConfig.mqtt_user;
char (&mqtt_pwd)[USER_PWD_LEN] = activeConfig.mqtt_pwd;
char (&homeassistant_server)[SERVER_ADDR_LEN] = activeConfig.homeassistant_server;
char (&homeassistant_port)[NW_PORT_LEN] = activeConfig.homeassistant_port;
char (&homeassistant_token)[TOKEN_LEN] = activeConfig.homeassistant_token;
bool shouldSaveConfig = false;
bool configDirty = false;
uint8_t configEditedFields = 0;
bool configFsReady = false;
bool mqttStarted = false;
String mqttUri;
const size_t kMaxConfigFileLength = 4096;
WiFiManager wifiManager;
WiFiManagerParameter custom_mqtt_server("mqttserver", "MQTT Server", mqtt_server, SERVER_ADDR_LEN);
WiFiManagerParameter custom_mqtt_port("mqttport", "MQTT Port", mqtt_port, NW_PORT_LEN);
WiFiManagerParameter custom_mqtt_user("mqttuser", "MQTT Username", mqtt_user, USER_PWD_LEN);
WiFiManagerParameter custom_mqtt_pwd("mqttpwd", "MQTT Password", mqtt_pwd, USER_PWD_LEN, "type='password'");
WiFiManagerParameter custom_homeassistant_server("homeassistantserver", "Home Assistant Server", homeassistant_server, SERVER_ADDR_LEN);
WiFiManagerParameter custom_homeassistant_port("homeassistantport", "Home Assistant Port", homeassistant_port, NW_PORT_LEN);
WiFiManagerParameter custom_homeassistant_token("homeassistanttoken", "Home Assistant Token", homeassistant_token, TOKEN_LEN, "type='password'");

void saveConfigCallback()
{
  shouldSaveConfig = true;
}

bool readConfigDocument(const char *path, DynamicJsonDocument &json)
{
  File file = SPIFFS.open(path, "r");
  if (!file) return false;
  if (!file.size() || file.size() > kMaxConfigFileLength)
  {
    file.close();
    return false;
  }
  json.clear();
  DeserializationError error = deserializeJson(json, file);
  bool complete = !error;
  while (complete && file.available())
  {
    int byte = file.read();
    if (byte != ' ' && byte != '\t' && byte != '\r' && byte != '\n') complete = false;
  }
  file.close();
  return complete && json.is<JsonObject>();
}

void loadConfigFromFS()
{
  configFsReady = SPIFFS.begin(false); // Never format existing Wi-Fi/config data.
  if (!configFsReady)
  {
    Serial.println("CONFIG ERROR FILESYSTEM_UNAVAILABLE");
    pendingConfig = activeConfig;
    return;
  }
  DynamicJsonDocument json(kMaxConfigFileLength);
  if (!readConfigDocument("/config.json", json))
  {
    if (!readConfigDocument("/config.bak", json))
    {
      Serial.println("CONFIG NO_VALID_SAVED_CONFIG");
      pendingConfig = activeConfig;
      return;
    }
    Serial.println("CONFIG LOADED_BACKUP");
  }
  for (int i = 0; i < static_cast<int>(HomeGearSerialConfig::Key::Invalid); ++i)
  {
    HomeGearSerialConfig::Key key = static_cast<HomeGearSerialConfig::Key>(i);
    const char *name = HomeGearSerialConfig::keyName(key);
    if (!json.containsKey(name)) continue;
    if (!json[name].is<const char *>())
    {
      Serial.printf("CONFIG IGNORED_INVALID_FIELD %s\n", name);
      continue;
    }
    JsonString value = json[name].as<JsonString>();
    // Empty legacy host fields mean unconfigured; all other fields are validated.
    if (!value.size() && (key == HomeGearSerialConfig::Key::MqttServer || key == HomeGearSerialConfig::Key::HomeassistantServer)) continue;
    if (!HomeGearSerialConfig::setValue(activeConfig, key, value.c_str(), value.size()))
      Serial.printf("CONFIG IGNORED_INVALID_FIELD %s\n", name);
  }
  pendingConfig = activeConfig;
  Serial.println("CONFIG LOADED");
}

bool saveConfigToFS()
{
  if (!configFsReady) configFsReady = SPIFFS.begin(false);
  if (!configFsReady)
  {
    Serial.println("CONFIG ERROR FILESYSTEM_UNAVAILABLE");
    return false;
  }
  DynamicJsonDocument json(kMaxConfigFileLength);
  bool hadCanonical = SPIFFS.exists("/config.json");
  bool canonicalValid = hadCanonical && readConfigDocument("/config.json", json);
  bool hadSavedDocument = canonicalValid;
  if (!canonicalValid)
  {
    if (SPIFFS.exists("/config.bak"))
    {
      if (!readConfigDocument("/config.bak", json))
      {
        Serial.println("CONFIG ERROR EXISTING_CONFIG_INVALID");
        return false;
      }
      hadSavedDocument = true;
    }
    else if (hadCanonical)
    {
      Serial.println("CONFIG ERROR EXISTING_CONFIG_INVALID");
      return false;
    }
    else json.clear();
  }
  // Only explicit edits replace saved fields. In particular, a legacy value
  // that was too long to load must survive an unrelated server-address edit.
  for (int i = 0; i < static_cast<int>(HomeGearSerialConfig::Key::Invalid); ++i)
  {
    if (!hadSavedDocument || (configEditedFields & (1U << i)))
    {
      HomeGearSerialConfig::Key key = static_cast<HomeGearSerialConfig::Key>(i);
      json[HomeGearSerialConfig::keyName(key)] = HomeGearSerialConfig::field(pendingConfig, key);
    }
  }
  size_t expected = measureJson(json);
  String serialized;
  size_t serializedLength = serializeJson(json, serialized);
  if (json.overflowed() || expected > kMaxConfigFileLength || serializedLength != expected || serialized.length() != expected)
  {
    Serial.println("CONFIG ERROR CONFIG_TOO_LARGE");
    return false;
  }
  File temporary = SPIFFS.open("/config.tmp", "w");
  if (!temporary)
  {
    Serial.println("CONFIG ERROR TEMP_OPEN_FAILED");
    return false;
  }
  size_t written = temporary.write(reinterpret_cast<const uint8_t *>(serialized.c_str()), expected);
  temporary.flush();
  temporary.close();
  File check = SPIFFS.open("/config.tmp", "r");
  bool complete = check && check.size() == expected && written == expected;
  size_t offset = 0;
  char buffer[64];
  while (complete && offset < expected)
  {
    size_t count = expected - offset;
    if (count > sizeof(buffer)) count = sizeof(buffer);
    size_t received = check.readBytes(buffer, count);
    complete = received == count && memcmp(buffer, serialized.c_str() + offset, count) == 0;
    offset += received;
  }
  if (check) check.close();
  if (!complete)
  {
    SPIFFS.remove("/config.tmp");
    Serial.println("CONFIG ERROR TEMP_WRITE_FAILED");
    return false;
  }
  // SPIFFS cannot rename over an existing destination. Keep the old document
  // as a recovery file until the new fully written document has been renamed.
  if (hadCanonical)
  {
    if (canonicalValid)
    {
      if ((SPIFFS.exists("/config.bak") && !SPIFFS.remove("/config.bak")) ||
          !SPIFFS.rename("/config.json", "/config.bak"))
      {
        SPIFFS.remove("/config.tmp");
        Serial.println("CONFIG ERROR BACKUP_RENAME_FAILED");
        return false;
      }
    }
    else if (!SPIFFS.remove("/config.json"))
    {
      SPIFFS.remove("/config.tmp");
      Serial.println("CONFIG ERROR INVALID_CONFIG_REMOVE_FAILED");
      return false;
    }
  }
  if (!SPIFFS.rename("/config.tmp", "/config.json"))
  {
    if (!SPIFFS.exists("/config.json") && SPIFFS.exists("/config.bak"))
      SPIFFS.rename("/config.bak", "/config.json");
    SPIFFS.remove("/config.tmp");
    Serial.println("CONFIG ERROR PUBLISH_RENAME_FAILED");
    return false;
  }
  configEditedFields = 0;
  configDirty = false;
  Serial.println("CONFIG OK SAVED");
  return true;
}

void syncPortalConfig()
{
  custom_mqtt_server.setValue(pendingConfig.mqtt_server, SERVER_ADDR_LEN);
  custom_mqtt_port.setValue(pendingConfig.mqtt_port, NW_PORT_LEN);
  custom_mqtt_user.setValue(pendingConfig.mqtt_user, USER_PWD_LEN);
  custom_mqtt_pwd.setValue(pendingConfig.mqtt_pwd, USER_PWD_LEN);
  custom_homeassistant_server.setValue(pendingConfig.homeassistant_server, SERVER_ADDR_LEN);
  custom_homeassistant_port.setValue(pendingConfig.homeassistant_port, NW_PORT_LEN);
  custom_homeassistant_token.setValue(pendingConfig.homeassistant_token, TOKEN_LEN);
}

bool importPortalConfig()
{
  const char *values[] = {custom_mqtt_server.getValue(), custom_mqtt_port.getValue(),
      custom_mqtt_user.getValue(), custom_mqtt_pwd.getValue(),
      custom_homeassistant_server.getValue(), custom_homeassistant_port.getValue(),
      custom_homeassistant_token.getValue()};
  HomeGearSerialConfig::Values candidate = pendingConfig;
  uint8_t changedFields = 0;
  for (int i = 0; i < static_cast<int>(HomeGearSerialConfig::Key::Invalid); ++i)
  {
    HomeGearSerialConfig::Key key = static_cast<HomeGearSerialConfig::Key>(i);
    const char *value = values[i];
    if (!value) return false;
    // An unchanged empty UI field must not erase an un-loaded legacy secret.
    if (strcmp(value, HomeGearSerialConfig::field(pendingConfig, key)) == 0) continue;
    size_t length = strlen(value);
    if (!length && (key == HomeGearSerialConfig::Key::MqttServer || key == HomeGearSerialConfig::Key::HomeassistantServer)) continue;
    if (!HomeGearSerialConfig::setValue(candidate, key, value, length))
    {
      Serial.printf("CONFIG ERROR INVALID_PORTAL_FIELD %s\n", HomeGearSerialConfig::keyName(key));
      return false;
    }
    changedFields |= static_cast<uint8_t>(1U << i);
  }
  pendingConfig = candidate;
  configEditedFields |= changedFields;
  configDirty = configEditedFields != 0;
  return true;
}

void showSerialConfig()
{
  Serial.println("CONFIG BEGIN");
  for (int i = 0; i < static_cast<int>(HomeGearSerialConfig::Key::Invalid); ++i)
  {
    HomeGearSerialConfig::Key key = static_cast<HomeGearSerialConfig::Key>(i);
    const char *value = HomeGearSerialConfig::field(pendingConfig, key);
    Serial.printf("CONFIG %s=%s\n", HomeGearSerialConfig::keyName(key),
        HomeGearSerialConfig::isSecret(key) ? (value[0] ? "set" : "empty") : value);
  }
  Serial.printf("CONFIG unsaved=%s\n", configDirty ? "yes" : "no");
  Serial.printf("CONFIG active_mqtt_server=%s\n", mqtt_server);
  Serial.printf("CONFIG active_mqtt_port=%s\n", mqtt_port);
  Serial.printf("CONFIG active_homeassistant_server=%s\n", homeassistant_server);
  Serial.printf("CONFIG active_homeassistant_port=%s\n", homeassistant_port);
  Serial.printf("CONFIG wifi_connected=%s\n", WiFi.status() == WL_CONNECTED ? "true" : "false");
  Serial.printf("CONFIG local_ip=%s\n", WiFi.localIP().toString().c_str());
  Serial.printf("CONFIG mqtt_connected=%s\n", mqttClient.isConnected() ? "true" : "false");
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
      if (!HomeGearSerialConfig::setValue(pendingConfig, command.key, command.value, command.valueLength))
      {
        Serial.println("CONFIG ERROR INVALID_VALUE");
        return;
      }
      configEditedFields |= static_cast<uint8_t>(1U << static_cast<unsigned>(command.key));
      configDirty = true;
      syncPortalConfig();
      Serial.printf("CONFIG OK STAGED %s\n", HomeGearSerialConfig::keyName(command.key));
    }
    else if (saveConfigToFS() && command.operation == HomeGearSerialConfig::Operation::Apply)
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
  if (cmd == "hitachi_on") commandHitachiAc3(true);
  else if (cmd == "hitachi_off") commandHitachiAc3(false);
  else if (cmd == "hitachi_status") printHitachiAc3State();
  else
#endif
  if (cmd == "lsmod") Serial.println(moduleList());
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

void wmStart()
{
  syncPortalConfig();
  wifiManager.setDebugOutput(false);
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setConnectTimeout(1);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setSaveParamsCallback(saveConfigCallback);
#if ZGB_MODULE
  wifiManager.setHostname("HomeGear-ZigBee");
#endif
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pwd);
  wifiManager.addParameter(&custom_homeassistant_server);
  wifiManager.addParameter(&custom_homeassistant_port);
  wifiManager.addParameter(&custom_homeassistant_token);
  wifiManager.autoConnect(nodename, "homegearconfig");
  Serial.println("CONFIG SERIAL_READY");
}

void mqttStart()
{
  mqttClient.setMaxPacketSize(2048);
#if SERIAL_DEBUG
  mqttClient.enableDebuggingMessages();
#endif
  // esp32mqttclient doesn't have OTA function
  // mqttClient.enableHTTPWebUpdater("admin", "homegearota", "/ota");
  mqttClient.enableLastWillMessage(avt, "offline", true);
  // Set the WiFi and MQTT information that we loaded from global param
  mqttClient.setMqttClientName(nodename);
  mqttUri = "mqtt://" + String(mqtt_server) + ":" + String(mqtt_port);
  mqttClient.setURI(mqttUri.c_str(), mqtt_user, mqtt_pwd);
  // Legacy ESP32MQTTClient 0.1.0 always returns false even after starting.
  // Guard the start attempt once; its internal reconnect handles network loss.
  mqttStarted = true;
  mqttClient.loopStart();
}

void modulesStart()
{
#if (THI_MODULE || IAQ_MODULE)
  Wire.begin();
#endif

#if THI_MODULE
  bh1750_1.begin(BH1750::ONE_TIME_HIGH_RES_MODE, 0x23);
  bh1750_2.begin(BH1750::ONE_TIME_HIGH_RES_MODE, 0x5C);
  shtc3.begin();
#endif

#if IAQ_MODULE
  bme680.begin(BME68X_I2C_ADDR_LOW, Wire);
  bsec_virtual_sensor_t bme680SenseList[6] = {
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_STATIC_IAQ,
      BSEC_OUTPUT_CO2_EQUIVALENT,
      BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  bme680.updateSubscription(bme680SenseList, 6, BSEC_SAMPLE_RATE_LP);
#endif

#if IR_MODULE
  irsend.begin();
#if IR_HITACHI_AC3
  hitachiAc3Queue = xQueueCreate(8, sizeof(uint8_t));
  if (!hitachiAc3Queue) Serial.println("HITACHI ERROR QUEUE_ALLOCATION_FAILED");
#endif
#if IR_GREE_AC
  greeAC.begin();

#endif
#if IR_MIDEA_AC
  mideaAC.begin();

#endif
#if IR_COOLIX_AC
  coolixAC.begin();

#endif
#if IR_HAIER_AC
  haierAC.begin();

#endif
#endif

#if RF_MODULE
  rfSerial.begin(115200, SERIAL_8N1, 13, 12);
#endif

#if ZGB_MODULE
  zgbSerial.begin(115200);
  ser2NetServer.begin();
#endif
}

void setup()
{
  // device name
  Serial.begin(115200);
  Serial.println();

  String nodenameStr = "HomeGear_" + chipId;
  nodenameStr.toUpperCase();
  strcpy(nodename, nodenameStr.c_str());

  loadConfigFromFS();
  // Serial control and sensors must work while Wi-Fi provisioning is pending.
  modulesStart();
  wmStart();
  if (WiFi.status() == WL_CONNECTED && mqtt_server[0]) mqttStart();

  startLoopMs = millis();
  lastPubMs = millis();
}

void onConnectionEstablishedCallback(esp_mqtt_client_handle_t client)
{
  // Publish online message to availabilityTopic
  mqttClient.publish(avt, "online", 0, true);

  // Subscribe to cmdTopic
  mqttClient.subscribe(hamdhHG.getCmdTopic(), [](const String &payload)
                       {
                         if (payload.equals("restore"))
                         {
                           SPIFFS.format();
                           WiFi.disconnect(true);
                           ESP.restart();
                           delay(1000);
                         }
                         else if (payload.equals("reboot"))
                         {
                           ESP.restart();
                           delay(1000);
                         }
                         else if (payload.equals("pubdata"))
                         {
#if THI_MODULE
                           thimEvent = true;
#endif

#if IAQ_MODULE
                           iaqmEvent = true;
#endif
                         }
#if RF_MODULE
                         else if (payload.equals("setrfmodbaud115200"))
                         {
                           sendRfSig(rf_module_set_baudrate_to_115200);
                         }
#endif
                       });

  // publish HA MQTT Discovery messages
#if (THI_MODULE || IAQ_MODULE)
  HAMqttDiscoverySensor hamdhSensor(hamdhHG);
#endif

#if THI_MODULE
  hamdhSensor.setDeviceClass("temperature");
  hamdhSensor.setDeviceNameMin("thim_temperature");
  hamdhSensor.setStateClass("measurement");
  hamdhSensor.setUnitOfMeasurement("°C");
  hamdhSensor.setValueTemplate("{{ value_json.thim_temperature }}");
  hamdhSensor.construct();
  mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);

  hamdhSensor.setDeviceClass("humidity");
  hamdhSensor.setDeviceNameMin("thim_humidity");
  hamdhSensor.setUnitOfMeasurement("%");
  hamdhSensor.setValueTemplate("{{ value_json.thim_humidity }}");
  hamdhSensor.construct();
  mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);

  hamdhSensor.setDeviceClass("illuminance");
  hamdhSensor.setDeviceNameMin("thim_illuminance");
  hamdhSensor.setUnitOfMeasurement("lx");
  hamdhSensor.setValueTemplate("{{ value_json.thim_illuminance }}");
  hamdhSensor.construct();
  mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);
#endif

#if IAQ_MODULE
  hamdhSensor.setDeviceClass("temperature");
  hamdhSensor.setDeviceNameMin("iaqm_temperature");
  hamdhSensor.setStateClass("measurement");
  hamdhSensor.setUnitOfMeasurement("°C");
  hamdhSensor.setValueTemplate("{{ value_json.iaqm_temperature }}");
  hamdhSensor.construct();
  mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);

  hamdhSensor.setDeviceClass("humidity");
  hamdhSensor.setDeviceNameMin("iaqm_humidity");
  hamdhSensor.setUnitOfMeasurement("%");
  hamdhSensor.setValueTemplate("{{ value_json.iaqm_humidity }}");
  hamdhSensor.construct();
  mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);

  hamdhSensor.setDeviceClass("pressure");
  hamdhSensor.setDeviceNameMin("iaqm_pressure");
  hamdhSensor.setUnitOfMeasurement("Pa");
  hamdhSensor.setValueTemplate("{{ value_json.iaqm_pressure }}");
  hamdhSensor.construct();
  mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);

  hamdhSensor.setDeviceClass("aqi");
  hamdhSensor.setDeviceNameMin("iaqm_iaq");
  hamdhSensor.setUnitOfMeasurement("");
  hamdhSensor.setValueTemplate("{{ value_json.iaqm_iaq }}");
  hamdhSensor.construct();
  mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);

  hamdhSensor.setDeviceClass("carbon_dioxide");
  hamdhSensor.setDeviceNameMin("iaqm_co2Equivalent");
  hamdhSensor.setUnitOfMeasurement("ppm");
  hamdhSensor.setValueTemplate("{{ value_json.iaqm_co2Equivalent }}");
  hamdhSensor.construct();
  mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);

  hamdhSensor.setDeviceClass("volatile_organic_compounds");
  hamdhSensor.setDeviceNameMin("iaqm_breathVocEquivalent");
  hamdhSensor.setUnitOfMeasurement("µg/m³");
  hamdhSensor.setValueTemplate("{{ value_json.iaqm_breathVocEquivalent }}");
  hamdhSensor.construct();
  mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);
#endif

#if IR_MODULE
#if IR_HITACHI_AC3
  HAMqttDiscoveryClimate hamdhHitachiAC(hamdhHitachi);
  hamdhHitachiAC.setDeviceNameMin("hitachi_ac3_25c_dry");
  hamdhHitachiAC.setModeCommandTemplate("{\"achitachi_mode\": \"{{ value }}\"}");
  hamdhHitachiAC.setModeStateTemplate("{{ value_json.achitachi_mode }}");
  String hitachiModes[10] = {"off", "dry", "", "", "", "", "", "", "", ""};
  hamdhHitachiAC.setModes(hitachiModes);
  // Fixed 25 C and low fan are attributes, not adjustable HA capabilities.
  hamdhHitachiAC.construct();
  mqttClient.publish(hamdhHitachiAC.getMqttDiscoveryConfigTopic(), hamdhHitachiAC.getMqttDiscoveryMesg(), 0, true);
  mqttClient.subscribe(hamdhHitachi.getCtrlTopic(), [](const String &payload) { handleHitachiAc3Command(payload); });
  // Schedule a loop-owned snapshot; reconnect never resets state or sends IR.
  enqueueHitachiAc3Event(kHitachiAc3PublishEvent);
#endif
#if IR_SINGFUN_FAN
  HAMqttDiscoveryFan hamdhSFF(hamdhSF);
  hamdhSFF.setDeviceNameMin("singfun_fan");
  hamdhSFF.setCommandTemplate("{\"sff_state\": \"{{ value }}\"}");
  hamdhSFF.setOscillationCommandTemplate("{\"sff_oscillation\": \"{{ value }}\"}");
  hamdhSFF.setStateValueTemplate("{{value_json.sff_state}}");
  hamdhSFF.setOscillationValueTemplate("{{value_json.sff_oscillation}}");
  hamdhSFF.construct();
  mqttClient.publish(hamdhSFF.getMqttDiscoveryConfigTopic(), hamdhSFF.getMqttDiscoveryMesg(), true);

  mqttClient.subscribe(hamdhSFF.getCtrlTopic(), [](const String &payload)
                       {
    StaticJsonDocument<100> doc;
    if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
    {
      StaticJsonDocument<100> feedbackDoc;
      String feedbackMesg;
      if (doc.containsKey("sff_state")) {
        String sffOnOff = doc["sff_state"];
        irsend.sendRaw(singfun_fan_power, 75, 38);
        feedbackDoc["sff_state"] = sffOnOff;
        if (sffOnOff.equals("OFF")) {
          feedbackDoc["sff_oscillation"] = "oscillate_off";
        }
      }
      if (doc.containsKey("sff_oscillation")) {
        String sffOsciOnOff = doc["sff_oscillation"];
        irsend.sendRaw(singfun_fan_oscillating, 75, 38);
        feedbackDoc["sff_oscillation"] = sffOsciOnOff;
      }
      serializeJson(feedbackDoc, feedbackMesg);
      mqttClient.publish(hamdhSF.getStateTopic(), feedbackMesg, true);
    } });
#endif
#if IR_GREE_AC
  HAMqttDiscoveryClimate hamdhGreeAC(hamdhGree);
  hamdhGreeAC.setDeviceNameMin("gree_ac");
  hamdhGreeAC.setModeCommandTemplate("{\"acgree_mode\": \"{{ value }}\"}");
  hamdhGreeAC.setFanModeCommandTemplate("{\"acgree_fan_mode\": \"{{ value }}\"}");
  hamdhGreeAC.setSwingModeCommandTemplate("{\"acgree_swing_mode\": \"{{ value }}\"}");
  hamdhGreeAC.setTemperatureCommandTemplate("{\"acgree_temperature\": \"{{ value }}\"}");
  hamdhGreeAC.setfanModeStateTemplate("{{ value_json.acgree_fan_mode }}");
  hamdhGreeAC.setModeStateTemplate("{{ value_json.acgree_mode }}");
  hamdhGreeAC.setSwingModeStateTemplate("{{ value_json.acgree_swing_mode }}");
  hamdhGreeAC.setTemperatureStateTemplate("{{ value_json.acgree_temperature }}");
  // String modes[10] = {"auto", "cool", "dry", "fan_only", "heat", "off", "", "", "", ""};
  String modes[10] = {"auto", "cool", "dry", "fan_only", "off", "", "", "", "", ""};
  String swingModes[10] = {"off", "vertical", "horizontal", "both", "", "", "", "", "", ""};
  String fanModes[10] = {"low", "medium", "high", "", "", "", "", "", "", ""};
  hamdhGreeAC.setModes(modes);
  hamdhGreeAC.setSwingModes(swingModes);
  hamdhGreeAC.setFanModes(fanModes);

  hamdhGreeAC.construct();
  mqttClient.publish(hamdhGreeAC.getMqttDiscoveryConfigTopic(), hamdhGreeAC.getMqttDiscoveryMesg(), true);

  setGreeACDefaultState();
  StaticJsonDocument<100> greeACdefaultStateDoc;
  String greeACdefaultState;
  greeACdefaultStateDoc["acgree_mode"] = "off";
  greeACdefaultStateDoc["acgree_fan_mode"] = "medium";
  greeACdefaultStateDoc["acgree_swing_mode"] = "off";
  greeACdefaultStateDoc["acgree_temperature"] = "26";
  serializeJson(greeACdefaultStateDoc, greeACdefaultState);
  mqttClient.publish(hamdhGreeAC.getStateTopic(), greeACdefaultState, true);

  mqttClient.subscribe(hamdhGreeAC.getCtrlTopic(), [](const String &payload)
                       {
    StaticJsonDocument<100> doc;
    if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
    {
      StaticJsonDocument<100> feedbackDoc;
      String feedbackMesg;
	  bool isOffCmd = false;
      if (doc.containsKey("acgree_mode"))
      {
        String mode = doc["acgree_mode"];
		greeAC.on();
        if (mode.equals("auto")) {
          greeAC.setMode(kGreeAuto);
        }
        else if (mode.equals("cool")) {
          greeAC.setMode(kGreeCool);
        }
        else if (mode.equals("dry")) {
          greeAC.setMode(kGreeDry);
        }
        else if (mode.equals("fan_only")) {
          greeAC.setMode(kGreeFan);
        }
        else if (mode.equals("heat")) {
          greeAC.setMode(kGreeHeat);
        }
		else if (mode.equals("off")) {
		  greeAC.off();
		  isOffCmd = true;
		}
        feedbackDoc["acgree_mode"] = mode;
      }
      if (doc.containsKey("acgree_fan_mode"))
      {
        String fanMode = doc["acgree_fan_mode"];
        if (fanMode.equals("low")) {
          greeAC.setFan(1);
        }
        else if (fanMode.equals("medium")) {
          greeAC.setFan(2);
        }
        else if (fanMode.equals("high")) {
          greeAC.setFan(3);
        }
        feedbackDoc["acgree_fan_mode"] = fanMode;
      }
      if (doc.containsKey("acgree_swing_mode"))
      {
        String swingMode = doc["acgree_swing_mode"];
        if (swingMode.equals("off")) {
          greeAC.setSwingVertical(false, kGreeSwingAuto);
          greeAC.setSwingHorizontal(false);
        }
        else if (swingMode.equals("vertical")) {
        greeAC.setSwingVertical(true, kGreeSwingAuto);
        }
        else if (swingMode.equals("horizontal")) {
        greeAC.setSwingHorizontal(true);
        }
        else if (swingMode.equals("both")) {
        greeAC.setSwingVertical(true, kGreeSwingAuto);
          greeAC.setSwingHorizontal(true);
        }
        feedbackDoc["acgree_swing_mode"] = swingMode;
      }
      if (doc.containsKey("acgree_temperature"))
      {
        String tempStr = doc["acgree_temperature"];
		float temp = tempStr.toFloat();
        int target = round(temp);
        greeAC.setTemp(target);
        feedbackDoc["acgree_temperature"] = tempStr;
      }
	  if(isOffCmd || ((!isOffCmd) && greeAC.getPower())){
		  greeAC.send();
	  }
      
      serializeJson(feedbackDoc, feedbackMesg);
      mqttClient.publish(hamdhGree.getStateTopic(), feedbackMesg, true);
    } });
#endif

#if IR_MIDEA_AC
  HAMqttDiscoveryClimate hamdhMideaAC(hamdhMidea);
  hamdhMideaAC.setDeviceNameMin("midea_ac");
  hamdhMideaAC.setModeCommandTemplate("{\"acmidea_mode\": \"{{ value }}\"}");
  hamdhMideaAC.setFanModeCommandTemplate("{\"acmidea_fan_mode\": \"{{ value }}\"}");
  hamdhMideaAC.setSwingModeCommandTemplate("{\"acmidea_swing_mode\": \"{{ value }}\"}");
  hamdhMideaAC.setTemperatureCommandTemplate("{\"acmidea_temperature\": \"{{ value }}\"}");
  hamdhMideaAC.setfanModeStateTemplate("{{ value_json.acmidea_fan_mode }}");
  hamdhMideaAC.setModeStateTemplate("{{ value_json.acmidea_mode }}");
  hamdhMideaAC.setSwingModeStateTemplate("{{ value_json.acmidea_swing_mode }}");
  hamdhMideaAC.setTemperatureStateTemplate("{{ value_json.acmidea_temperature }}");
  String modes[10] = {"auto", "cool", "dry", "fan_only", "heat", "off", "", "", "", ""};
  // String modes[10] = {"auto", "cool", "dry", "fan_only", "off", "", "", "", "", ""};
  String swingModes[10] = {"off", "vertical", "", "", "", "", "", "", "", ""};
  String fanModes[10] = {"low", "medium", "high", "auto", "", "", "", "", "", ""};
  hamdhMideaAC.setModes(modes);
  hamdhMideaAC.setSwingModes(swingModes);
  hamdhMideaAC.setFanModes(fanModes);

  hamdhMideaAC.construct();
  mqttClient.publish(hamdhMideaAC.getMqttDiscoveryConfigTopic(), hamdhMideaAC.getMqttDiscoveryMesg(), true);

  setMideaACDefaultState();
  StaticJsonDocument<100> mideaACdefaultStateDoc;
  String mideaACdefaultState;
  mideaACdefaultStateDoc["acmidea_mode"] = "off";
  mideaACdefaultStateDoc["acmidea_fan_mode"] = "medium";
  mideaACdefaultStateDoc["acmidea_swing_mode"] = "off";
  mideaACdefaultStateDoc["acmidea_temperature"] = "26";
  serializeJson(mideaACdefaultStateDoc, mideaACdefaultState);
  mqttClient.publish(hamdhMideaAC.getStateTopic(), mideaACdefaultState, true);

  mqttClient.subscribe(hamdhMideaAC.getCtrlTopic(), [](const String &payload)
                       {
    StaticJsonDocument<100> doc;
    if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
    {
      StaticJsonDocument<100> feedbackDoc;
      String feedbackMesg;
	  bool isOffCmd = false;
      if (doc.containsKey("acmidea_mode"))
      {
        String mode = doc["acmidea_mode"];
		mideaAC.on();
        if (mode.equals("auto")) {
          mideaAC.setMode(kMideaACAuto);
        }
        else if (mode.equals("cool")) {
          mideaAC.setMode(kMideaACCool);
        }
        else if (mode.equals("dry")) {
          mideaAC.setMode(kMideaACDry);
        }
        else if (mode.equals("fan_only")) {
          mideaAC.setMode(kMideaACFan);
        }
        else if (mode.equals("heat")) {
          mideaAC.setMode(kMideaACHeat);
        }
		else if (mode.equals("off")) {
		  mideaAC.off();
		  isOffCmd = true;
		}
        feedbackDoc["acmidea_mode"] = mode;
      }
      if (doc.containsKey("acmidea_fan_mode"))
      {
        String fanMode = doc["acmidea_fan_mode"];
        if (fanMode.equals("low")) {
          mideaAC.setFan(1);
        }
        else if (fanMode.equals("medium")) {
          mideaAC.setFan(2);
        }
        else if (fanMode.equals("high")) {
          mideaAC.setFan(3);
        }
        else if (fanMode.equals("auto")) {
          mideaAC.setFan(0);
        }
        feedbackDoc["acmidea_fan_mode"] = fanMode;
      }
      if (doc.containsKey("acmidea_swing_mode"))
      {
        String swingMode = doc["acmidea_swing_mode"];
        if (swingMode.equals("off")) {
          mideaAC.setSwingVToggle(false);
        }
        else if (swingMode.equals("vertical")) {
        mideaAC.setSwingVToggle(true);
        }
        feedbackDoc["acmidea_swing_mode"] = swingMode;
      }
      if (doc.containsKey("acmidea_temperature"))
      {
        String tempStr = doc["acmidea_temperature"];
		float temp = tempStr.toFloat();
        int target = round(temp);
        mideaAC.setTemp(target);
        feedbackDoc["acmidea_temperature"] = tempStr;
      }
	  if(isOffCmd || ((!isOffCmd) && mideaAC.getPower())){
		  mideaAC.send();
	  }
      
      serializeJson(feedbackDoc, feedbackMesg);
      mqttClient.publish(hamdhMidea.getStateTopic(), feedbackMesg, true);
    } });
#endif

#if IR_COOLIX_AC
  HAMqttDiscoveryClimate hamdhCoolixAC(hamdhCoolix);
  hamdhCoolixAC.setDeviceNameMin("coolix_ac");
  hamdhCoolixAC.setModeCommandTemplate("{\"accoolix_mode\": \"{{ value }}\"}");
  hamdhCoolixAC.setSwingModeCommandTemplate("{\"accoolix_swing_mode\": \"{{ value }}\"}");
  hamdhCoolixAC.setTemperatureCommandTemplate("{\"accoolix_temperature\": \"{{ value }}\"}");
  hamdhCoolixAC.setModeStateTemplate("{{ value_json.accoolix_mode }}");
  hamdhCoolixAC.setSwingModeStateTemplate("{{ value_json.accoolix_swing_mode }}");
  hamdhCoolixAC.setTemperatureStateTemplate("{{ value_json.accoolix_temperature }}");
  String modes[10] = {"cool", "heat", "off", "", "", "", "", "", "", ""};
  String swingModes[10] = {"off", "vertical", "", "", "", "", "", "", "", ""};
  hamdhCoolixAC.setModes(modes);
  hamdhCoolixAC.setSwingModes(swingModes);

  hamdhCoolixAC.construct();
  mqttClient.publish(hamdhCoolixAC.getMqttDiscoveryConfigTopic(), hamdhCoolixAC.getMqttDiscoveryMesg(), true);

  setCoolixACDefaultState();
  StaticJsonDocument<100> coolixACdefaultStateDoc;
  String coolixACdefaultState;
  coolixACdefaultStateDoc["accoolix_mode"] = "off";
  coolixACdefaultStateDoc["accoolix_swing_mode"] = "off";
  coolixACdefaultStateDoc["accoolix_temperature"] = "26";
  serializeJson(coolixACdefaultStateDoc, coolixACdefaultState);
  mqttClient.publish(hamdhCoolixAC.getStateTopic(), coolixACdefaultState, true);

  mqttClient.subscribe(hamdhCoolixAC.getCtrlTopic(), [](const String &payload)
                       {
    StaticJsonDocument<100> doc;
    if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
    {
      StaticJsonDocument<100> feedbackDoc;
      String feedbackMesg;
	  bool isOffCmd = false;
      if (doc.containsKey("accoolix_mode"))
      {
        String mode = doc["accoolix_mode"];
		coolixAC.on();
        if (mode.equals("cool")) {
          coolixAC.setMode(kCoolixCool);
        }
        else if (mode.equals("heat")) {
          coolixAC.setMode(kCoolixHeat);
        }
		else if (mode.equals("off")) {
		  coolixAC.off();
		  isOffCmd = true;
		}
        feedbackDoc["accoolix_mode"] = mode;
      }
      if (doc.containsKey("accoolix_swing_mode"))
      {
        String swingMode = doc["accoolix_swing_mode"];
        coolixAC.setSwing();
        feedbackDoc["accoolix_swing_mode"] = swingMode;
      }
      if (doc.containsKey("accoolix_temperature"))
      {
        String tempStr = doc["accoolix_temperature"];
		float temp = tempStr.toFloat();
        int target = round(temp);
        coolixAC.setTemp(target);
        feedbackDoc["accoolix_temperature"] = tempStr;
      }
	  if(isOffCmd || ((!isOffCmd) && coolixAC.getPower())){
	    coolixAC.send();
	  }
      
      serializeJson(feedbackDoc, feedbackMesg);
      mqttClient.publish(hamdhCoolix.getStateTopic(), feedbackMesg, true);
    } });
#endif

#if IR_HAIER_AC
  HAMqttDiscoveryClimate hamdhHaierAC(hamdhHaier);
  hamdhHaierAC.setDeviceNameMin("haier_ac");
  hamdhHaierAC.setModeCommandTemplate("{\"achaier_mode\": \"{{ value }}\"}");
  hamdhHaierAC.setSwingModeCommandTemplate("{\"achaier_swing_mode\": \"{{ value }}\"}");
  hamdhHaierAC.setTemperatureCommandTemplate("{\"achaier_temperature\": \"{{ value }}\"}");
  hamdhHaierAC.setModeStateTemplate("{{ value_json.achaier_mode }}");
  hamdhHaierAC.setSwingModeStateTemplate("{{ value_json.achaier_swing_mode }}");
  hamdhHaierAC.setTemperatureStateTemplate("{{ value_json.achaier_temperature }}");
  String modes[10] = {"cool", "heat", "off", "", "", "", "", "", "", ""};
  String swingModes[10] = {"off", "vertical", "", "", "", "", "", "", "", ""};
  hamdhHaierAC.setModes(modes);
  hamdhHaierAC.setSwingModes(swingModes);

  hamdhHaierAC.construct();
  mqttClient.publish(hamdhHaierAC.getMqttDiscoveryConfigTopic(), hamdhHaierAC.getMqttDiscoveryMesg(), true);

  setHaierACDefaultState();
  StaticJsonDocument<100> haierACdefaultStateDoc;
  String haierACdefaultState;
  haierACdefaultStateDoc["achaier_mode"] = "off";
  haierACdefaultStateDoc["achaier_swing_mode"] = "off";
  haierACdefaultStateDoc["achaier_temperature"] = "26";
  serializeJson(haierACdefaultStateDoc, haierACdefaultState);
  mqttClient.publish(hamdhHaierAC.getStateTopic(), haierACdefaultState, true);

  mqttClient.subscribe(hamdhHaierAC.getCtrlTopic(), [](const String &payload)
                       {
    StaticJsonDocument<100> doc;
    if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
    {
      StaticJsonDocument<100> feedbackDoc;
      String feedbackMesg;
	  bool isOffCmd = false;
      if (doc.containsKey("achaier_mode"))
      {
        String mode = doc["achaier_mode"];
		haierAC.on();
        if (mode.equals("cool")) {
          haierAC.setMode(kHaierAcCool);
        }
        else if (mode.equals("heat")) {
          haierAC.setMode(kHaierAcHeat);
        }
		else if (mode.equals("off")) {
		  haierAC.off();
		  isOffCmd = true;
		}
        feedbackDoc["achaier_mode"] = mode;
      }
      if (doc.containsKey("achaier_swing_mode"))
      {
        String swingMode = doc["achaier_swing_mode"];
        if(swingMode.equals("off")){
          haierAC.setSwingV(kHaierAcYrw02SwingVOff);
        }else if(swingMode.equals("vertical")){
          haierAC.setSwingV(kHaierAcYrw02SwingVAuto);
        }
        
        feedbackDoc["achaier_swing_mode"] = swingMode;
      }
      if (doc.containsKey("achaier_temperature"))
      {
        String tempStr = doc["achaier_temperature"];
		float temp = tempStr.toFloat();
        int target = round(temp);
        haierAC.setTemp(target);
        feedbackDoc["achaier_temperature"] = tempStr;
      }
	  if(isOffCmd || ((!isOffCmd) && haierAC.getPower())){
	    haierAC.send();
	  }
      
      serializeJson(feedbackDoc, feedbackMesg);
      mqttClient.publish(hamdhHaier.getStateTopic(), feedbackMesg, true);
    } });
#endif
#endif

#if RF_MODULE
#if RF_PHILIPS_FAN_LIGHT
  HAMqttDiscoveryFan hamdhPFLF(hamdhPFL);
  HAMqttDiscoveryLight hamdhPFLL(hamdhPFL);

  hamdhPFLF.setDeviceNameMin("philips_fanlight_fan");
  hamdhPFLF.setCommandTemplate("{\"pflf_state\": \"{{ value }}\"}");
  hamdhPFLF.setPercentageCommandTemplate("{\"pflf_percentage\": {{ value }}}");
  hamdhPFLF.setSpeedRangeMin(1);
  hamdhPFLF.setSpeedRangeMax(3);
  hamdhPFLF.setStateValueTemplate("{{value_json.pflf_state}}");
  hamdhPFLF.setPercentageValueTemplate("{{value_json.pflf_percentage}}");
  hamdhPFLF.construct();
  mqttClient.publish(hamdhPFLF.getMqttDiscoveryConfigTopic(), hamdhPFLF.getMqttDiscoveryMesg(), true);

  hamdhPFLL.setDeviceNameMin("philips_fanlight_light");
  hamdhPFLL.setPayloadOn("{\"pfll_state\":\"ON\"}");
  hamdhPFLL.setPayloadOff("{\"pfll_state\":\"OFF\"}");
  hamdhPFLL.setStateTemplate("{{ value_json.pfll_state }}");
  hamdhPFLL.construct();
  mqttClient.publish(hamdhPFLL.getMqttDiscoveryConfigTopic(), hamdhPFLL.getMqttDiscoveryMesg(), true);

  mqttClient.subscribe(hamdhPFL.getCtrlTopic(), [](const String &payload)
                       {
    StaticJsonDocument<100> doc;
    if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
    {
      StaticJsonDocument<100> feedbackDoc;
      String feedbackMesg;
      if (doc.containsKey("pflf_state"))
      {
        String pflf_state = doc["pflf_state"];
        feedbackDoc["pflf_state"] = pflf_state;
        if (pflf_state.equals("ON"))
        {
          sendRfSig(philips_fanlight_fan_low);
          feedbackDoc["pflf_percentage"] = 1;
        }
        else if (pflf_state.equals("OFF"))
        {
          sendRfSig(philips_fanlight_fan_off);
          feedbackDoc["pflf_percentage"] = 0;
        }
      }
      if (doc.containsKey("pflf_percentage"))
      {
        int val = doc["pflf_percentage"];
        feedbackDoc["pflf_percentage"] = val;
        if (val == 0) {
          sendRfSig(philips_fanlight_fan_off);
          feedbackDoc["pflf_state"] = "OFF";
        } else if (val == 1) {
          sendRfSig(philips_fanlight_fan_low);
          feedbackDoc["pflf_state"] = "ON";
        } else if (val == 2) {
          sendRfSig(philips_fanlight_fan_mid);
          feedbackDoc["pflf_state"] = "ON";
        } else if (val == 3) {
          sendRfSig(philips_fanlight_fan_high);
          feedbackDoc["pflf_state"] = "ON";
        }
      }
      if (doc.containsKey("pfll_state"))
      {
        String pfll_state = doc["pfll_state"];
        feedbackDoc["pfll_state"] = pfll_state;
        if (pfll_state.equals("ON")) {
          sendRfSig(philips_fanlight_light_on);
        }
        else if (pfll_state.equals("OFF")) {
          sendRfSig(philips_fanlight_light_off);
        }
      }
      serializeJson(feedbackDoc, feedbackMesg);
      mqttClient.publish(hamdhPFL.getStateTopic(), feedbackMesg, true);
    } });

#endif
#endif
}

void pubSensorsData()
{

  DynamicJsonDocument json(1024);
  String pubPayload;
#if THI_MODULE
  if (thimHasNewData)
  {
    dataShift(thim_humidity);
    dataShift(thim_temperature);
    dataShift(thim_illuminance);

    json["thim_temperature"] = String(thim_temperature[0]);
    json["thim_humidity"] = String(thim_humidity[0]);
    json["thim_illuminance"] = String(thim_illuminance[0]);

    thimHasNewData = false;
  }
#endif

#if IAQ_MODULE
  if (bme680HasNewData)
  {
    dataShift(iaqm_temperature);
    dataShift(iaqm_humidity);
    dataShift(iaqm_pressure);
    dataShift(iaqm_iaq);
    dataShift(iaqm_co2Equivalent);
    dataShift(iaqm_breathVocEquivalent);

    json["iaqm_temperature"] = String(iaqm_temperature[0]);
    json["iaqm_humidity"] = String(iaqm_humidity[0]);
    json["iaqm_pressure"] = String(iaqm_pressure[0]);

    if (iaqIniting)
    {
      if (millis() - startLoopMs > 360000)
      {
        iaqIniting = false;
      }
    }
    else
    {
      json["iaqm_iaq"] = String(iaqm_iaq[0]);
      json["iaqm_co2Equivalent"] = String(iaqm_co2Equivalent[0]);
      json["iaqm_breathVocEquivalent"] = String(iaqm_breathVocEquivalent[0] * 1000);
    }

    bme680HasNewData = false;
  }

#endif

  if (!json.isNull())
  {
    serializeJson(json, pubPayload);
    mqttClient.publish(hamdhHG.getStateTopic(), pubPayload);
  }
}

void loop()
{
  processSerialCommands();
  wifiManager.process();
  if (shouldSaveConfig)
  {
    shouldSaveConfig = false;
    if (importPortalConfig()) saveConfigToFS();
  }
  if (!mqttStarted && WiFi.status() == WL_CONNECTED && mqtt_server[0]) mqttStart();
#if (IR_MODULE && IR_HITACHI_AC3)
  processHitachiAc3Event();
#endif
  //mqttClient.loop();
#if (THI_MODULE || IAQ_MODULE) // if there is sensor need to public

#if THI_MODULE
  if (millis() - lastReadTHIMs > 2000 && bh1750_1.measurementReady(true) && bh1750_2.measurementReady(true)) // read thi mod data to the arrays
  {
    shtc3.update();
    if (shtc3.lastStatus == SHTC3_Status_Nominal)
    {
      thim_temperature[1] = shtc3.toDegC();
      thim_humidity[1] = shtc3.toPercent();
    }

    int tempInt = round(thim_temperature[1]);
    float tempFactor = 1.0f - (tempInt - 20.0f) * 0.0005f;

    float lux1 = bh1750_1.readLightLevel();
    float lux2 = bh1750_2.readLightLevel();

    bh1750_1.configure(BH1750::ONE_TIME_HIGH_RES_MODE);
    bh1750_2.configure(BH1750::ONE_TIME_HIGH_RES_MODE);
    thim_illuminance[1] = fmaxf(lux1, lux2) * tempFactor;

    if (abs(thim_temperature[0] - thim_temperature[1]) > 0.3 || abs(thim_humidity[0] - thim_humidity[1]) > 3)
    {
      thimEvent = true;
    }

    thimHasNewData = true;
    lastReadTHIMs = millis();
  }
#endif

#if IAQ_MODULE

  if (bme680.run()) // read bme680 data to the arrays
  {
    bme680HasNewData = true;
    iaqm_temperature[1] = bme680.temperature;
    iaqm_humidity[1] = bme680.humidity;
    iaqm_pressure[1] = bme680.pressure;
    iaqm_iaq[1] = bme680.staticIaq;
    iaqm_co2Equivalent[1] = bme680.co2Equivalent;
    iaqm_breathVocEquivalent[1] = bme680.breathVocEquivalent;

    if (abs(iaqm_temperature[0] - iaqm_temperature[1]) > 0.3 || abs(iaqm_humidity[0] - iaqm_humidity[1]) > 3 || abs(iaqm_iaq[0] - iaqm_iaq[1]) > 15)
    {
      iaqmEvent = true; // event flag ture if measurements change too fast
    }
  }

#endif

  if (mqttClient.isConnected())
  {
    // time driven and event drivent public
    if ((millis() - lastPubMs > timeDrivenPubInterval * 1000)
#if IAQ_MODULE
        || iaqmEvent
#endif
#if THI_MODULE
        || thimEvent
#endif
    )
    {
      pubSensorsData();
      lastPubMs = millis();

#if IAQ_MODULE
      iaqmEvent = false; // reset event flags
#endif
#if THI_MODULE
      thimEvent = false;
#endif
    }
  }
#endif

#if ZGB_MODULE
  size_t bytes_read;
  uint8_t net_buf[BUFFER_SIZE];
  uint8_t serial_buf[BUFFER_SIZE];

  if (WiFi.status() != WL_CONNECTED)
  {
    // we've lost the connection, so we need to reconnect
    if (wClient)
    {
      wClient.stop();
    }
  }
  // Check if a client has connected
  if (!wClient.connected())
  {
    // eat any bytes in the serial buffer as there is nothing to see them
    while (zgbSerial.available())
    {
      zgbSerial.read();
    }

    wClient = ser2NetServer.available();
  }

#define min(a, b) ((a) < (b) ? (a) : (b))

  if (wClient.connected())
  {
    // check the network for any bytes to send to the serial
    int count = wClient.available();
    if (count > 0)
    {
      bytes_read = wClient.read(net_buf, min(count, BUFFER_SIZE));
      zgbSerial.write(net_buf, bytes_read);
      zgbSerial.flush();
    }

    // now check the serial for any bytes to send to the network
    bytes_read = 0;
    while (zgbSerial.available() && bytes_read < BUFFER_SIZE)
    {
      serial_buf[bytes_read] = zgbSerial.read();
      bytes_read++;
    }

    if (bytes_read > 0)
    {
      wClient.write((const uint8_t *)serial_buf, bytes_read);
      wClient.flush();
    }
  }
  else
  {
    wClient.stop();
  }
#endif


}

esp_err_t handleMQTT(esp_mqtt_event_handle_t event)
{
#if (IR_MODULE && IR_HITACHI_AC3)
    // Reject the whole Hitachi message before the MQTT library can dispatch a
    // partial payload. The SDK reports the topic only on the first fragment.
    static bool discardHitachiMessage = false;
    if (event && event->event_id == MQTT_EVENT_DATA)
    {
      if (event->current_data_offset == 0)
      {
        discardHitachiMessage = false;
        String topic = hamdhHitachi.getCtrlTopic();
        bool isHitachiTopic = event->topic &&
            event->topic_len == static_cast<int>(topic.length()) &&
            memcmp(event->topic, topic.c_str(), topic.length()) == 0;
        if (isHitachiTopic &&
            (event->retain || event->total_data_len > static_cast<int>(HitachiAc3Control::kMaxCommandLength) ||
             event->data_len != event->total_data_len ||
             (event->data && event->data_len > 0 && memchr(event->data, '\0', event->data_len))))
        {
          discardHitachiMessage = true;
          Serial.println("HITACHI ERROR RETAINED_FRAGMENTED_OR_INVALID_COMMAND_IGNORED");
        }
      }
      if (discardHitachiMessage)
      {
        if (static_cast<int64_t>(event->current_data_offset) + event->data_len >= event->total_data_len)
          discardHitachiMessage = false;
        return ESP_OK;
      }
    }
#endif
    mqttClient.onEventCallback(event);
    return ESP_OK;
}