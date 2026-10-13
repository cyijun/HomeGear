#include "ConfigStore.h"
#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
namespace homegear { namespace config {
namespace {
Values activeConfig;
Values pendingConfig;
bool configDirty = false;
uint8_t configEditedFields = 0;
bool configFsReady = false;
const size_t kMaxConfigFileLength = 4096;
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

}
void begin() { loadConfigFromFS(); }
const Values &active() { return activeConfig; }
const Values &pending() { return pendingConfig; }
bool dirty() { return configDirty; }
bool save() { return saveConfigToFS(); }
bool stage(Key key, const char *value, size_t length) {
  if (!HomeGearSerialConfig::setValue(pendingConfig, key, value, length)) return false;
  configEditedFields |= static_cast<uint8_t>(1U << static_cast<unsigned>(key));
  configDirty = true;
  return true;
}
bool stagePortal(const char *const values[])
{
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
    if (!length && key == HomeGearSerialConfig::Key::MqttServer) continue;
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

}}
