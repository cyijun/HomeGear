#pragma once

#include <cstddef>
#include <cstring>

// Pure command parsing and bounded validation. No filesystem, network or IR.
class HomeGearSerialConfig {
 public:
  static constexpr std::size_t kHostCapacity = 64;
  static constexpr std::size_t kPortCapacity = 6;
  static constexpr std::size_t kCredentialCapacity = 32;
  static constexpr std::size_t kTokenCapacity = 256;
  static constexpr std::size_t kMaxLineLength = 512;
  enum class Key { MqttServer, MqttPort, MqttUser, MqttPwd,
                   HomeassistantServer, HomeassistantPort, HomeassistantToken,
                   Invalid };
  enum class Operation { Show, Set, Save, Apply };
  struct Command {
    Operation operation;
    Key key;
    const char *value;
    std::size_t valueLength;
  };
  struct Values {
    char mqtt_server[kHostCapacity] = {};
    char mqtt_port[kPortCapacity] = "1883";
    char mqtt_user[kCredentialCapacity] = {};
    char mqtt_pwd[kCredentialCapacity] = {};
    char homeassistant_server[kHostCapacity] = {};
    char homeassistant_port[kPortCapacity] = "8123";
    char homeassistant_token[kTokenCapacity] = {};
  };

  static const char *keyName(Key key) {
    switch (key) {
      case Key::MqttServer: return "mqtt_server";
      case Key::MqttPort: return "mqtt_port";
      case Key::MqttUser: return "mqtt_user";
      case Key::MqttPwd: return "mqtt_pwd";
      case Key::HomeassistantServer: return "homeassistant_server";
      case Key::HomeassistantPort: return "homeassistant_port";
      case Key::HomeassistantToken: return "homeassistant_token";
      default: return "invalid";
    }
  }
  static Key findKey(const char *name, std::size_t length) {
    if (!name) return Key::Invalid;
    for (int i = 0; i < static_cast<int>(Key::Invalid); ++i) {
      Key key = static_cast<Key>(i);
      const char *expected = keyName(key);
      if (std::strlen(expected) == length && std::memcmp(name, expected, length) == 0)
        return key;
    }
    return Key::Invalid;
  }
  static std::size_t capacity(Key key) {
    switch (key) {
      case Key::MqttServer:
      case Key::HomeassistantServer: return kHostCapacity;
      case Key::MqttPort:
      case Key::HomeassistantPort: return kPortCapacity;
      case Key::MqttUser:
      case Key::MqttPwd: return kCredentialCapacity;
      case Key::HomeassistantToken: return kTokenCapacity;
      default: return 0;
    }
  }
  static char *field(Values &values, Key key) {
    switch (key) {
      case Key::MqttServer: return values.mqtt_server;
      case Key::MqttPort: return values.mqtt_port;
      case Key::MqttUser: return values.mqtt_user;
      case Key::MqttPwd: return values.mqtt_pwd;
      case Key::HomeassistantServer: return values.homeassistant_server;
      case Key::HomeassistantPort: return values.homeassistant_port;
      case Key::HomeassistantToken: return values.homeassistant_token;
      default: return nullptr;
    }
  }
  static bool isSecret(Key key) {
    return key == Key::MqttPwd || key == Key::HomeassistantToken;
  }
  static bool validateValue(Key key, const char *value, std::size_t length) {
    if (key == Key::Invalid || !value || length >= capacity(key)) return false;
    for (std::size_t i = 0; i < length; ++i)
      if (static_cast<unsigned char>(value[i]) < 32 ||
          static_cast<unsigned char>(value[i]) == 127) return false;
    if (key == Key::MqttPort || key == Key::HomeassistantPort) {
      if (!length) return false;
      unsigned port = 0;
      for (std::size_t i = 0; i < length; ++i) {
        if (value[i] < '0' || value[i] > '9') return false;
        port = port * 10 + static_cast<unsigned>(value[i] - '0');
      }
      return port >= 1 && port <= 65535;
    }
    if (key == Key::MqttServer || key == Key::HomeassistantServer) {
      if (!length) return false;
      for (std::size_t i = 0; i < length; ++i) {
        const char c = value[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '_' ||
              c == ':' || c == '[' || c == ']')) return false;
      }
    }
    return true;
  }
  static bool setValue(Values &values, Key key, const char *value, std::size_t length) {
    if (!validateValue(key, value, length)) return false;
    char *destination = field(values, key);
    std::memcpy(destination, value, length);
    destination[length] = '\0';
    return true;
  }
  static bool parseCommand(const char *line, std::size_t length, Command &result) {
    if (!line || !length || length > kMaxLineLength) return false;
    for (std::size_t i = 0; i < length; ++i)
      if (static_cast<unsigned char>(line[i]) < 32 ||
          static_cast<unsigned char>(line[i]) == 127) return false;
    std::size_t cursor = 0;
    auto spaces = [&]() { while (cursor < length && line[cursor] == ' ') ++cursor; };
    auto token = [&](const char *expected) {
      const std::size_t n = std::strlen(expected);
      if (length - cursor < n || std::memcmp(line + cursor, expected, n) != 0 ||
          (cursor + n < length && line[cursor + n] != ' ')) return false;
      cursor += n;
      return true;
    };
    spaces();
    if (!token("config") || cursor == length) return false;
    spaces();
    Command parsed = {Operation::Show, Key::Invalid, nullptr, 0};
    if (token("show")) parsed.operation = Operation::Show;
    else if (token("save")) parsed.operation = Operation::Save;
    else if (token("apply")) parsed.operation = Operation::Apply;
    else if (token("set")) {
      parsed.operation = Operation::Set;
      spaces();
      const std::size_t start = cursor;
      while (cursor < length && line[cursor] != ' ') ++cursor;
      parsed.key = findKey(line + start, cursor - start);
      if (parsed.key == Key::Invalid) return false;
      if (cursor < length) ++cursor; // Remaining spaces belong to the value.
      parsed.value = line + cursor;
      parsed.valueLength = length - cursor;
      // An omitted value or the explicit empty string clears a credential.
      if (parsed.valueLength == 2 && line[cursor] == '"' && line[cursor + 1] == '"')
        parsed.valueLength = 0;
      if (!validateValue(parsed.key, parsed.value, parsed.valueLength)) return false;
      result = parsed;
      return true;
    } else return false;
    spaces();
    if (cursor != length) return false;
    result = parsed;
    return true;
  }
};
