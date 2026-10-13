#include <cassert>
#include <cstring>
#include <iostream>
#include <string>

#include "../Software/HomeGear/SerialConfig.h"

using Key = HomeGearSerialConfig::Key;
using Operation = HomeGearSerialConfig::Operation;

HomeGearSerialConfig::Command parse(const char *line) {
  HomeGearSerialConfig::Command command;
  assert(HomeGearSerialConfig::parseCommand(line, std::strlen(line), command));
  return command;
}

void rejected(const std::string &line) {
  const char sentinel[] = "unchanged";
  HomeGearSerialConfig::Command command = {Operation::Apply, Key::MqttPwd, sentinel, 9};
  assert(!HomeGearSerialConfig::parseCommand(line.data(), line.size(), command));
  assert(command.operation == Operation::Apply && command.key == Key::MqttPwd);
  assert(command.value == sentinel && command.valueLength == 9);
}

int main() {
  assert(parse("config show").operation == Operation::Show);
  assert(parse("config save").operation == Operation::Save);
  assert(parse("  config   apply   ").operation == Operation::Apply);
  for (const std::string &line : {"", "config", "configshow", "config shown",
       "config show extra", "config save extra", "config apply extra",
       "config set", "config set unknown value", "config set mqtt_server",
       "config set mqtt_server http://192.168.0.109", "config set mqtt_port 0",
       "config set mqtt_port 65536", "config set mqtt_port -1",
       "config set mqtt_port 1883 extra", "config\tshow"}) rejected(line);
  rejected(std::string("config set mqtt_pwd x\0y", 23));
  rejected("config set mqtt_pwd x\ry");
  rejected("config set mqtt_pwd x\ny");
  rejected("config set mqtt_pwd " + std::string(32, 'p'));
  rejected("config set homeassistant_token " + std::string(HomeGearSerialConfig::kTokenCapacity, 't'));
  rejected("config show" + std::string(HomeGearSerialConfig::kMaxLineLength, ' '));

  HomeGearSerialConfig::Values values;
  assert(std::strcmp(values.mqtt_port, "1883") == 0);
  assert(std::strcmp(values.homeassistant_port, "8123") == 0);
  assert(HomeGearSerialConfig::setValue(values, Key::MqttUser, "test-user", 9));
  assert(HomeGearSerialConfig::setValue(values, Key::MqttPwd, " old password ", 14));
  assert(HomeGearSerialConfig::setValue(values, Key::HomeassistantToken, "test-token", 10));
  HomeGearSerialConfig::Command command = parse("config set mqtt_server 192.168.0.109");
  assert(command.operation == Operation::Set && command.key == Key::MqttServer);
  assert(HomeGearSerialConfig::setValue(values, command.key, command.value, command.valueLength));
  assert(std::strcmp(values.mqtt_server, "192.168.0.109") == 0);
  // A server change must preserve the connection's other settings.
  assert(std::strcmp(values.mqtt_port, "1883") == 0);
  assert(std::strcmp(values.mqtt_user, "test-user") == 0);
  assert(std::strcmp(values.mqtt_pwd, " old password ") == 0);
  assert(std::strcmp(values.homeassistant_token, "test-token") == 0);

  command = parse("config set mqtt_pwd  spaced password ");
  assert(HomeGearSerialConfig::setValue(values, command.key, command.value, command.valueLength));
  assert(std::strcmp(values.mqtt_pwd, " spaced password ") == 0);
  for (const char *line : {"config set mqtt_pwd", "config set mqtt_pwd \"\""}) {
    command = parse(line);
    assert(command.valueLength == 0);
    assert(HomeGearSerialConfig::setValue(values, command.key, command.value, command.valueLength));
    assert(values.mqtt_pwd[0] == '\0');
  }
  const std::string maxHost(63, 'a'), maxPassword(31, 'p'), maxToken(HomeGearSerialConfig::kTokenCapacity - 1, 't');
  const std::string longTokenCommand = "config set homeassistant_token " + maxToken;
  assert(HomeGearSerialConfig::parseCommand(longTokenCommand.data(), longTokenCommand.size(), command));
  assert(command.key == Key::HomeassistantToken && command.valueLength == maxToken.size());
  const std::string maxLine = "config show" + std::string(HomeGearSerialConfig::kMaxLineLength - 11, ' ');
  assert(HomeGearSerialConfig::parseCommand(maxLine.data(), maxLine.size(), command));
  assert(command.operation == Operation::Show);
  assert(HomeGearSerialConfig::setValue(values, Key::MqttServer, maxHost.data(), maxHost.size()));
  assert(values.mqtt_server[63] == '\0');
  assert(HomeGearSerialConfig::setValue(values, Key::MqttPwd, maxPassword.data(), maxPassword.size()));
  assert(values.mqtt_pwd[31] == '\0');
  assert(HomeGearSerialConfig::setValue(values, Key::HomeassistantToken, maxToken.data(), maxToken.size()));
  assert(values.homeassistant_token[HomeGearSerialConfig::kTokenCapacity - 1] == '\0');
  for (const char *port : {"1", "1883", "8123", "65535"})
    assert(HomeGearSerialConfig::validateValue(Key::MqttPort, port, std::strlen(port)));
  for (const char *port : {"", "0", "00000", "65536", "99999", "100000", "1.0", " 1883", "1883 "})
    assert(!HomeGearSerialConfig::validateValue(Key::MqttPort, port, std::strlen(port)));

  const HomeGearSerialConfig::Values before = values;
  assert(!HomeGearSerialConfig::setValue(values, Key::MqttServer, maxHost.data(), 64));
  assert(!HomeGearSerialConfig::setValue(values, Key::MqttPort, "65536", 5));
  assert(!HomeGearSerialConfig::setValue(values, Key::MqttPwd, nullptr, 0));
  assert(!HomeGearSerialConfig::setValue(values, Key::Invalid, "x", 1));
  assert(std::memcmp(&values, &before, sizeof(values)) == 0);
  assert(HomeGearSerialConfig::isSecret(Key::MqttPwd));
  assert(HomeGearSerialConfig::isSecret(Key::HomeassistantToken));
  assert(!HomeGearSerialConfig::isSecret(Key::MqttServer));
  assert(!HomeGearSerialConfig::isSecret(Key::MqttUser));
  HomeGearSerialConfig::Command unused;
  assert(!HomeGearSerialConfig::parseCommand(nullptr, 1, unused));
  assert(HomeGearSerialConfig::findKey(nullptr, 1) == Key::Invalid);
  std::cout << "Serial configuration validation and preservation tests passed\n";
}
