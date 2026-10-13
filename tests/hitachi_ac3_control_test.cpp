#include <algorithm>
#include <array>
#include <cassert>
#include <climits>
#include <cstring>
#include <iostream>
#include <string>

#include "../Software/HomeGear/HitachiAc3Control.h"

// Golden frames independently confirmed on the user's wall controller.
const unsigned char kVerifiedOff[] = {
    0x01, 0x10, 0x00, 0x40, 0xBF, 0xFF, 0x00, 0xE6,
    0x19, 0x89, 0x76, 0x01, 0xFE, 0x3F, 0xC0, 0x25,
    0xDA, 0x14, 0xEB, 0x00, 0xFF, 0xA0, 0x5F};
const unsigned char kVerifiedOn[] = {
    0x01, 0x10, 0x00, 0x40, 0xBF, 0xFF, 0x00, 0xE6,
    0x19, 0x89, 0x76, 0x01, 0xFE, 0x3F, 0xC0, 0x25,
    0xDA, 0x14, 0xEB, 0x00, 0xFF, 0x90, 0x6F};

// Independent temperature pairs transcribed from the original PC-LH3B
// captures (17-30 C); not calculated using the production encoder.
const unsigned char kCapturedTemperaturePairs[][2] = {
    {0x15, 0xEA}, {0x17, 0xE8}, {0x19, 0xE6}, {0x1B, 0xE4},
    {0x1D, 0xE2}, {0x1F, 0xE0}, {0x21, 0xDE}, {0x23, 0xDC},
    {0x25, 0xDA}, {0x27, 0xD8}, {0x29, 0xD6}, {0x2B, 0xD4},
    {0x2D, 0xD2}, {0x2F, 0xD0}};

using Command = HitachiAc3Control::Command;

std::array<unsigned char, HitachiAc3Control::kStateLength> frameCopy(
    const HitachiAc3Control &control) {
  std::array<unsigned char, HitachiAc3Control::kStateLength> frame;
  std::copy_n(control.getRaw(), frame.size(), frame.begin());
  return frame;
}

void assertFrame(const HitachiAc3Control &control, const unsigned char *expected) {
  assert(std::equal(expected, expected + sizeof(kVerifiedOff), control.getRaw()));
  for (std::size_t i = 3; i < HitachiAc3Control::kStateLength; i += 2)
    assert((control.getRaw()[i] ^ control.getRaw()[i + 1]) == 0xFF);
}

void assertCommand(const std::string &command, bool expectedPower) {
  bool power = !expectedPower;
  assert(HitachiAc3Control::parseModeCommand(command.data(), command.size(), power));
  assert(power == expectedPower);
  Command parsed;
  parsed.type = Command::Type::Temperature;
  parsed.temperature = 19;
  assert(HitachiAc3Control::parseCommand(command.data(), command.size(), parsed));
  assert(parsed.type == Command::Type::Mode && parsed.power == expectedPower);
}

void assertRejected(const std::string &command) {
  for (bool previousPower : {false, true}) {
    bool power = previousPower;
    assert(!HitachiAc3Control::parseModeCommand(command.data(), command.size(), power));
    assert(power == previousPower);
  }
}

void assertInvalidCommand(const std::string &payload) {
  assertRejected(payload);
  for (Command::Type type : {Command::Type::Mode, Command::Type::Temperature}) {
    for (bool previousPower : {false, true}) {
      Command parsed;
      parsed.type = type;
      parsed.power = previousPower;
      parsed.temperature = 29;
      assert(!HitachiAc3Control::parseCommand(payload.data(), payload.size(), parsed));
      assert(parsed.type == type && parsed.power == previousPower && parsed.temperature == 29);
    }
  }
}

void assertTemperatureCommand(const std::string &payload, unsigned expected) {
  Command parsed;
  parsed.type = Command::Type::Mode;
  parsed.power = true;
  parsed.temperature = 19;
  assert(HitachiAc3Control::parseCommand(payload.data(), payload.size(), parsed));
  assert(parsed.type == Command::Type::Temperature && parsed.temperature == expected);
  assertRejected(payload);  // Legacy mode-only parser cannot interpret a target as power.
}

void testTemperatureFramesAndState() {
  assert(HitachiAc3Control::kMinTemperature == 17);
  assert(HitachiAc3Control::kMaxTemperature == 30);
  for (unsigned previousState = 0; previousState < 3; ++previousState) {
    HitachiAc3Control control;
    if (previousState) control.setPower(previousState == 2);
    for (unsigned index = 0; index < 14; ++index) {
      const int temperature = 17 + index;
      const bool wasKnown = control.isKnown(), wasOn = control.getPower();
      auto expected = frameCopy(control);
      expected[15] = kCapturedTemperaturePairs[index][0];
      expected[16] = kCapturedTemperaturePairs[index][1];
      assert(control.setTemperature(temperature));
      assert(control.getTemperature() == temperature);
      assert(control.isKnown() == wasKnown && control.getPower() == wasOn);
      assertFrame(control, expected.data());
      assert(control.getRaw()[11] == 0x01 && control.getRaw()[12] == 0xFE);
      assert(control.getRaw()[17] == 0x14 && control.getRaw()[18] == 0xEB);
      assert(control.setTemperature(temperature));  // Repeated targets are absolute/idempotent.
      assertFrame(control, expected.data());
      for (int invalid : {INT_MIN, -300, -1, 0, 16, 31, 32, 300, INT_MAX}) {
        assert(!control.setTemperature(invalid));
        assert(control.getTemperature() == temperature);
        assert(control.isKnown() == wasKnown && control.getPower() == wasOn);
        assertFrame(control, expected.data());
      }
    }
  }
  HitachiAc3Control preset;
  assert(preset.setTemperature(24));
  assert(!preset.isKnown());
  assert(std::strcmp(preset.getMode(), "unknown") == 0);
  assert(std::strcmp(preset.getStateSource(), "unknown") == 0);
  preset.setPower(true);
  preset.setPower(false);
  preset.setPower(true);
  assert(preset.getTemperature() == 24 && preset.isKnown() && preset.getPower());
  auto expected = frameCopy(preset);
  assert(expected[15] == 0x23 && expected[16] == 0xDC);
  assert(preset.setTemperature(25));
  assertFrame(preset, kVerifiedOn);  // Default target still produces the measured golden frame.
  preset.setPower(false);
  assertFrame(preset, kVerifiedOff);
}

void testTemperatureParser() {
  for (unsigned target = 17; target <= 30; ++target) {
    const std::string number = std::to_string(target);
    for (const std::string &value : {number, number + ".0", number + ".000",
                                    "\"" + number + "\"", "\"" + number + ".0\""}) {
      const std::string payload = "{\"achitachi_temperature\":" + value + "}";
      assertTemperatureCommand(payload, target);
      for (std::size_t length = 0; length < payload.size(); ++length)
        assertInvalidCommand(payload.substr(0, length));
      for (std::size_t position = 0; position <= payload.size(); ++position) {
        std::string nul = payload;
        nul.insert(position, 1, '\0');
        assertInvalidCommand(nul);
      }
    }
  }
  assertTemperatureCommand(" \r\n{ \t\"achitachi_temperature\" : 24.00 } \t", 24);
  struct RoundedTarget { const char *value; unsigned target; };
  for (const auto &rounded : {
           RoundedTarget{"17.1", 17}, RoundedTarget{"17.9", 18},
           RoundedTarget{"24.4", 24}, RoundedTarget{"24.5", 25},
           RoundedTarget{"24.0001", 24}, RoundedTarget{"24.4999999999999999999999999", 24},
           RoundedTarget{"24.5000000000000000000000000", 25},
           RoundedTarget{"29.9", 30}, RoundedTarget{"30.0000000000000000000", 30}}) {
    assertTemperatureCommand(std::string("{\"achitachi_temperature\":") + rounded.value + "}", rounded.target);
    assertTemperatureCommand(std::string("{\"achitachi_temperature\":\"") + rounded.value + "\"}", rounded.target);
  }
  const std::string target = "{\"achitachi_temperature\":24}";
  assertTemperatureCommand(target + std::string(128 - target.size(), ' '), 24);
  assertInvalidCommand(target + std::string(129 - target.size(), ' '));
  for (const std::string &value : {
           "16", "16.9", "30.1", "30.0000000000000000000000001", "31", "32", "-24", "+24", "024", "24.", ".24",
           "24e0", "24E+0", "24.5e0", "24..0", "NaN", "Infinity", "true", "null", "[]", "{}",
           "\"\"", "\"16\"", "\"31\"", "\"32\"", "\"+24\"", "\"024\"", "\" 24\"",
           "\"24 \"", "\"24.\"", "\"16.9\"", "\"30.1\"", "\"30.0000000001\"", "\"24e0\"", "\"2\\u0034\"", "\"24\\u0000\"",
           "99999999999999999999999999999999999999"})
    assertInvalidCommand("{\"achitachi_temperature\":" + value + "}");
  for (const std::string &malformed : {
           "{\"achitachi_temperature\":24,}",
           "{\"achitachi_temperature\":24,\"achitachi_temperature\":25}",
           "{\"achitachi_temperature\":24,\"achitachi_mode\":\"dry\"}",
           "{\"achitachi_mode\":\"off\",\"achitachi_temperature\":24}",
           "{\"achitachi_temperatur\\u0065\":24}",
           "{\"achitachi_temperature\":24}{}", "{\"achitachi_temperature\":24}tail",
           "{\"achitachi_temperature\":24 // comment\n}"})
    assertInvalidCommand(malformed);
  Command unchanged;
  unchanged.type = Command::Type::Temperature;
  unchanged.power = true;
  unchanged.temperature = 21;
  assert(!HitachiAc3Control::parseCommand(nullptr, 3, unchanged));
  assert(unchanged.type == Command::Type::Temperature && unchanged.power && unchanged.temperature == 21);
}

int main() {
  static_assert(HitachiAc3Control::kStateLength == sizeof(kVerifiedOff),
                "The verified AC3 command is exactly 23 bytes");
  HitachiAc3Control control;
  assert(!control.isKnown());
  assert(std::strcmp(control.getMode(), "unknown") == 0);
  assert(std::strcmp(control.getStateSource(), "unknown") == 0);
  assertFrame(control, kVerifiedOff);

  // Repeated absolute ON commands cannot toggle an already-running unit off.
  control.setPower(true);
  control.setPower(true);
  assert(control.isKnown() && control.getPower());
  assert(control.getTemperature() == 25);
  assert(std::strcmp(control.getMode(), "dry") == 0);
  assert(std::strcmp(control.getFanMode(), "low") == 0);
  assert(std::strcmp(control.getStateSource(), "commanded") == 0);
  assertFrame(control, kVerifiedOn);

  control.setPower(false);
  control.setPower(false);
  assert(control.isKnown() && !control.getPower());
  assert(std::strcmp(control.getMode(), "off") == 0);
  assertFrame(control, kVerifiedOff);

  // Only the absolute power pair changes between the two captured states.
  std::size_t changed = 0;
  for (std::size_t i = 0; i < sizeof(kVerifiedOn); ++i)
    if (kVerifiedOn[i] != kVerifiedOff[i]) {
      assert(i == 21 || i == 22);
      ++changed;
    }
  assert(changed == 2);

  HitachiAc3Control anotherDevice;
  assert(!anotherDevice.isKnown());
  assertFrame(anotherDevice, kVerifiedOff);

  assertCommand("{\"achitachi_mode\":\"dry\"}", true);
  assertCommand(" \r\n{ \t\"achitachi_mode\" : \"off\" } \t", false);
  const std::string on = "{\"achitachi_mode\":\"dry\"}";
  assertCommand(on + std::string(128 - on.size(), ' '), true);
  assertRejected(on + std::string(129 - on.size(), ' '));
  for (const std::string &invalid : {
           "", "[]", "{}", "null", "{\"achitachi_mode\":true}",
           "{\"achitachi_mode\":\"cool\"}",
           "{\"achitachi_mode\":\"dry\",\"achitachi_temperature\":30}",
           "{\"achitachi_mode\":\"cool\",\"achitachi_mode\":\"dry\"}",
           "{\"achitachi_mode\":\"dry\"}trailing",
           "{\"achitachi_mode\":\"dry\"}{\"temperature\":30}",
           "{\"achitachi_mode\":\"dry\\u0000cool\"}",
           "{\"achitachi_mode\":\"dry\\\"\"}",
           "{\"achitachi_mode\":\"dry\"", "{\"achitachi_mode\":\"dry\",}"})
    assertInvalidCommand(invalid);
  assertInvalidCommand(on + std::string(1, '\0') + "trailing");
  bool unchanged = true;
  assert(!HitachiAc3Control::parseModeCommand(nullptr, 3, unchanged));
  assert(unchanged);
  testTemperatureFramesAndState();
  testTemperatureParser();
  std::cout << "Hitachi AC3: verified frames, captured 17-30 C targets, preserved power/unknown "
               "state and strict mode/temperature commands passed\n";
}
