#include <algorithm>
#include <cassert>
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

void assertFrame(const HitachiAc3Control &control, const unsigned char *expected) {
  assert(std::equal(expected, expected + sizeof(kVerifiedOff), control.getRaw()));
  for (std::size_t i = 3; i < HitachiAc3Control::kStateLength; i += 2)
    assert((control.getRaw()[i] ^ control.getRaw()[i + 1]) == 0xFF);
}

void assertCommand(const std::string &command, bool expectedPower) {
  bool power = !expectedPower;
  assert(HitachiAc3Control::parseModeCommand(command.data(), command.size(), power));
  assert(power == expectedPower);
}

void assertRejected(const std::string &command) {
  for (bool previousPower : {false, true}) {
    bool power = previousPower;
    assert(!HitachiAc3Control::parseModeCommand(command.data(), command.size(), power));
    assert(power == previousPower);
  }
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
    assertRejected(invalid);
  assertRejected(on + std::string(1, '\0') + "trailing");
  bool unchanged = true;
  assert(!HitachiAc3Control::parseModeCommand(nullptr, 3, unchanged));
  assert(unchanged);
  std::cout << "Hitachi AC3: verified frames, unknown state, absolute power and strict commands passed\n";
}
