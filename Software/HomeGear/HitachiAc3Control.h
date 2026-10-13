#pragma once

#include <cstddef>
#include <cstdint>

// The user's Hitachi air conditioner has been tested with these fixed
// 25 C, dry, low-fan frames. Other temperatures and modes are not exposed.
// This class only stores command state and bytes; it never sends IR.
class HitachiAc3Control {
 public:
  static constexpr std::size_t kStateLength = 23;
  static constexpr std::size_t kMaxCommandLength = 128;
  static constexpr std::uint8_t kDefaultTemperature = 25;
  static constexpr std::uint8_t kDryLowModeByte = 0x14;
  static constexpr std::uint8_t kPowerOnByte = 0x90;
  static constexpr std::uint8_t kPowerOffByte = 0xA0;

  HitachiAc3Control()
      : known_(false), power_(false), raw_{
            0x01, 0x10, 0x00, 0x40, 0xBF, 0xFF, 0x00, 0xE6,
            0x19, 0x89, 0x76, 0x01, 0xFE, 0x3F, 0xC0, 0x25,
            0xDA, kDryLowModeByte, 0xEB, 0x00, 0xFF,
            kPowerOffByte, 0x5F} {}

  // This deliberately small protocol accepts one literal key and one literal
  // value. Escapes, duplicate keys, additional objects and trailing bytes are
  // rejected rather than delegated to a permissive general JSON decoder.
  static bool parseModeCommand(const char *payload, std::size_t length,
                               bool &power) {
    if (!payload || !length || length > kMaxCommandLength) return false;
    std::size_t cursor = 0;
    auto skipWhitespace = [&]() {
      while (cursor < length &&
             (payload[cursor] == ' ' || payload[cursor] == '\t' ||
              payload[cursor] == '\r' || payload[cursor] == '\n'))
        ++cursor;
    };
    auto consume = [&](char byte) {
      if (cursor == length || payload[cursor] != byte) return false;
      ++cursor;
      return true;
    };
    auto consumeLiteral = [&](const char *literal) {
      std::size_t next = cursor;
      for (std::size_t i = 0; literal[i] != '\0'; ++i) {
        if (next == length || payload[next] != literal[i]) return false;
        ++next;
      }
      cursor = next;
      return true;
    };

    skipWhitespace();
    if (!consume('{')) return false;
    skipWhitespace();
    if (!consumeLiteral("\"achitachi_mode\"")) return false;
    skipWhitespace();
    if (!consume(':')) return false;
    skipWhitespace();
    bool requestedPower;
    if (consumeLiteral("\"dry\""))
      requestedPower = true;
    else if (consumeLiteral("\"off\""))
      requestedPower = false;
    else
      return false;
    skipWhitespace();
    if (!consume('}')) return false;
    skipWhitespace();
    if (cursor != length) return false;
    power = requestedPower;
    return true;
  }

  void setPower(bool on) {
    power_ = on;
    known_ = true;
    raw_[21] = on ? kPowerOnByte : kPowerOffByte;
    raw_[22] = static_cast<std::uint8_t>(~raw_[21]);
  }

  bool isKnown() const { return known_; }
  bool getPower() const { return power_; }
  const std::uint8_t *getRaw() const { return raw_; }
  std::uint8_t getTemperature() const { return kDefaultTemperature; }
  const char *getMode() const {
    return known_ ? (power_ ? "dry" : "off") : "unknown";
  }
  const char *getFanMode() const { return "low"; }
  const char *getStateSource() const { return known_ ? "commanded" : "unknown"; }

 private:
  bool known_;
  bool power_;
  std::uint8_t raw_[kStateLength];
};
