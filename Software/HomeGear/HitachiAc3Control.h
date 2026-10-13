#pragma once

#include <cstddef>
#include <cstdint>

// The user's Hitachi air conditioner has been tested with these 23-byte
// dry, low-fan ON/OFF frames at 25 C. Temperature encoding for 17-30 C is
// supported by the PC-LH3B's original captures; compatibility still needs
// confirmation on the user's wall controller. Other modes are not exposed.
// This class only stores command state and bytes; it never sends IR.
class HitachiAc3Control {
 public:
  static constexpr std::size_t kStateLength = 23;
  static constexpr std::size_t kMaxCommandLength = 128;
  static constexpr std::uint8_t kMinTemperature = 17;
  static constexpr std::uint8_t kMaxTemperature = 30;
  static constexpr std::uint8_t kDefaultTemperature = 25;
  static constexpr std::uint8_t kDryLowModeByte = 0x14;
  static constexpr std::uint8_t kPowerOnByte = 0x90;
  static constexpr std::uint8_t kPowerOffByte = 0xA0;

  struct Command {
    enum class Type { Mode, Temperature };
    Type type = Type::Mode;
    bool power = false;
    std::uint8_t temperature = kDefaultTemperature;
  };

  HitachiAc3Control()
      : known_(false), power_(false), temperature_(kDefaultTemperature), raw_{
            0x01, 0x10, 0x00, 0x40, 0xBF, 0xFF, 0x00, 0xE6,
            0x19, 0x89, 0x76, 0x01, 0xFE, 0x3F, 0xC0, 0x25,
            0xDA, kDryLowModeByte, 0xEB, 0x00, 0xFF,
            kPowerOffByte, 0x5F} {}

  // Accept exactly one unescaped key. HA/HomeKit can send decimal targets as
  // a JSON number or string. Validate the original value against 17-30 C,
  // then round half up to the integer protocol target without floating point.
  // Escapes, duplicate keys, exponents and trailing bytes are rejected here.
  static bool parseCommand(const char *payload, std::size_t length,
                           Command &command) {
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
    Command parsed;
    if (consumeLiteral("\"achitachi_mode\""))
      parsed.type = Command::Type::Mode;
    else if (consumeLiteral("\"achitachi_temperature\""))
      parsed.type = Command::Type::Temperature;
    else
      return false;
    skipWhitespace();
    if (!consume(':')) return false;
    skipWhitespace();
    if (parsed.type == Command::Type::Mode) {
      if (consumeLiteral("\"dry\""))
        parsed.power = true;
      else if (consumeLiteral("\"off\""))
        parsed.power = false;
      else
        return false;
    } else {
      const bool quoted = consume('"');
      // Every supported target is exactly two decimal digits. Requiring this
      // also rejects leading zeros and avoids overflow with very long input.
      if (cursor + 2 > length || payload[cursor] < '1' || payload[cursor] > '3' ||
          payload[cursor + 1] < '0' || payload[cursor + 1] > '9')
        return false;
      int target = (payload[cursor] - '0') * 10 + payload[cursor + 1] - '0';
      cursor += 2;
      if (target < kMinTemperature || target > kMaxTemperature) return false;
      if (consume('.')) {
        if (cursor == length || payload[cursor] < '0' || payload[cursor] > '9')
          return false;
        const bool roundUp = payload[cursor] >= '5';
        bool nonZeroFraction = false;
        while (cursor < length && payload[cursor] >= '0' && payload[cursor] <= '9') {
          nonZeroFraction = nonZeroFraction || payload[cursor] != '0';
          ++cursor;
        }
        if (target == kMaxTemperature && nonZeroFraction) return false;
        if (roundUp) ++target;
      }
      if (quoted && !consume('"')) return false;
      parsed.temperature = static_cast<std::uint8_t>(target);
    }
    skipWhitespace();
    if (!consume('}')) return false;
    skipWhitespace();
    if (cursor != length) return false;
    command = parsed;
    return true;
  }

  static bool parseModeCommand(const char *payload, std::size_t length,
                               bool &power) {
    Command parsed;
    if (!parseCommand(payload, length, parsed) || parsed.type != Command::Type::Mode)
      return false;
    power = parsed.power;
    return true;
  }

  bool setTemperature(int celsius) {
    if (celsius < kMinTemperature || celsius > kMaxTemperature) return false;
    temperature_ = static_cast<std::uint8_t>(celsius);
    raw_[15] = static_cast<std::uint8_t>(2 * celsius - 13);
    raw_[16] = static_cast<std::uint8_t>(~raw_[15]);
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
  std::uint8_t getTemperature() const { return temperature_; }
  const char *getMode() const {
    return known_ ? (power_ ? "dry" : "off") : "unknown";
  }
  const char *getFanMode() const { return "low"; }
  const char *getStateSource() const { return known_ ? "commanded" : "unknown"; }

 private:
  bool known_;
  bool power_;
  std::uint8_t temperature_;
  std::uint8_t raw_[kStateLength];
};
