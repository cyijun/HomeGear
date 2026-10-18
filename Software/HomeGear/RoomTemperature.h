#pragma once

#include <cstdint>

class String;

namespace homegear {
namespace sensors {
namespace room {

constexpr uint32_t kMaxAgeMs = 10000;
constexpr float kChangeThresholdCelsius = 0.1f;

struct Reading {
  bool available;
  float celsius;
  const char *source;
};

// Loop-owned room temperature. Only validated SHTC3 readings are accepted;
// the heated air-quality module is not a fallback room thermometer.
Reading reading(uint32_t now);
String topic();
void reset();
void update(float celsius, uint32_t now);
void invalidate();
void tick(uint32_t now, bool force = false);

}  // namespace room
}  // namespace sensors
}  // namespace homegear
