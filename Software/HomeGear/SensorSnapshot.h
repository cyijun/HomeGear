#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace homegear {
namespace sensors {

enum class Field : uint8_t {
  ThTemperature, ThHumidity, ThIlluminance,
  IaqTemperature, IaqHumidity, IaqPressure, IaqIndex, IaqCo2, IaqVoc,
  Count
};

constexpr size_t kFieldCount = static_cast<size_t>(Field::Count);
constexpr uint32_t kPublishIntervalMs = 15000;
constexpr uint32_t kHeartbeatIntervalMs = 300000;
constexpr uint32_t kPublishRetryMs = 1000;

inline bool elapsed(uint32_t now, uint32_t then, uint32_t interval) {
  return static_cast<uint32_t>(now - then) >= interval;
}

inline bool validReading(Field field, float value) {
  if (!std::isfinite(value)) return false;
  switch (field) {
    case Field::ThTemperature: return value >= -40.0f && value <= 125.0f;
    case Field::IaqTemperature: return value >= -40.0f && value <= 85.0f;
    case Field::ThHumidity:
    case Field::IaqHumidity: return value >= 0.0f && value <= 100.0f;
    case Field::ThIlluminance: return value >= 0.0f;
    case Field::IaqPressure: return value > 0.0f;
    case Field::IaqIndex: return value >= 0.0f && value <= 500.0f;
    case Field::IaqCo2: return value > 0.0f;
    case Field::IaqVoc: return value >= 0.0f;
    default: return false;
  }
}

inline float changeThreshold(Field field, float published) {
  switch (field) {
    case Field::ThTemperature:
    case Field::IaqTemperature: return 0.3f;
    case Field::ThHumidity:
    case Field::IaqHumidity: return 3.0f;
    case Field::ThIlluminance: return std::fmax(1.0f, std::fabs(published) * 0.05f);
    case Field::IaqPressure: return 50.0f;
    case Field::IaqIndex: return 15.0f;
    case Field::IaqCo2: return 50.0f;
    case Field::IaqVoc: return std::fmax(10.0f, std::fabs(published) * 0.10f);
    default: return 0.0f;
  }
}

struct Snapshot {
  float values[kFieldCount] = {};
  bool known[kFieldCount] = {};

  bool anyKnown() const {
    for (size_t i = 0; i < kFieldCount; ++i) if (known[i]) return true;
    return false;
  }
};

// Sampling, snapshot creation and commit are loop-owned. A callback only sets
// the atomic request flag in SensorService, never this state.
class SnapshotPublisher {
 public:
  bool update(Field field, float value) {
    if (!validReading(field, value)) return false;
    const size_t i = static_cast<size_t>(field);
    latest_.values[i] = value;
    latest_.known[i] = true;
    return true;
  }

  const Snapshot &snapshot() const { return latest_; }

  void request() { forced_ = true; }

  bool dirty() const {
    for (size_t i = 0; i < kFieldCount; ++i) {
      if (!latest_.known[i]) continue;
      if (!published_.known[i] ||
          std::fabs(latest_.values[i] - published_.values[i]) >=
              changeThreshold(static_cast<Field>(i), published_.values[i]))
        return true;
    }
    return false;
  }

  bool shouldPublish(uint32_t now) const {
    if (!latest_.anyKnown()) return false;
    if (failedAttempt_ && !elapsed(now, lastAttempt_, kPublishRetryMs)) return false;
    if (forced_ || !hasPublished_) return true;
    return elapsed(now, lastPublished_, kHeartbeatIntervalMs) ||
           (dirty() && elapsed(now, lastPublished_, kPublishIntervalMs));
  }

  void failed(uint32_t now) {
    lastAttempt_ = now;
    failedAttempt_ = true;
  }

  void committed(uint32_t now, const Snapshot &sent) {
    published_ = sent;
    lastPublished_ = now;
    hasPublished_ = true;
    failedAttempt_ = false;
    forced_ = false;
  }

  bool hasPublished() const { return hasPublished_; }
  uint32_t lastPublished() const { return lastPublished_; }

 private:
  Snapshot latest_;
  Snapshot published_;
  uint32_t lastPublished_ = 0;
  uint32_t lastAttempt_ = 0;
  bool hasPublished_ = false;
  bool forced_ = false;
  bool failedAttempt_ = false;
};

template <typename Consumer>
inline size_t forEachKnown(const Snapshot &snapshot, Consumer consumer) {
  size_t count = 0;
  for (size_t i = 0; i < kFieldCount; ++i) {
    if (!snapshot.known[i]) continue;
    consumer(static_cast<Field>(i), snapshot.values[i]);
    ++count;
  }
  return count;
}

}  // namespace sensors
}  // namespace homegear
