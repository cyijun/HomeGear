#include "RoomTemperature.h"

#include <Arduino.h>
#include <cmath>
#include "ESP32MQTTClient.h"
#include "HAMDHandler.h"
#include "MqttService.h"
#include "SensorSnapshot.h"

namespace homegear {
namespace sensors {
namespace room {
namespace {

bool sampleAvailable = false;
float latestCelsius = 0.0f;
uint32_t sampledAt = 0;
bool hasPublished = false;
bool publishedAvailable = false;
float publishedCelsius = 0.0f;
uint32_t publishedAt = 0;
bool forced = false;
bool failedAttempt = false;
uint32_t attemptedAt = 0;

}  // namespace

Reading reading(uint32_t now) {
  const bool available = sampleAvailable &&
      static_cast<uint32_t>(now - sampledAt) <= kMaxAgeMs;
  return {available, available ? latestCelsius : NAN, available ? "shtc3" : "unknown"};
}

String topic() {
  return homegear::mqtt::discovery().getStateTopic() + "/room_temperature";
}

void reset() {
  sampleAvailable = false;
  latestCelsius = 0.0f;
  sampledAt = publishedAt = attemptedAt = 0;
  hasPublished = publishedAvailable = failedAttempt = forced = false;
  publishedCelsius = 0.0f;
}

void update(float celsius, uint32_t now) {
  if (!validReading(Field::ThTemperature, celsius)) {
    invalidate();
    return;
  }
  latestCelsius = celsius;
  sampledAt = now;
  sampleAvailable = true;
}

void invalidate() {
  sampleAvailable = false;
}

void tick(uint32_t now, bool force) {
  forced = forced || force;
  const Reading current = reading(now);
  const bool availabilityChanged = !hasPublished || current.available != publishedAvailable;
  const bool changed = current.available &&
      std::fabs(current.celsius - publishedCelsius) >= kChangeThresholdCelsius;
  const bool due = forced || availabilityChanged ||
      elapsed(now, publishedAt, kHeartbeatIntervalMs) ||
      (changed && elapsed(now, publishedAt, kPublishIntervalMs));
  if (!due || (failedAttempt && !elapsed(now, attemptedAt, kPublishRetryMs))) return;
  ESP32MQTTClient &client = homegear::mqtt::client();
  if (!client.isConnected()) return;

  // MQTT Climate treats the exact payload "None" as an unknown temperature.
  // A retained unknown also clears the broker's last valid value after faults.
  const String payload = current.available ? String(current.celsius) : String("None");
  attemptedAt = now;
  if (!client.publish(topic(), payload, 0, true)) {
    failedAttempt = true;
    return;
  }
  hasPublished = true;
  publishedAvailable = current.available;
  publishedCelsius = current.celsius;
  publishedAt = now;
  failedAttempt = false;
  forced = false;
}

}  // namespace room
}  // namespace sensors
}  // namespace homegear
