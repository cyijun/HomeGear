#include "SensorService.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <atomic>
#include <cmath>
#include "ESP32MQTTClient.h"
#include "HAMDHandler.h"
#include "HomeGearConfig.h"
#include "MqttService.h"
#include "RoomTemperature.h"
#include "SensorSnapshot.h"

#if THI_MODULE
#include "SparkFun_SHTC3.h"
#include <BH1750.h>
#endif
#if IAQ_MODULE
#include "TrackedBsec.h"
#endif

namespace homegear {
namespace sensors {
namespace {

constexpr uint32_t kSampleIntervalMs = 2000;
constexpr uint32_t kGasWarmupMs = 360000;
struct Metadata {
  const char *key;
  const char *deviceClass;
  const char *unit;
  bool enabled;
};
// Preserve existing HA identifiers, units and numeric conventions.
const Metadata kMetadata[kFieldCount] = {
  {"thim_temperature", "temperature", "°C", THI_MODULE},
  {"thim_humidity", "humidity", "%", THI_MODULE},
  {"thim_illuminance", "illuminance", "lx", THI_MODULE},
  {"iaqm_temperature", "temperature", "°C", IAQ_MODULE},
  {"iaqm_humidity", "humidity", "%", IAQ_MODULE},
  {"iaqm_pressure", "pressure", "Pa", IAQ_MODULE},
  {"iaqm_iaq", "aqi", "", IAQ_MODULE},
  {"iaqm_co2Equivalent", "carbon_dioxide", "ppm", IAQ_MODULE},
  {"iaqm_breathVocEquivalent", "volatile_organic_compounds", "µg/m³", IAQ_MODULE}
};

SnapshotPublisher publisher;
std::atomic<bool> publishRequested{false};
uint32_t startedAt = 0;
uint32_t publishAttempts = 0;
uint32_t publishSuccesses = 0;
uint32_t publishFailures = 0;

#if THI_MODULE
SHTC3 shtc3;
bool shtReady = false;
uint32_t lastShtSample = 0;
uint32_t shtSamples = 0;
uint32_t shtErrors = 0;
SHTC3_Status_TypeDef lastShtStatus = SHTC3_Status_Nominal;

struct LightChannel {
  explicit LightChannel(uint8_t sensorAddress) : sensor(sensorAddress), address(sensorAddress) {}
  BH1750 sensor;
  uint8_t address;
  bool ready = false;
  bool known = false;
  float lux = 0.0f;
  uint32_t lastSample = 0;
  uint32_t samples = 0;
  uint32_t errors = 0;
};
LightChannel light1(0x23);
LightChannel light2(0x5C);

void sampleSht(uint32_t now) {
  if (!elapsed(now, lastShtSample, kSampleIntervalMs)) return;
  lastShtSample = now;
  if (!shtReady) {
    lastShtStatus = shtc3.begin();
    shtReady = lastShtStatus == SHTC3_Status_Nominal;
    shtc3.setMode(SHTC3_CMD_CSE_RHF_NPM);
  }
  if (shtReady) {
    lastShtStatus = shtc3.update();
    const float temperature = shtc3.toDegC();
    const float humidity = shtc3.toPercent();
    // This library can return Nominal even if one CRC failed.
    if (lastShtStatus == SHTC3_Status_Nominal && shtc3.passTcrc && shtc3.passRHcrc &&
        validReading(Field::ThTemperature, temperature) && validReading(Field::ThHumidity, humidity)) {
      publisher.update(Field::ThTemperature, temperature);
      publisher.update(Field::ThHumidity, humidity);
      room::update(temperature, now);
      ++shtSamples;
      return;
    }
    shtReady = false;
  }
  ++shtErrors;
  room::invalidate();
  // abortUpdate() does not call endProcess(); keep failed reads asleep too.
  shtc3.sleep(true);
}

void sampleLight(LightChannel &channel, uint32_t now) {
  if (!elapsed(now, channel.lastSample, kSampleIntervalMs)) return;
  channel.lastSample = now;
  if (!channel.ready) {
    channel.ready = channel.sensor.begin(BH1750::ONE_TIME_HIGH_RES_MODE, channel.address, &Wire);
    if (!channel.ready) {
      channel.known = false;
      ++channel.errors;
    }
    return;  // New one-shot measurement will be read on the next independent tick.
  }
  if (!channel.sensor.measurementReady(true)) return;
  const float lux = channel.sensor.readLightLevel();
  channel.known = validReading(Field::ThIlluminance, lux);
  if (channel.known) {
    channel.lux = lux;
    ++channel.samples;
  } else {
    ++channel.errors;
  }
  channel.ready = channel.sensor.configure(BH1750::ONE_TIME_HIGH_RES_MODE);
  if (!channel.ready) ++channel.errors;
}

void updateIlluminance() {
  if (!light1.known && !light2.known) return;
  const float lux = light1.known && light2.known ? std::fmax(light1.lux, light2.lux) :
                    (light1.known ? light1.lux : light2.lux);
  const Snapshot &snapshot = publisher.snapshot();
  const size_t temperatureIndex = static_cast<size_t>(Field::ThTemperature);
  const float temperature = snapshot.known[temperatureIndex] ? snapshot.values[temperatureIndex] : 20.0f;
  const float factor = 1.0f - (std::round(temperature) - 20.0f) * 0.0005f;
  publisher.update(Field::ThIlluminance, lux * factor);
}
#endif

#if IAQ_MODULE
const uint8_t kBsecConfigIaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};
static_assert(sizeof(kBsecConfigIaq) == BSEC_MAX_PROPERTY_BLOB_SIZE, "BSEC config version/size mismatch");
HomeGearTrackedBsec bme680;
bool iaqReady = false;
bool gasWarmedUp = false;
uint32_t iaqSamples = 0;
struct IaqDiagnostics {
  bool accuracyKnown = false;
  bool co2AccuracyKnown = false;
  bool vocAccuracyKnown = false;
  bool stabilizationKnown = false;
  bool runInKnown = false;
  uint8_t accuracy = 0, co2Accuracy = 0, vocAccuracy = 0;
  float stabilization = 0.0f, runIn = 0.0f;
} iaqDiagnostics;

void sampleIaq(uint32_t now) {
  // A scheduled run can return no data, and each successful partial frame
  // zeroes all wrapper members. Never infer presence from their numeric values.
  trackedBsec::resetOutputs();
  if (!iaqReady || !bme680.run()) return;
  ++iaqSamples;
  if (trackedBsec::hasOutput(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE))
    publisher.update(Field::IaqTemperature, bme680.temperature);
  if (trackedBsec::hasOutput(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY))
    publisher.update(Field::IaqHumidity, bme680.humidity);
  if (trackedBsec::hasOutput(BSEC_OUTPUT_RAW_PRESSURE))
    publisher.update(Field::IaqPressure, bme680.pressure);
  if (trackedBsec::hasOutput(BSEC_OUTPUT_STABILIZATION_STATUS) && std::isfinite(bme680.stabStatus)) {
    iaqDiagnostics.stabilizationKnown = true;
    iaqDiagnostics.stabilization = bme680.stabStatus;
  }
  if (trackedBsec::hasOutput(BSEC_OUTPUT_RUN_IN_STATUS) && std::isfinite(bme680.runInStatus)) {
    iaqDiagnostics.runInKnown = true;
    iaqDiagnostics.runIn = bme680.runInStatus;
  }
  if (trackedBsec::hasOutput(BSEC_OUTPUT_STATIC_IAQ) && validReading(Field::IaqIndex, bme680.staticIaq)) {
    iaqDiagnostics.accuracyKnown = true;
    iaqDiagnostics.accuracy = bme680.staticIaqAccuracy;
  }
  if (trackedBsec::hasOutput(BSEC_OUTPUT_CO2_EQUIVALENT) && validReading(Field::IaqCo2, bme680.co2Equivalent)) {
    iaqDiagnostics.co2AccuracyKnown = true;
    iaqDiagnostics.co2Accuracy = bme680.co2Accuracy;
  }
  if (trackedBsec::hasOutput(BSEC_OUTPUT_BREATH_VOC_EQUIVALENT) && validReading(Field::IaqVoc, bme680.breathVocEquivalent)) {
    iaqDiagnostics.vocAccuracyKnown = true;
    iaqDiagnostics.vocAccuracy = bme680.breathVocAccuracy;
  }
  // Retain the deployed six-minute warmup, and expose accuracy separately.
  gasWarmedUp = elapsed(now, startedAt, kGasWarmupMs);
  if (gasWarmedUp) {
    if (trackedBsec::hasOutput(BSEC_OUTPUT_STATIC_IAQ))
      publisher.update(Field::IaqIndex, bme680.staticIaq);
    if (trackedBsec::hasOutput(BSEC_OUTPUT_CO2_EQUIVALENT))
      publisher.update(Field::IaqCo2, bme680.co2Equivalent);
    if (trackedBsec::hasOutput(BSEC_OUTPUT_BREATH_VOC_EQUIVALENT))
      publisher.update(Field::IaqVoc, bme680.breathVocEquivalent * 1000.0f);
  }
}
#endif

void publishSnapshot(uint32_t now) {
  if (!publisher.shouldPublish(now)) return;
  ESP32MQTTClient &client = homegear::mqtt::client();
  if (!client.isConnected()) return;
  const Snapshot sent = publisher.snapshot();
  StaticJsonDocument<1024> document;
  const size_t count = forEachKnown(sent, [&](Field field, float value) {
    document[kMetadata[static_cast<size_t>(field)].key] = String(value);
  });
  if (!count) return;
#if IAQ_MODULE
  if (iaqSamples) {
    if (iaqDiagnostics.accuracyKnown) document["iaqm_accuracy"] = iaqDiagnostics.accuracy;
    if (iaqDiagnostics.stabilizationKnown) document["iaqm_stabilizationStatus"] = iaqDiagnostics.stabilization;
    if (iaqDiagnostics.runInKnown) document["iaqm_runInStatus"] = iaqDiagnostics.runIn;
    document["iaqm_warmingUp"] = !gasWarmedUp;
  }
#endif
  String payload;
  if (document.overflowed() || !serializeJson(document, payload)) {
    publisher.failed(now);
    ++publishFailures;
    return;
  }
  ++publishAttempts;
  if (client.publish(homegear::mqtt::discovery().getStateTopic(), payload, 0, true)) {
    publisher.committed(now, sent);
    ++publishSuccesses;
  } else {
    publisher.failed(now);
    ++publishFailures;
  }
}

}  // namespace

void begin() {
  startedAt = millis();
  room::reset();
  publisher = SnapshotPublisher();
  publisher.request();
  publishRequested.store(false, std::memory_order_relaxed);
  publishAttempts = publishSuccesses = publishFailures = 0;
#if THI_MODULE || IAQ_MODULE
  Wire.begin();
#endif
#if THI_MODULE
  shtSamples = shtErrors = 0;
  lastShtSample = startedAt - kSampleIntervalMs;
  lastShtStatus = shtc3.begin();
  shtReady = lastShtStatus == SHTC3_Status_Nominal;
  shtc3.setMode(SHTC3_CMD_CSE_RHF_NPM);  // Preserve normal-mode temperature precision.
  light1.ready = light1.sensor.begin(BH1750::ONE_TIME_HIGH_RES_MODE, light1.address, &Wire);
  light2.ready = light2.sensor.begin(BH1750::ONE_TIME_HIGH_RES_MODE, light2.address, &Wire);
  light1.known = light2.known = false;
  light1.samples = light2.samples = light1.errors = light2.errors = 0;
  light1.lastSample = light2.lastSample = startedAt;
#endif
#if IAQ_MODULE
  iaqReady = false;
  iaqSamples = 0;
  iaqDiagnostics = IaqDiagnostics();
  trackedBsec::resetOutputs();
  bme680.begin(BME68X_I2C_ADDR_LOW, Wire);
  if (bme680.bsecStatus >= BSEC_OK && bme680.bme68xStatus >= BME68X_OK) {
    bme680.setConfig(kBsecConfigIaq);
    if (bme680.bsecStatus >= BSEC_OK) {
      bsec_virtual_sensor_t outputs[] = {
        BSEC_OUTPUT_RAW_PRESSURE, BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT, BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_STABILIZATION_STATUS, BSEC_OUTPUT_RUN_IN_STATUS
      };
      bme680.updateSubscription(outputs, sizeof(outputs) / sizeof(outputs[0]), BSEC_SAMPLE_RATE_LP);
      iaqReady = bme680.bsecStatus >= BSEC_OK;
    }
  }
  gasWarmedUp = false;
#endif
}

void requestPublish() {
  publishRequested.store(true, std::memory_order_release);
}

void tick(uint32_t now) {
  const bool force = publishRequested.exchange(false, std::memory_order_acq_rel);
  if (force) publisher.request();
#if THI_MODULE
  sampleSht(now);
  sampleLight(light1, now);
  sampleLight(light2, now);
  updateIlluminance();
#endif
#if IAQ_MODULE
  sampleIaq(now);
#endif
  publishSnapshot(now);
  room::tick(now, force);
}

void publishDiscovery(ESP32MQTTClient &client, HAMqttDiscoveryHandler &device) {
  for (size_t i = 0; i < kFieldCount; ++i) {
    if (!kMetadata[i].enabled) continue;
    HAMqttDiscoverySensor sensor(device);
    sensor.setDeviceNameMin(kMetadata[i].key);
    sensor.setDeviceClass(kMetadata[i].deviceClass);
    sensor.setStateClass("measurement");
    sensor.setUnitOfMeasurement(kMetadata[i].unit);
    sensor.setValueTemplate(String("{{ value_json.") + kMetadata[i].key + " }}");
    sensor.construct();
    client.publish(sensor.getMqttDiscoveryConfigTopic(), sensor.getMqttDiscoveryMesg(), 0, true);
  }
}

void printStatus() {
  Serial.println("SENSORS BEGIN");
  const room::Reading currentRoom = room::reading(millis());
  Serial.printf("SENSORS room_temperature_source=%s\nSENSORS room_temperature_max_age_ms=%lu\n",
      currentRoom.source, static_cast<unsigned long>(room::kMaxAgeMs));
  if (currentRoom.available) Serial.printf("SENSORS room_temperature=%.2f\n", currentRoom.celsius);
  else Serial.println("SENSORS room_temperature=unknown");
  Serial.printf("SENSORS publish_interval_ms=%lu\nSENSORS heartbeat_ms=%lu\nSENSORS publish_attempts=%lu\nSENSORS publish_count=%lu\nSENSORS publish_failures=%lu\nSENSORS dirty=%u\n",
      static_cast<unsigned long>(kPublishIntervalMs), static_cast<unsigned long>(kHeartbeatIntervalMs),
      static_cast<unsigned long>(publishAttempts), static_cast<unsigned long>(publishSuccesses),
      static_cast<unsigned long>(publishFailures), publisher.dirty());
  const Snapshot &snapshot = publisher.snapshot();
  const char *statusKeys[kFieldCount] = {"thi_temperature", "thi_humidity", "thi_illuminance",
      "iaq_temperature", "iaq_humidity", "iaq_pressure", "iaq_index", "iaq_co2_equivalent", "iaq_voc_equivalent"};
  for (size_t i = 0; i < kFieldCount; ++i) {
    if (!kMetadata[i].enabled) continue;
    if (snapshot.known[i]) Serial.printf("SENSORS %s=%.2f\n", statusKeys[i], snapshot.values[i]);
    else Serial.printf("SENSORS %s=unknown\n", statusKeys[i]);
  }
#if THI_MODULE
  Serial.printf("SENSORS shtc3_valid=%u\nSENSORS shtc3_mode=normal_sleep\nSENSORS shtc3_sample_ms=2000\nSENSORS shtc3_samples=%lu\nSENSORS shtc3_errors=%lu\nSENSORS shtc3_status=%d\n",
      shtReady && snapshot.known[static_cast<size_t>(Field::ThTemperature)] && snapshot.known[static_cast<size_t>(Field::ThHumidity)],
      static_cast<unsigned long>(shtSamples), static_cast<unsigned long>(shtErrors), lastShtStatus);
  Serial.printf("SENSORS bh1750_mode=one_shot\nSENSORS bh1750_1_samples=%lu\nSENSORS bh1750_2_samples=%lu\nSENSORS bh1750_1_errors=%lu\nSENSORS bh1750_2_errors=%lu\n",
      static_cast<unsigned long>(light1.samples), static_cast<unsigned long>(light2.samples),
      static_cast<unsigned long>(light1.errors), static_cast<unsigned long>(light2.errors));
#endif
#if IAQ_MODULE
  Serial.printf("SENSORS bme680_valid=%u\nSENSORS bsec_config=33v_3s_4d\nSENSORS bsec_mode=LP\nSENSORS temperature_offset=0\nSENSORS iaq_warming_up=%u\nSENSORS bme680_samples=%lu\nSENSORS bsec_status=%d\nSENSORS bme680_status=%d\n",
      iaqReady && bme680.bsecStatus >= BSEC_OK && bme680.bme68xStatus >= BME68X_OK &&
          snapshot.known[static_cast<size_t>(Field::IaqTemperature)] && snapshot.known[static_cast<size_t>(Field::IaqHumidity)] &&
          snapshot.known[static_cast<size_t>(Field::IaqPressure)],
      !gasWarmedUp, static_cast<unsigned long>(iaqSamples),
      bme680.bsecStatus, bme680.bme68xStatus);
  if (iaqDiagnostics.accuracyKnown) Serial.printf("SENSORS iaq_accuracy=%u\n", iaqDiagnostics.accuracy);
  else Serial.println("SENSORS iaq_accuracy=unknown");
  if (iaqDiagnostics.co2AccuracyKnown) Serial.printf("SENSORS co2_accuracy=%u\n", iaqDiagnostics.co2Accuracy);
  else Serial.println("SENSORS co2_accuracy=unknown");
  if (iaqDiagnostics.vocAccuracyKnown) Serial.printf("SENSORS voc_accuracy=%u\n", iaqDiagnostics.vocAccuracy);
  else Serial.println("SENSORS voc_accuracy=unknown");
  if (iaqDiagnostics.stabilizationKnown) Serial.printf("SENSORS stabilization=%.0f\n", iaqDiagnostics.stabilization);
  else Serial.println("SENSORS stabilization=unknown");
  if (iaqDiagnostics.runInKnown) Serial.printf("SENSORS run_in=%.0f\n", iaqDiagnostics.runIn);
  else Serial.println("SENSORS run_in=unknown");
#endif
  Serial.println("SENSORS END");
}

}  // namespace sensors
}  // namespace homegear
