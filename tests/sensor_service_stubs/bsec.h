#pragma once
#include "Wire.h"
#include <cstring>
#define BSEC_MAX_PROPERTY_BLOB_SIZE 462
#define BSEC_OK 0
#define BME68X_OK 0
#define BME68X_I2C_ADDR_LOW 0x76
#define BSEC_SAMPLE_RATE_LP 0.33333f
enum bsec_virtual_sensor_t {
  BSEC_OUTPUT_STATIC_IAQ = 2, BSEC_OUTPUT_CO2_EQUIVALENT = 3,
  BSEC_OUTPUT_BREATH_VOC_EQUIVALENT = 4, BSEC_OUTPUT_RAW_PRESSURE = 7,
  BSEC_OUTPUT_STABILIZATION_STATUS = 12, BSEC_OUTPUT_RUN_IN_STATUS = 13,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE = 14,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY = 15
};
using bsec_library_return_t = int;
struct bsec_input_t { float signal; uint8_t sensor_id; };
struct bsec_output_t { float signal; uint8_t sensor_id; uint8_t accuracy; };
extern "C" bsec_library_return_t bsec_do_steps(const bsec_input_t *const, const uint8_t, bsec_output_t *, uint8_t *);
extern "C" bsec_library_return_t homegearTrackedDoSteps(const bsec_input_t *const, const uint8_t, bsec_output_t *, uint8_t *);
class Bsec {
 public:
  int bsecStatus = 0, bme68xStatus = 0;
  float temperature = 0, humidity = 0, pressure = 0;
  float staticIaq = 0, co2Equivalent = 0, breathVocEquivalent = 0;
  float stabStatus = 0, runInStatus = 0;
  uint8_t staticIaqAccuracy = 0, co2Accuracy = 0, breathVocAccuracy = 0;
  uint8_t config[462] = {};
  void begin(uint8_t, TwoWire &) {
    last_ = sensor_mock::now - 3000;
    bsecStatus = bme68xStatus = 0;
    temperature = humidity = pressure = staticIaq = co2Equivalent = breathVocEquivalent = 0.0f;
    stabStatus = runInStatus = 0.0f;
    staticIaqAccuracy = co2Accuracy = breathVocAccuracy = 0;
  }
  void setConfig(const uint8_t *blob) { std::memcpy(config, blob, sizeof(config)); ++sensor_mock::bsecConfigurations; }
  void updateSubscription(bsec_virtual_sensor_t *, uint8_t, float rate) { requestedRate = rate; }
  float requestedRate = 0;
  bool run() {
    if (!sensor_mock::bsecHealthy || uint32_t(sensor_mock::now - last_) < 3000) return false;
    last_ = sensor_mock::now;
    bsec_output_t outputs[8];
    uint8_t count = 8;
    bsecStatus = homegearTrackedDoSteps(nullptr, 0, outputs, &count);
    if (bsecStatus != BSEC_OK) return false;
    // Faithfully mimic the pinned wrapper's per-frame zeroOutputs(), including
    // fields which this partial frame did not produce.
    temperature = humidity = pressure = staticIaq = co2Equivalent = breathVocEquivalent = 0.0f;
    stabStatus = runInStatus = 0.0f;
    staticIaqAccuracy = co2Accuracy = breathVocAccuracy = 0;
    for (uint8_t i = 0; i < count; ++i) {
      const bsec_output_t &output = outputs[i];
      switch (output.sensor_id) {
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE: temperature = output.signal; break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY: humidity = output.signal; break;
        case BSEC_OUTPUT_RAW_PRESSURE: pressure = output.signal; break;
        case BSEC_OUTPUT_STATIC_IAQ: staticIaq = output.signal; staticIaqAccuracy = output.accuracy; break;
        case BSEC_OUTPUT_CO2_EQUIVALENT: co2Equivalent = output.signal; co2Accuracy = output.accuracy; break;
        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT: breathVocEquivalent = output.signal; breathVocAccuracy = output.accuracy; break;
        case BSEC_OUTPUT_STABILIZATION_STATUS: stabStatus = output.signal; break;
        case BSEC_OUTPUT_RUN_IN_STATUS: runInStatus = output.signal; break;
      }
    }
    return count > 0;
  }
 private:
  uint32_t last_ = 0;
};
