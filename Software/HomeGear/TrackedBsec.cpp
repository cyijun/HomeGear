#include "TrackedBsec.h"

#if IAQ_MODULE
namespace homegear {
namespace sensors {
namespace trackedBsec {
namespace {
uint32_t returnedOutputs = 0;
}

void resetOutputs() { returnedOutputs = 0; }
bool hasOutput(uint8_t sensorId) {
  return sensorId < 32 && (returnedOutputs & (uint32_t(1) << sensorId));
}
uint32_t outputMask() { return returnedOutputs; }

void capture(const bsec_output_t *outputs, uint8_t count) {
  resetOutputs();
  for (uint8_t i = 0; i < count; ++i) {
    const uint8_t id = outputs[i].sensor_id;
    if (id < 32) returnedOutputs |= uint32_t(1) << id;
  }
}
}  // namespace trackedBsec
}  // namespace sensors
}  // namespace homegear

// Defined before the macro redirect so this calls the actual linked BSEC algo.
extern "C" bsec_library_return_t homegearTrackedDoSteps(
    const bsec_input_t *const inputs, const uint8_t nInputs,
    bsec_output_t *outputs, uint8_t *nOutputs) {
  homegear::sensors::trackedBsec::resetOutputs();
  const bsec_library_return_t status = bsec_do_steps(inputs, nInputs, outputs, nOutputs);
  if (status == BSEC_OK) homegear::sensors::trackedBsec::capture(outputs, *nOutputs);
  return status;
}

// Include the unchanged pinned BSD-3-Clause wrapper, including its copyright
// and license notice. Only the class and processing call are locally renamed;
// the vendor library on disk stays unchanged and the adaptation is reproducible.
#define Bsec HomeGearTrackedBsec
#define bsec_do_steps homegearTrackedDoSteps
#include <bsec.cpp>
#undef bsec_do_steps
#undef Bsec
#endif
