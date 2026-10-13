#pragma once

#include "HomeGearConfig.h"
#if IAQ_MODULE
// Reuse the pinned Bosch 1.8.1492 wrapper under a project-owned class name.
// Its public values are cleared on every frame; presence must be tracked too.
#define Bsec HomeGearTrackedBsec
#include <bsec.h>
#undef Bsec

namespace homegear {
namespace sensors {
namespace trackedBsec {
void resetOutputs();
bool hasOutput(uint8_t sensorId);
uint32_t outputMask();
}  // namespace trackedBsec
}  // namespace sensors
}  // namespace homegear
#endif
