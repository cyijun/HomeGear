#pragma once

namespace homegear {
namespace legacy {
// One coordinator owns command dispatch. MQTT callbacks never execute actions.
void begin();
void onConnected();
void tick();
void setRfBaud();
}  // namespace legacy
}  // namespace homegear
