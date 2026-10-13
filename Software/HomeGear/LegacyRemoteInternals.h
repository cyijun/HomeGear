#pragma once
#include <Arduino.h>

namespace homegear {
namespace legacy {
namespace detail {
using Action = void (*)(const String &);
// Brand adapters only enqueue bounded payloads. LegacyRemotes::tick() is the
// sole action executor, sharing one queue and one scoped IR clock across brands.
void enqueue(const String &payload, Action action);
namespace singfun { void begin(); void onConnected(); }
namespace gree { void begin(); void onConnected(); }
namespace midea { void begin(); void onConnected(); }
namespace coolix { void begin(); void onConnected(); }
namespace haier { void begin(); void onConnected(); }
namespace philips { void begin(); void onConnected(); }
// RF transport exists whenever RF_MODULE is on, independently of Philips.
namespace rf {
void begin();
void send(const unsigned char data[6]);
void setBaud();
}
}  // namespace detail
}  // namespace legacy
}  // namespace homegear
