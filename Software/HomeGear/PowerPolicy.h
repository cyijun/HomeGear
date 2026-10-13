#pragma once

#include <cstdint>

namespace homegear {
namespace power {

// Keep the CPU at a Wi-Fi-compatible frequency; this is task waiting, not
// automatic light sleep. The pinned Arduino core is built without ESP PM.
void begin();
void beginLoop();
void endLoop();
void printStatus();

// The locked IR library's software-carrier offset was measured at 240 MHz.
// Use that clock only during an explicit transmit, then restore the idle clock.
class ScopedIrClock {
 public:
  ScopedIrClock();
  ~ScopedIrClock();
  bool ready() const { return ready_; }
  ScopedIrClock(const ScopedIrClock &) = delete;
  ScopedIrClock &operator=(const ScopedIrClock &) = delete;
 private:
  uint32_t previousMhz_;
  bool ready_;
};

}  // namespace power
}  // namespace homegear
