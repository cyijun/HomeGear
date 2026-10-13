// Exercise the actual power policy against simulated clock/timer drivers.
#include "../Software/HomeGear/PowerPolicy.cpp"
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <iostream>
#include <vector>

MockSerial Serial;
MockEsp ESP;
namespace {
uint32_t cpuMhz = 240;
uint32_t rejectedClock = 0;
int64_t currentUs = 0;
std::vector<uint32_t> clockRequests;
uint32_t waitedMs = 0;
}
void MockSerial::println(const char *text) { output += std::string(text) + "\n"; }
void MockSerial::printf(const char *format, ...) {
  char buffer[1024];
  va_list arguments;
  va_start(arguments, format);
  std::vsnprintf(buffer, sizeof(buffer), format, arguments);
  va_end(arguments);
  output += buffer;
}
uint32_t millis() { return static_cast<uint32_t>(currentUs / 1000); }
void delay(uint32_t duration) { waitedMs += duration; currentUs += duration * 1000; }
int64_t esp_timer_get_time() { return currentUs; }
bool setCpuFrequencyMhz(uint32_t frequency) {
  clockRequests.push_back(frequency);
  if (frequency == rejectedClock) return false;
  cpuMhz = frequency;
  return true;
}
uint32_t getCpuFrequencyMhz() { return cpuMhz; }
uint32_t getApbFrequency() { return 80000000; }
esp_err_t esp_wifi_get_ps(wifi_ps_type_t *mode) { *mode = WIFI_PS_MIN_MODEM; return ESP_OK; }

int main() {
  using namespace homegear::power;
  begin();
  assert(cpuMhz == 160);
  printStatus();
  assert(Serial.output.find("POWER ir_clock_uses=0\n") != std::string::npos);
  // Startup only changes the CPU clock; IR clocks are acquired on a command.
  assert(clockRequests == std::vector<uint32_t>{160});

  beginLoop();
  currentUs += 37;
  endLoop();
  assert(waitedMs == 2);
  printStatus();
  assert(Serial.output.find("POWER loop_work_us=37\n") != std::string::npos);
  assert(Serial.output.find("POWER loop_wait_us=2000\n") != std::string::npos);

  {
    ScopedIrClock outer;
    assert(outer.ready() && cpuMhz == 240);
    {
      ScopedIrClock inner;
      assert(inner.ready() && cpuMhz == 240);
    }
    assert(cpuMhz == 240);  // A nested guard restores the enclosing clock.
  }
  assert(cpuMhz == 160);
  auto earlyReturn = []() { ScopedIrClock clock; return clock.ready(); };
  assert(earlyReturn() && cpuMhz == 160);

  rejectedClock = 240;
  {
    ScopedIrClock clock;
    assert(!clock.ready());  // Failed setup must not authorize a transmit.
  }
  assert(cpuMhz == 160);
  assert(Serial.output.find("POWER ERROR IR_CLOCK_UNAVAILABLE") != std::string::npos);
  rejectedClock = 0;
  {
    ScopedIrClock clock;
    assert(clock.ready());
    rejectedClock = 160;
  }
  assert(cpuMhz == 240);
  Serial.output.clear();
  printStatus();
  assert(Serial.output.find("POWER cpu_configured=false") != std::string::npos);
  rejectedClock = 0;
  begin();
  assert(cpuMhz == 160);
  std::cout << "power_policy_test: PASS\n";
}
