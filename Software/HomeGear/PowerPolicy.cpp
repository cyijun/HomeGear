#include "PowerPolicy.h"

#include <Arduino.h>
#include <esp_wifi.h>
#include <esp32-hal-cpu.h>
#include <esp_timer.h>
#include <sdkconfig.h>

#ifndef HOMEGEAR_CPU_MHZ
#define HOMEGEAR_CPU_MHZ 160
#endif

static_assert(HOMEGEAR_CPU_MHZ == 80 || HOMEGEAR_CPU_MHZ == 160 ||
                  HOMEGEAR_CPU_MHZ == 240,
              "HomeGear requires a Wi-Fi-compatible CPU clock");

namespace homegear {
namespace power {
namespace {
const uint32_t kIdleWaitMs = 2;
bool clockConfigured = false;
uint64_t loopCount = 0;
uint64_t workUs = 0;
uint64_t waitUs = 0;
uint32_t irClockUses = 0;
int64_t loopStartUs = 0;
}  // namespace

void begin() {
  clockConfigured = setCpuFrequencyMhz(HOMEGEAR_CPU_MHZ) &&
                    getCpuFrequencyMhz() == HOMEGEAR_CPU_MHZ;
  Serial.printf("POWER BOOT cpu_mhz=%u target_mhz=%u configured=%s\n",
                static_cast<unsigned>(getCpuFrequencyMhz()),
                static_cast<unsigned>(HOMEGEAR_CPU_MHZ),
                clockConfigured ? "true" : "false");
}

void beginLoop() { loopStartUs = esp_timer_get_time(); }

ScopedIrClock::ScopedIrClock()
    : previousMhz_(getCpuFrequencyMhz()), ready_(false) {
  ready_ = (previousMhz_ == 240 || setCpuFrequencyMhz(240)) &&
           getCpuFrequencyMhz() == 240;
  if (ready_) ++irClockUses;
  else Serial.println("POWER ERROR IR_CLOCK_UNAVAILABLE");
}

ScopedIrClock::~ScopedIrClock() {
  if (getCpuFrequencyMhz() != previousMhz_ &&
      (!setCpuFrequencyMhz(previousMhz_) || getCpuFrequencyMhz() != previousMhz_))
    Serial.println("POWER ERROR CPU_CLOCK_RESTORE_FAILED");
}

void endLoop() {
  const int64_t waitStartUs = esp_timer_get_time();
  workUs += static_cast<uint64_t>(waitStartUs - loopStartUs);
  ++loopCount;
  // Yield a time slice even while networking is down. ZigBee has its own
  // bounded task and UART buffers, and MQTT runs in the IDF client task.
  delay(kIdleWaitMs);
  waitUs += static_cast<uint64_t>(esp_timer_get_time() - waitStartUs);
}

void printStatus() {
  wifi_ps_type_t wifiSleep = WIFI_PS_NONE;
  const esp_err_t sleepResult = esp_wifi_get_ps(&wifiSleep);
  Serial.println("POWER BEGIN");
  Serial.printf("POWER cpu_mhz=%u\n", static_cast<unsigned>(getCpuFrequencyMhz()));
  Serial.printf("POWER target_mhz=%u\n", static_cast<unsigned>(HOMEGEAR_CPU_MHZ));
  Serial.printf("POWER cpu_configured=%s\n", clockConfigured && getCpuFrequencyMhz() == HOMEGEAR_CPU_MHZ ? "true" : "false");
  Serial.printf("POWER apb_mhz=%u\n", static_cast<unsigned>(getApbFrequency() / 1000000));
  Serial.printf("POWER idle_wait_ms=%lu\n", static_cast<unsigned long>(kIdleWaitMs));
  Serial.printf("POWER ir_clock_mhz=240\nPOWER ir_clock_uses=%lu\n", static_cast<unsigned long>(irClockUses));
  Serial.printf("POWER uptime_ms=%lu\n", static_cast<unsigned long>(millis()));
  Serial.printf("POWER loop_count=%llu\n", static_cast<unsigned long long>(loopCount));
  Serial.printf("POWER loop_work_us=%llu\n", static_cast<unsigned long long>(workUs));
  Serial.printf("POWER loop_wait_us=%llu\n", static_cast<unsigned long long>(waitUs));
  const uint64_t measuredUs = workUs + waitUs;
  Serial.printf("POWER loop_work_percent=%.2f\n",
                measuredUs ? 100.0 * static_cast<double>(workUs) / measuredUs : 0.0);
  // This percentage describes elapsed main-loop work versus waiting, not
  // utilization of both CPU cores or a measurement of electrical power.
  Serial.printf("POWER wifi_ps_mode=%d\n", sleepResult == ESP_OK ? static_cast<int>(wifiSleep) : -1);
  Serial.println("POWER automatic_light_sleep=false");
  Serial.printf("POWER free_heap=%u\n", static_cast<unsigned>(ESP.getFreeHeap()));
  Serial.printf("POWER min_free_heap=%u\n", static_cast<unsigned>(ESP.getMinFreeHeap()));
  Serial.println("POWER END");
}

}  // namespace power
}  // namespace homegear
