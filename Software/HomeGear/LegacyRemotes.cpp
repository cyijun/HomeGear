#include "LegacyRemotes.h"
#include "LegacyRemoteInternals.h"
#include "HomeGearConfig.h"
#include "PowerPolicy.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define HOMEGEAR_LEGACY_QUEUE_ENABLED \
((IR_MODULE && (IR_SINGFUN_FAN || IR_GREE_AC || IR_MIDEA_AC || IR_COOLIX_AC || IR_HAIER_AC)) || \
(RF_MODULE && RF_PHILIPS_FAN_LIGHT))

namespace homegear {
namespace legacy {
namespace {
#if HOMEGEAR_LEGACY_QUEUE_ENABLED
struct Command { detail::Action action; char payload[513]; };
QueueHandle_t commandQueue = nullptr;
#endif
}  // namespace

namespace detail {
#if HOMEGEAR_LEGACY_QUEUE_ENABLED
void enqueue(const String &payload, Action action) {
  if (!commandQueue || payload.length() >= sizeof(Command::payload)) {
    Serial.println("LEGACY ERROR COMMAND_REJECTED");
    return;
  }
  Command command = {};
  command.action = action;
  payload.toCharArray(command.payload, sizeof(command.payload));
  if (xQueueSend(commandQueue, &command, 0) != pdTRUE)
  Serial.println("LEGACY ERROR QUEUE_FULL");
}
#endif
}  // namespace detail

void begin() {
#if HOMEGEAR_LEGACY_QUEUE_ENABLED
  commandQueue = xQueueCreate(4, sizeof(Command));
  if (!commandQueue) Serial.println("LEGACY ERROR QUEUE_ALLOCATION_FAILED");
#endif
#if (IR_MODULE && IR_SINGFUN_FAN)
  detail::singfun::begin();
#endif
#if (IR_MODULE && IR_GREE_AC)
  detail::gree::begin();
#endif
#if (IR_MODULE && IR_MIDEA_AC)
  detail::midea::begin();
#endif
#if (IR_MODULE && IR_COOLIX_AC)
  detail::coolix::begin();
#endif
#if (IR_MODULE && IR_HAIER_AC)
  detail::haier::begin();
#endif
#if RF_MODULE
  detail::rf::begin();
#endif
#if (RF_MODULE && RF_PHILIPS_FAN_LIGHT)
  detail::philips::begin();
#endif
}

void onConnected() {
#if (IR_MODULE && IR_SINGFUN_FAN)
  detail::singfun::onConnected();
#endif
#if (IR_MODULE && IR_GREE_AC)
  detail::gree::onConnected();
#endif
#if (IR_MODULE && IR_MIDEA_AC)
  detail::midea::onConnected();
#endif
#if (IR_MODULE && IR_COOLIX_AC)
  detail::coolix::onConnected();
#endif
#if (IR_MODULE && IR_HAIER_AC)
  detail::haier::onConnected();
#endif
#if (RF_MODULE && RF_PHILIPS_FAN_LIGHT)
  detail::philips::onConnected();
#endif
}

void tick() {
#if HOMEGEAR_LEGACY_QUEUE_ENABLED
  // Main loop alone mutates remote state, transmits and publishes feedback.
  Command command;
  if (commandQueue && xQueueReceive(commandQueue, &command, 0) == pdTRUE) {
    power::ScopedIrClock clock;
    if (!clock.ready()) { Serial.println("LEGACY ERROR IR_CLOCK_UNAVAILABLE"); return; }
    command.action(String(command.payload));
  }
#endif
}
void setRfBaud() {
#if RF_MODULE
  detail::rf::setBaud();
#endif
}
}  // namespace legacy
}  // namespace homegear
