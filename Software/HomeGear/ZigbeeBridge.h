#ifndef HOMEGEAR_ZIGBEE_BRIDGE_H
#define HOMEGEAR_ZIGBEE_BRIDGE_H

#include <cstdint>

namespace homegear {
namespace zigbee {

struct Stats {
  bool serialReady = false;
  bool taskRunning = false;
  bool networkListening = false;
  bool clientConnected = false;
  std::uint32_t connections = 0;
  std::uint32_t disconnects = 0;
  std::uint32_t socketErrors = 0;
  std::uint32_t uartErrors = 0;
  std::uint32_t networkBackpressure = 0;
  std::uint32_t uartBackpressure = 0;
  std::uint32_t toSerialPending = 0;
  std::uint32_t toNetworkPending = 0;
  std::uint32_t toSerialHighWater = 0;
  std::uint32_t toNetworkHighWater = 0;
  std::uint64_t networkReceived = 0;
  std::uint64_t serialTransmitted = 0;
  std::uint64_t serialReceived = 0;
  std::uint64_t networkTransmitted = 0;
  std::uint64_t serialDiscardedNoClient = 0;
  std::uint64_t bufferedDiscardedDisconnect = 0;
};

// Call once during setup, in this order. UART starts before Wi-Fi; sockets and
// the independent bridge task start only after Wi-Fi/lwIP initialization.
bool beginSerial();
bool beginNetwork();
Stats getStats();
void printStatus();

}  // namespace zigbee
}  // namespace homegear

#endif
