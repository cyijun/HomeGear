#include "HomeGearConfig.h"
#include "ZigbeeBridge.h"

#if ZGB_MODULE

#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/sockets.h>
#include <cerrno>

#include "ZigbeeBridgeBuffer.h"

namespace homegear {
namespace zigbee {
namespace {

constexpr std::uint16_t kListenPort = 1234;
constexpr std::uint32_t kBaudRate = 115200;
constexpr std::size_t kRxBufferSize = 4096;
constexpr std::size_t kTxBufferSize = 1024;
constexpr std::size_t kQueueSize = 4096;
constexpr std::size_t kIoBudget = 256;
constexpr std::size_t kUartWriteBudget = 128;
// Leave room for IDF ring-buffer item headers/wrap. The bridge task is the only
// UART2 writer, so its free space cannot shrink between this check and write.
constexpr std::size_t kTxReserve = 256;
#if CONFIG_FREERTOS_UNICORE
constexpr BaseType_t kBridgeCore = tskNO_AFFINITY;
#else
// Keep the priority-2 bridge off the Arduino/IR carrier core. Wi-Fi's higher
// priority tasks still preempt the bridge on the other core when needed.
constexpr BaseType_t kBridgeCore = ARDUINO_RUNNING_CORE == 0 ? 1 : 0;
#endif

HardwareSerial serial(2);
WiFiServer server(kListenPort, 1);
WiFiClient client;
ByteRing<kQueueSize> toSerial;
ByteRing<kQueueSize> toNetwork;
TaskHandle_t taskHandle = nullptr;
portMUX_TYPE statsMux = portMUX_INITIALIZER_UNLOCKED;
Stats publishedStats;

std::size_t minimum(std::size_t a, std::size_t b) {
  return a < b ? a : b;
}

bool temporarySocketError(int error) {
  return error == EAGAIN || error == EWOULDBLOCK || error == EINTR;
}

void publishStats(Stats &stats) {
  stats.toSerialPending = static_cast<std::uint32_t>(toSerial.size());
  stats.toNetworkPending = static_cast<std::uint32_t>(toNetwork.size());
  if (stats.toSerialPending > stats.toSerialHighWater)
    stats.toSerialHighWater = stats.toSerialPending;
  if (stats.toNetworkPending > stats.toNetworkHighWater)
    stats.toNetworkHighWater = stats.toNetworkPending;
  portENTER_CRITICAL(&statsMux);
  publishedStats = stats;
  portEXIT_CRITICAL(&statsMux);
}

void closeClient(Stats &stats) {
  if (stats.clientConnected) ++stats.disconnects;
  stats.bufferedDiscardedDisconnect += toSerial.size() + toNetwork.size();
  toSerial.clear();
  toNetwork.clear();
  client.stop();
  stats.clientConnected = false;
}

// No consumer exists when the TCP client is absent. Match the old bridge's
// discard policy, but bound each pass instead of draining a busy UART forever.
bool discardSerialWithoutClient(Stats &stats) {
  std::uint8_t discarded[kIoBudget];
  const int available = serial.available();
  if (available <= 0) return false;
  const std::size_t count = serial.read(
      discarded, minimum(kIoBudget, static_cast<std::size_t>(available)));
  stats.serialDiscardedNoClient += count;
  return count != 0;
}

bool transferToSerial(Stats &stats) {
  if (toSerial.size() == 0) return false;
  std::size_t freeSpace = 0;
  if (uart_get_tx_buffer_free_size(UART_NUM_2, &freeSpace) != ESP_OK) {
    ++stats.uartErrors;
    return false;
  }
  if (freeSpace <= kTxReserve) {
    ++stats.uartBackpressure;
    return false;
  }
  const std::ptrdiff_t sent = toSerial.writeTo(
      minimum(kUartWriteBudget, freeSpace - kTxReserve),
      [](const std::uint8_t *data, std::size_t count) -> std::ptrdiff_t {
        return uart_write_bytes(UART_NUM_2, data, count);
      });
  if (sent < 0) ++stats.uartErrors;
  if (sent > 0) stats.serialTransmitted += static_cast<std::uint64_t>(sent);
  return sent > 0;
}

bool readSerial(Stats &stats) {
  const int available = serial.available();
  if (available <= 0 || toNetwork.free() == 0) return false;
  const std::ptrdiff_t received = toNetwork.readFrom(
      minimum(kIoBudget, static_cast<std::size_t>(available)),
      [](std::uint8_t *data, std::size_t count) -> std::ptrdiff_t {
        // HardwareSerial::read(buffer, count) uses a zero-tick UART read here.
        return serial.read(data, count);
      });
  if (received > 0)
    stats.serialReceived += static_cast<std::uint64_t>(received);
  return received > 0;
}

bool transferToNetwork(Stats &stats, bool &closed) {
  if (toNetwork.size() == 0) return false;
  int socketError = 0;
  const std::ptrdiff_t sent = toNetwork.writeTo(
      kIoBudget,
      [&](const std::uint8_t *data, std::size_t count) -> std::ptrdiff_t {
        // WiFiClient::write retries inside select; a single nonblocking send
        // keeps this task responsive when the network applies backpressure.
        const int result = ::send(client.fd(), data, count, MSG_DONTWAIT);
        if (result < 0) socketError = errno;
        return result;
      });
  if (sent > 0) stats.networkTransmitted += static_cast<std::uint64_t>(sent);
  if (sent < 0 && temporarySocketError(socketError)) {
    ++stats.networkBackpressure;
  } else if (sent <= 0) {
    ++stats.socketErrors;
    closed = true;
  }
  return sent > 0;
}

bool readNetwork(Stats &stats, bool &closed) {
  if (toSerial.free() == 0) {
    ++stats.uartBackpressure;
    return false;  // Leave bytes in TCP so its receive window applies backpressure.
  }
  int socketError = 0;
  const std::ptrdiff_t received = toSerial.readFrom(
      kIoBudget,
      [&](std::uint8_t *data, std::size_t count) -> std::ptrdiff_t {
        const int result = ::recv(client.fd(), data, count, MSG_DONTWAIT);
        if (result < 0) socketError = errno;
        return result;
      });
  if (received > 0)
    stats.networkReceived += static_cast<std::uint64_t>(received);
  else if (received == 0)
    closed = true;  // TCP peer closed, rather than a temporarily empty socket.
  else if (!temporarySocketError(socketError)) {
    ++stats.socketErrors;
    closed = true;
  }
  return received > 0;
}

void bridgeTask(void *) {
  Stats stats = getStats();
  stats.taskRunning = true;
  for (;;) {
    bool active = false;
    if (WiFi.status() != WL_CONNECTED) {
      if (stats.clientConnected) closeClient(stats);
      active = discardSerialWithoutClient(stats);
    } else if (!stats.clientConnected) {
      active = discardSerialWithoutClient(stats);
      // Do not hand stale bytes from before a connection to a new HA client.
      // With sustained no-client traffic, drain it over bounded passes first.
      if (serial.available() == 0) {
        client = server.available();
        if (client.fd() >= 0) {
          stats.clientConnected = true;
          ++stats.connections;
          active = true;
        }
      }
    } else {
      bool closed = false;
      // Flush existing queues first. A full queue stops reads in that direction
      // while the opposite direction can still make progress.
      active |= transferToSerial(stats);
      active |= transferToNetwork(stats, closed);
      if (!closed) active |= readNetwork(stats, closed);
      if (!closed) active |= readSerial(stats);
      if (closed) closeClient(stats);
    }
    publishStats(stats);
    // The independent task survives a blocking portal shutdown, and yields
    // under continuous traffic and when either sink is blocked. On single-core
    // builds it necessarily shares the CPU with IR software carrier generation.
    const TickType_t waitTicks = pdMS_TO_TICKS(active ? 1 : 2);
    vTaskDelay(waitTicks == 0 ? 1 : waitTicks);
  }
}

}  // namespace

bool beginSerial() {
  if (getStats().serialReady) return true;
  if (serial.setRxBufferSize(kRxBufferSize) != kRxBufferSize ||
      serial.setTxBufferSize(kTxBufferSize) != kTxBufferSize)
    return false;
  serial.begin(kBaudRate);  // UART2 uses the existing board default RX/TX pins.
  if (!serial) return false;
  portENTER_CRITICAL(&statsMux);
  publishedStats.serialReady = true;
  portEXIT_CRITICAL(&statsMux);
  return true;
}

bool beginNetwork() {
  if (taskHandle) return true;
  if (!getStats().serialReady) return false;
  server.begin();
  if (!server) return false;
  server.setNoDelay(true);
  portENTER_CRITICAL(&statsMux);
  publishedStats.networkListening = true;
  portEXIT_CRITICAL(&statsMux);
  const BaseType_t result = xTaskCreatePinnedToCore(
      bridgeTask, "homegear_zigbee", 4096, nullptr, 2, &taskHandle,
      kBridgeCore);
  if (result != pdPASS) {
    taskHandle = nullptr;
    server.end();
    portENTER_CRITICAL(&statsMux);
    publishedStats.networkListening = false;
    portEXIT_CRITICAL(&statsMux);
    return false;
  }
  return true;
}

Stats getStats() {
  portENTER_CRITICAL(&statsMux);
  const Stats result = publishedStats;
  portEXIT_CRITICAL(&statsMux);
  return result;
}

void printStatus() {
  const Stats stats = getStats();
  Serial.println("ZIGBEE BEGIN");
  Serial.printf("ZIGBEE task_core=%d\n", static_cast<int>(kBridgeCore));
  Serial.printf(
      "ZIGBEE serial_ready=%u\nZIGBEE task_started=%u\n"
      "ZIGBEE listening=%u\nZIGBEE client_connected=%u\n"
      "ZIGBEE connections=%lu\nZIGBEE disconnects=%lu\n"
      "ZIGBEE socket_errors=%lu\nZIGBEE uart_errors=%lu\n"
      "ZIGBEE to_serial_pending=%lu\nZIGBEE to_network_pending=%lu\n"
      "ZIGBEE to_serial_high_water=%lu\nZIGBEE to_network_high_water=%lu\n"
      "ZIGBEE net_rx_bytes=%llu\nZIGBEE uart_tx_bytes=%llu\n"
      "ZIGBEE uart_rx_bytes=%llu\nZIGBEE net_tx_bytes=%llu\n"
      "ZIGBEE no_client_discard_bytes=%llu\n"
      "ZIGBEE disconnect_discard_bytes=%llu\n"
      "ZIGBEE net_backpressure=%lu\nZIGBEE uart_backpressure=%lu\n",
      stats.serialReady, stats.taskRunning, stats.networkListening,
      stats.clientConnected, static_cast<unsigned long>(stats.connections),
      static_cast<unsigned long>(stats.disconnects),
      static_cast<unsigned long>(stats.socketErrors),
      static_cast<unsigned long>(stats.uartErrors),
      static_cast<unsigned long>(stats.toSerialPending),
      static_cast<unsigned long>(stats.toNetworkPending),
      static_cast<unsigned long>(stats.toSerialHighWater),
      static_cast<unsigned long>(stats.toNetworkHighWater),
      static_cast<unsigned long long>(stats.networkReceived),
      static_cast<unsigned long long>(stats.serialTransmitted),
      static_cast<unsigned long long>(stats.serialReceived),
      static_cast<unsigned long long>(stats.networkTransmitted),
      static_cast<unsigned long long>(stats.serialDiscardedNoClient),
      static_cast<unsigned long long>(stats.bufferedDiscardedDisconnect),
      static_cast<unsigned long>(stats.networkBackpressure),
      static_cast<unsigned long>(stats.uartBackpressure));
  Serial.println("ZIGBEE END");
}

}  // namespace zigbee
}  // namespace homegear

#else

namespace homegear {
namespace zigbee {
bool beginSerial() { return true; }
bool beginNetwork() { return true; }
Stats getStats() { return Stats(); }
void printStatus() {}
}  // namespace zigbee
}  // namespace homegear

#endif
