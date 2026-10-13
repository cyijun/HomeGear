#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

#include "../Software/HomeGear/ZigbeeBridgeBuffer.h"

using homegear::zigbee::ByteRing;

std::vector<std::uint8_t> payload(std::size_t count, unsigned seed) {
  std::vector<std::uint8_t> bytes;
  for (std::size_t i = 0; i < count; ++i) {
    seed = seed * 1664525U + 1013904223U;
    bytes.push_back(static_cast<std::uint8_t>((seed >> 16) ^ i));
  }
  return bytes;
}

// A transport model with an independent source cursor and captured sink. Each
// call can transfer less than requested or nothing; the production queue must
// honor the returned count rather than assume an entire request succeeded.
struct Endpoint {
  explicit Endpoint(std::vector<std::uint8_t> data) : source(std::move(data)) {}

  std::ptrdiff_t read(std::uint8_t *destination, std::size_t requested) {
    ++readCalls;
    if (readFailure) return -1;
    const std::size_t count = std::min({requested, readLimit, source.size() - cursor});
    std::copy_n(source.data() + cursor, count, destination);
    cursor += count;
    return static_cast<std::ptrdiff_t>(count);
  }

  std::ptrdiff_t write(const std::uint8_t *data, std::size_t requested) {
    ++writeCalls;
    if (writeFailure) return -1;
    const std::size_t count = std::min(requested, writeLimit);
    sink.insert(sink.end(), data, data + count);
    return static_cast<std::ptrdiff_t>(count);
  }

  std::vector<std::uint8_t> source;
  std::vector<std::uint8_t> sink;
  std::size_t cursor = 0;
  std::size_t readLimit = 1000;
  std::size_t writeLimit = 1000;
  unsigned readCalls = 0;
  unsigned writeCalls = 0;
  bool readFailure = false;
  bool writeFailure = false;
};

template <std::size_t Capacity>
void step(ByteRing<Capacity> &ring, Endpoint &endpoint, std::size_t readBudget,
          std::size_t writeBudget) {
  ring.writeTo(writeBudget, [&](const std::uint8_t *data, std::size_t count) {
    return endpoint.write(data, count);
  });
  ring.readFrom(readBudget, [&](std::uint8_t *data, std::size_t count) {
    return endpoint.read(data, count);
  });
  assert(ring.size() <= Capacity);
  assert(ring.size() + ring.free() == Capacity);
}

void testPartialTransferAndWrap() {
  ByteRing<7> ring;
  Endpoint endpoint(payload(1231, 19));
  // Small, relatively prime capacities/limits force wrap and short I/O at
  // different boundaries; expected bytes come from the independent source.
  for (unsigned tick = 0; tick < 5000 &&
       (endpoint.cursor != endpoint.source.size() || ring.size() != 0); ++tick) {
    endpoint.readLimit = tick % 5 + 1;
    endpoint.writeLimit = tick % 4;  // Includes a blocked writer every fourth pass.
    step(ring, endpoint, 6, 5);
  }
  assert(endpoint.cursor == endpoint.source.size());
  assert(endpoint.sink == endpoint.source);
  assert(ring.size() == 0);
  assert(endpoint.readCalls > 200 && endpoint.writeCalls > 200);
}

void testFullQueueAppliesBackpressure() {
  ByteRing<8> ring;
  Endpoint endpoint(payload(100, 31));
  endpoint.writeLimit = 0;
  for (unsigned tick = 0; tick < 20; ++tick) step(ring, endpoint, 8, 8);
  assert(ring.size() == 8 && ring.free() == 0);
  assert(endpoint.cursor == 8 && endpoint.readCalls == 1);
  assert(endpoint.sink.empty());

  // A two-byte partial write frees exactly two bytes for the next read, rather
  // than dropping the six bytes that the sink could not yet take.
  endpoint.writeLimit = 2;
  step(ring, endpoint, 8, 8);
  assert(endpoint.cursor == 10 && endpoint.sink.size() == 2);
  assert(ring.size() == 8);
  endpoint.writeLimit = 3;
  for (unsigned tick = 0; tick < 100; ++tick) step(ring, endpoint, 8, 8);
  assert(endpoint.sink == endpoint.source && ring.size() == 0);
}

void testTemporaryFailurePreservesBytes() {
  ByteRing<11> ring;
  Endpoint endpoint(payload(23, 42));
  assert(ring.readFrom(9, [&](std::uint8_t *data, std::size_t count) {
    return endpoint.read(data, count);
  }) == 9);
  endpoint.readFailure = true;
  endpoint.writeFailure = true;
  assert(ring.readFrom(2, [&](std::uint8_t *data, std::size_t count) {
    return endpoint.read(data, count);
  }) == -1);
  assert(ring.writeTo(9, [&](const std::uint8_t *data, std::size_t count) {
    return endpoint.write(data, count);
  }) == -1);
  assert(ring.size() == 9 && endpoint.cursor == 9 && endpoint.sink.empty());
  endpoint.readFailure = endpoint.writeFailure = false;
  endpoint.readLimit = 3;
  endpoint.writeLimit = 2;
  for (unsigned tick = 0; tick < 40; ++tick) step(ring, endpoint, 4, 5);
  assert(endpoint.sink == endpoint.source && ring.size() == 0);
}

void testIndependentDirectionsUnderLongStall() {
  // Model a stalled network sink while TCP->UART continues. The queue reaches
  // capacity without further source reads; reopening it must deliver all data.
  ByteRing<2048> uartToNetwork;
  ByteRing<2048> networkToUart;
  Endpoint uplink(payload(40000, 83));
  Endpoint downlink(payload(31000, 79));
  for (unsigned tick = 0; tick < 1800; ++tick) {
    uplink.readLimit = 97;
    uplink.writeLimit = tick < 600 ? 0 : (tick % 7 == 0 ? 0 : 61);
    downlink.readLimit = tick % 3 == 0 ? 0 : 109;
    downlink.writeLimit = tick % 4 == 0 ? 0 : 53;
    step(uartToNetwork, uplink, 256, 256);
    step(networkToUart, downlink, 256, 128);
    if (tick == 599) {
      assert(uplink.cursor == 2048 && uartToNetwork.size() == 2048);
      assert(uplink.sink.empty());
      assert(downlink.sink.size() > 20000);
    }
  }
  assert(uplink.sink == uplink.source && uartToNetwork.size() == 0);
  assert(downlink.sink == downlink.source && networkToUart.size() == 0);
}

void testDisconnectDoesNotReplayOldCommands() {
  ByteRing<8> ring;
  Endpoint oldConnection(payload(8, 11));
  step(ring, oldConnection, 8, 8);
  assert(ring.size() == 8);
  ring.clear();
  Endpoint newConnection(payload(35, 13));
  for (unsigned tick = 0; tick < 20; ++tick) step(ring, newConnection, 5, 3);
  assert(newConnection.sink == newConnection.source);
  assert(ring.size() == 0);
}

int main() {
  testPartialTransferAndWrap();
  testFullQueueAppliesBackpressure();
  testTemporaryFailurePreservesBytes();
  testIndependentDirectionsUnderLongStall();
  testDisconnectDoesNotReplayOldCommands();
  std::cout << "ZigBee bridge: partial I/O, wrap, full queue/backpressure, errors, "
               "two-way traffic and disconnect passed\n";
}
