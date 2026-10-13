#pragma once
#include "Arduino.h"
#include <cassert>
#include <functional>
#include <vector>
class IRsend {
 public:
  explicit IRsend(uint16_t) {}
  unsigned beginCalls = 0;
  unsigned activeSends = 0;
  std::vector<std::vector<uint8_t>> frames;
  std::function<void()> duringSend;
  void begin() { ++beginCalls; }
  void sendHitachiAc3(const uint8_t *frame, uint16_t length, uint16_t repeats) {
    assert(ir_mock::clockReady && ir_mock::clockDepth == 1);
    assert(activeSends == 0 && length == 23 && repeats == 0);
    ++activeSends;
    frames.emplace_back(frame, frame + length);
    if (duringSend) duringSend();
    --activeSends;
  }
};
