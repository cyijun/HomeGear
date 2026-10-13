#pragma once
#include <cstdint>
#include <string>
struct MockSerial {
  std::string output;
  void println(const char *text);
  void printf(const char *format, ...);
};
extern MockSerial Serial;
struct MockEsp {
  uint32_t getFreeHeap() const { return 120000; }
  uint32_t getMinFreeHeap() const { return 110000; }
};
extern MockEsp ESP;
uint32_t millis();
void delay(uint32_t milliseconds);
