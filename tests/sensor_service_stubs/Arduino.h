#pragma once
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#define ARDUINOJSON_ENABLE_ARDUINO_STREAM 0
#define ARDUINOJSON_ENABLE_ARDUINO_PRINT 0
#define ARDUINOJSON_ENABLE_PROGMEM 0
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <string>

class String {
 public:
  String() = default;
  String(const char *text) : value_(text) {}
  String(float number) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.2f", static_cast<double>(number));
    value_ = buffer;
  }
  const char *c_str() const { return value_.c_str(); }
  size_t length() const { return value_.length(); }
  bool concat(const char *text) { value_ += text; return true; }
  friend String operator+(String lhs, const char *rhs) { lhs.concat(rhs); return lhs; }
 private:
  std::string value_;
};

namespace sensor_mock {
extern uint32_t now;
extern bool shtHealthy, crcHealthy, lightsHealthy, bsecHealthy;
extern float temperature;
extern unsigned shtReads, lightReads, wireBegins, bsecConfigurations;
extern uint32_t bsecOutputMask;
extern bool bsecZeroSignals;
extern int bsecAlgorithmStatus;
}
inline uint32_t millis() { return sensor_mock::now; }
class MockSerial {
 public:
  std::string output;
  void println(const char *text) { output += text; output += '\n'; }
  void printf(const char *format, ...) {
    char buffer[4096];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    output += buffer;
  }
};
extern MockSerial Serial;
