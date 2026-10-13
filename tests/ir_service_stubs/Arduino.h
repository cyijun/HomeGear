#pragma once
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#define ARDUINOJSON_ENABLE_ARDUINO_STREAM 0
#define ARDUINOJSON_ENABLE_ARDUINO_PRINT 0
#define ARDUINOJSON_ENABLE_PROGMEM 0
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <string>

class String {
 public:
  String() = default;
  String(const char *text) : value_(text ? text : "") {}
  String(const char *text, size_t length) : value_(text, length) {}
  String(const std::string &text) : value_(text) {}
  String(float number) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.2f", static_cast<double>(number));
    value_ = buffer;
  }
  String(int number) : value_(std::to_string(number)) {}
  const char *c_str() const { return value_.c_str(); }
  size_t length() const { return value_.length(); }
  bool isEmpty() const { return value_.empty(); }
  bool concat(const char *text) { value_ += text; return true; }
  bool equals(const char *text) const { return value_ == text; }
  long toInt() const { return std::strtol(value_.c_str(), nullptr, 10); }
  friend String operator+(String lhs, const char *rhs) { lhs.concat(rhs); return lhs; }
  friend String operator+(String lhs, const String &rhs) { lhs.concat(rhs.c_str()); return lhs; }
  String &operator+=(const String &rhs) { concat(rhs.c_str()); return *this; }
 private:
  std::string value_;
};
class MockSerial {
 public:
  std::string output;
  void print(const char *text) { output += text; }
  void print(const String &text) { output.append(text.c_str(), text.length()); }
  void println(const char *text) { print(text); output += '\n'; }
  void println(const String &text) { print(text); output += '\n'; }
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
namespace ir_mock { extern uint32_t now; extern bool clockReady; extern unsigned clockDepth, clockUses; }
inline uint32_t millis() { return ir_mock::now; }
