#include "HomeGearConfig.h"
#if RF_MODULE
#include "LegacyRemoteInternals.h"
#include "RF_Code.h"
#include <HardwareSerial.h>

namespace homegear {
namespace legacy {
namespace detail {
namespace rf {
namespace {
HardwareSerial &serial() { static HardwareSerial value(1); return value; }
}  // namespace
void begin() { serial().begin(115200, SERIAL_8N1, 13, 12); }
void send(const unsigned char data[6]) {
  serial().write(rf_module_uart_protocol[0]);
  for (int i = 0; i < 6; ++i) serial().write(data[i]);
  serial().write(rf_module_uart_protocol[1]);
}
void setBaud() { send(rf_module_set_baudrate_to_115200); }

}  // namespace rf
}  // namespace detail
}  // namespace legacy
}  // namespace homegear
#endif
