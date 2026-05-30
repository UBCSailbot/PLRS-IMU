/**
 * Firmware side transport and clock for the Septentrio GNSS driver.
 *
 * Mirrors lib/mti_imu/mti_transport.h. Compiles only under the Arduino
 * framework; the protocol headers stay Arduino-free for host testing.
 */

#pragma once

#ifdef ARDUINO
#include "wire_parser.h"
#include <Arduino.h>
#include <optional>
#include <string_view>

namespace septentrio_gnss {
/**
 * @brief Monotonic 64-bit millisecond clock built from Arduino's 32-bit
 * clock. Rollover on a 32-bit clock occurs every ~50 days.
 *
 * @return The number of milliseconds elapsed since boot.
 */
inline Ms millis64() {
  static uint32_t last = 0;
  static uint32_t high = 0;
  uint32_t now = millis();

  if (now < last) {
    high += (uint64_t(1) << 32); // detect wraparound
    last = now;
  }
  return Ms(high | now);
}

/**
 * @brief Minimal UART transport layer. Make sure to setup pins and baud
 * before construction.
 */
class Uart {
public:
  explicit Uart(HardwareSerial &serial) : _serial(serial) {}

  void write(sbf::ByteSpan bytes) { _serial.write(bytes.data(), bytes.size()); }

  void write(std::string_view text) { _serial.write(text.data(), text.size()); }

  std::optional<uint8_t> read() {
    if (_serial.available()) {
      return static_cast<uint8_t>(_serial.read());
    }
    return std::nullopt;
  }

private:
  HardwareSerial &_serial;
};

} // namespace septentrio_gnss

#endif
