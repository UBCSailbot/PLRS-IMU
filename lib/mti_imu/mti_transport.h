/**
 * Firmware side transport and clock for the MTI Xbus driver.
 *
 * This is the hardware specific implementation, depending on whichever HAL is
 * chosen. As the arduino framework is quick, that is the first HAL
 * implementation.
 */

#pragma once

#ifdef ARDUINO
#include "xbus_protocol.h"
#include <Arduino.h>

namespace mti {
/**
 * @brief Minimal UART transport layer. Make sure to setup pins and baud before
 * construction.
 */
class Uart {
public:
  explicit Uart(HardwareSerial &serial) : _serial(serial) {}

  void write(xbus::ByteSpan bytes) {
    _serial.write(bytes.data(), bytes.size());
  }

  std::optional<uint8_t> read() {
    if (_serial.available()) {
      return static_cast<uint8_t>(_serial.read());
    }
    return std::nullopt;
  }

private:
  HardwareSerial &_serial;
};

} // namespace mti

#endif
