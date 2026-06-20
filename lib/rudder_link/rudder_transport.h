/**
 * Firmware side transport for the rudder link. Arduino HAL.
 *
 * Send only: the link is one-way (see docs/rudder_link.md).
 */

#pragma once

#ifdef ARDUINO
#include "rudder_protocol.h"
#include <Arduino.h>

namespace rudder {
/**
 * @brief Minimal UART transport. Set up pins and baud before construction.
 */
class Uart {
public:
  explicit Uart(HardwareSerial &serial) : _serial(serial) {}

  void write(ByteSpan bytes) { _serial.write(bytes.data(), bytes.size()); }

private:
  HardwareSerial &_serial;
};

} // namespace rudder

#endif
