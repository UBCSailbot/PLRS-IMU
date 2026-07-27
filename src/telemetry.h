/**
 * Telemetry sink that fans one stream out to three places at once:
 *   - the USB CDC (boat readout), written only while the host is connected so a
 *     headless run never blocks on it;
 *   - RTT over the SWD probe (bench readout, no extra wires; see rtt.h);
 *   - a hardware UART (fallback bench readout over the probe's UART bridge).
 * Boat and bench share one firmware.
 */

#pragma once

#ifdef ARDUINO
#include "rtt.h"
#include <Arduino.h>
#include <cstddef>

namespace plrs {

class TelemetrySink : public Print {
public:
  explicit TelemetrySink(Print &uart) : _uart(uart) {}

  size_t write(uint8_t c) override {
    if (Serial) {
      Serial.write(c);
    }
    rtt::write(c);
    return _uart.write(c);
  }

  size_t write(const uint8_t *buffer, size_t size) override {
    if (Serial) {
      Serial.write(buffer, size);
    }
    rtt::write(buffer, size);
    return _uart.write(buffer, size);
  }

private:
  Print &_uart;
};

} // namespace plrs

#endif
