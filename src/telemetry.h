/**
 * Telemetry sink that fans one stream out to three places at once:
 *   - the USB CDC (boat readout), written only while the host is connected so a
 *     headless run never blocks on it;
 *   - RTT over the SWD probe (bench readout, no extra wires; see rtt.h);
 *   - a hardware UART (fallback bench readout over the probe's UART bridge).
 * Boat and bench share one firmware.
 *
 * Several tasks write here concurrently, and a telemetry line is several Print
 * calls long (a tag, its fields, the newline), so the invariant the sink has to
 * defend is that a whole line reaches the wire uninterrupted. It does that by
 * handing out a Line: an RAII hold on the sink that is the only way to write.
 * Interleaving is then a compile-time impossibility rather than a convention,
 * which matters because the failure mode is silent -- a spliced line still
 * parses as two damaged ones.
 */

#pragma once

#ifdef ARDUINO
#include "rtt.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <cstddef>
#include <semphr.h>

namespace plrs {

class TelemetrySink {
public:
  explicit TelemetrySink(Print &uart)
      : _uart(uart), _mutex(xSemaphoreCreateMutex()) {}

  /**
   * Exclusive hold on the sink for one line. Construct it, print the whole
   * line through it, and let it go out of scope; no other task can write in
   * between. Holding one across a blocking wait would stall every other
   * task's telemetry, so keep the scope to a single line.
   */
  class Line : public Print {
  public:
    explicit Line(TelemetrySink &sink) : _sink(sink) {
      if (_sink._mutex != nullptr) {
        xSemaphoreTake(_sink._mutex, portMAX_DELAY);
      }
    }

    ~Line() {
      if (_sink._mutex != nullptr) {
        xSemaphoreGive(_sink._mutex);
      }
    }

    Line(const Line &) = delete;
    Line &operator=(const Line &) = delete;

    size_t write(uint8_t c) override { return _sink.emit(&c, 1); }

    size_t write(const uint8_t *buffer, size_t size) override {
      return _sink.emit(buffer, size);
    }

  private:
    TelemetrySink &_sink;
  };

  /** @brief Take the sink for the duration of one telemetry line. */
  Line line() { return Line {*this}; }

private:
  size_t emit(const uint8_t *buffer, size_t size) {
    if (Serial) {
      Serial.write(buffer, size);
    }
    rtt::write(buffer, size);
    return _uart.write(buffer, size);
  }

  Print &_uart;
  // Null only if the FreeRTOS heap was exhausted at construction, which on this
  // firmware means boot has already failed; writes then fall back to unlocked.
  SemaphoreHandle_t _mutex;
};

} // namespace plrs

#endif
