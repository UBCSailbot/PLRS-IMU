/**
 * Minimal SEGGER-RTT up-channel: telemetry over the SWD probe with no extra
 * wires. The target writes bytes into a RAM ring buffer; the debug host reads
 * them over SWD while the CPU keeps running (openocd `rtt server`). Up-channel
 * only, non-blocking (drops when the host is not draining), which keeps it off
 * the critical path of the 100 Hz loop.
 */

#pragma once

#ifdef ARDUINO
#include <cstddef>
#include <cstdint>

namespace rtt {

size_t write(const uint8_t *data, size_t len);
size_t write(uint8_t c);

} // namespace rtt

#endif
