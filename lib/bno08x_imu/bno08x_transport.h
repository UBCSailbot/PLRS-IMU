/**
 * Firmware-side I2C transport for the BNO085 SHTP link.
 *
 * This is the hardware-specific half of the driver; the Arduino-free protocol
 * lives in shtp_protocol.h / sh2_reports.h. Construct with an already-`begun`
 * TwoWire (pins, clock, and begin() belong in main.cpp setup(), not here).
 */

#pragma once

#ifdef ARDUINO
#include "shtp_protocol.h"
#include <Arduino.h>
#include <Wire.h>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace bno08x {

using plrs::ByteSpan;

// Adafruit breakout default 7-bit address (0x4B if the ADR jumper is bridged).
constexpr uint8_t DEFAULT_ADDR = 0x4a;

// Largest cargo we buffer: a reset advertisement is ~270 bytes; a sensor batch
// is well under 64. Sized to hold the advertisement so bring-up can drain it.
constexpr std::size_t MAX_CARGO = 300;

// Payload bytes per I2C read. The BNO re-sends the 4-byte SHTP header on every
// read of a multi-read cargo, so each slice moves `CHUNK_PAYLOAD` payload bytes
// and 4 header bytes; 28 keeps each transaction at a universally safe 32 bytes.
constexpr std::size_t CHUNK_PAYLOAD = 28;

/**
 * @brief Minimal SHTP-over-I2C transport. Poll-driven; no INT/RST line needed.
 */
class I2cTransport {
public:
  I2cTransport(TwoWire &wire, uint8_t addr = DEFAULT_ADDR)
      : _wire(wire), _addr(addr) {}

  /**
   * @brief Whether the sensor ACKs its address (bus/power/wiring sanity check).
   */
  bool present() {
    _wire.beginTransmission(_addr);
    return _wire.endTransmission() == 0;
  }

  /**
   * @brief Scan the bus and report each 7-bit address that ACKs, to `out`.
   *
   * A bring-up diagnostic: with nothing ACKing, the sensor is unpowered or
   * mis-wired; the wrong address ACKing flags the ADR jumper.
   */
  void scan(Print &out) {
    out.print("# I2C scan:");
    bool any = false;
    for (uint8_t a = 0x08; a < 0x78; a++) {
      _wire.beginTransmission(a);
      if (_wire.endTransmission() == 0) {
        out.print(" 0x");
        out.print(a, HEX);
        any = true;
      }
    }
    if (!any) {
      out.print(" (none)");
    }
    out.println();
  }

  /**
   * @brief Read one full SHTP cargo (header + payload) into `out`.
   *
   * Peeks the 4-byte header for the cargo length, then reads the whole cargo in
   * <=32-byte transactions, dropping the header the BNO re-sends on every
   * continued read. Feed the returned span to shtp::parse_packet.
   *
   * @param out  Scratch buffer; should be MAX_CARGO to hold a reset
   *   advertisement. A cargo longer than `out` is fully drained off the bus
   *   (to keep framing aligned) but reported truncated to `out.size()`.
   *
   * @return The cargo span, or nullopt when no data is pending (header length
   * 0) or an I2C read comes up short.
   */
  std::optional<ByteSpan> read_cargo(std::span<uint8_t> out) {
    if (out.size() < shtp::HEADER_LEN) {
      return std::nullopt;
    }
    if (_wire.requestFrom(_addr, static_cast<size_t>(shtp::HEADER_LEN)) !=
        shtp::HEADER_LEN) {
      return std::nullopt;
    }
    for (std::size_t i = 0; i < shtp::HEADER_LEN; i++) {
      out[i] = static_cast<uint8_t>(_wire.read());
    }
    const std::size_t total =
        (static_cast<std::size_t>(out[1]) << 8 | out[0]) & shtp::LENGTH_MASK;
    if (total < shtp::HEADER_LEN) {
      return std::nullopt; // length 0 => nothing pending
    }

    std::size_t remaining = total - shtp::HEADER_LEN;
    std::size_t stored = shtp::HEADER_LEN; // header already in out
    while (remaining > 0) {
      const std::size_t chunk = std::min(CHUNK_PAYLOAD, remaining);
      const std::size_t want = chunk + shtp::HEADER_LEN;
      if (_wire.requestFrom(_addr, want) != want) {
        return std::nullopt;
      }
      for (std::size_t i = 0; i < shtp::HEADER_LEN; i++) {
        _wire.read(); // continuation header, re-sent each read
      }
      for (std::size_t i = 0; i < chunk; i++) {
        const uint8_t b = static_cast<uint8_t>(_wire.read());
        if (stored < out.size()) {
          out[stored++] = b;
        }
      }
      remaining -= chunk;
    }
    return ByteSpan(out.data(), stored);
  }

  /**
   * @brief Send one already-framed SHTP cargo (shtp::Encoded::view()).
   */
  void write_packet(ByteSpan cargo) {
    _wire.beginTransmission(_addr);
    _wire.write(cargo.data(), cargo.size());
    _wire.endTransmission();
  }

private:
  TwoWire &_wire;
  uint8_t _addr;
};

} // namespace bno08x

#endif
