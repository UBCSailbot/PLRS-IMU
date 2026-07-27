/**
 * SHTP (Sensor Hub Transport Protocol) framing for the BNO085.
 *
 * All fields are little endian. A cargo is one SHTP transfer: a 4-byte header
 * followed by its payload. Over I2C each read returns exactly one cargo, so
 * framing is stateless free functions (there is no half-decoded frame to
 * defend, unlike the xbus UART parser).
 */

#pragma once

#include "common.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace shtp {

using plrs::ByteSpan;

constexpr std::size_t HEADER_LEN = 4;
// Length lives in the low 15 bits of the header; the top bit flags a transfer
// that continues a previous one (never set for the small reports we exchange).
constexpr uint16_t LENGTH_MASK = 0x7fff;
constexpr uint16_t CONTINUATION_BIT = 0x8000;

// Longest control payload we send (Set-Feature is 17 bytes).
constexpr std::size_t MAX_TX_PAYLOAD = 32;
constexpr std::size_t MAX_TX_FRAME = HEADER_LEN + MAX_TX_PAYLOAD;

/**
 * SHTP channel numbers (fixed by the SH-2 firmware).
 */
enum class Channel : uint8_t {
  Command = 0,
  Executable = 1,   // reset / on-off, and reset-complete notifications
  Control = 2,      // SH-2 command + response (Set-Feature, Product ID)
  InputReports = 3, // normal sensor reports
  WakeInput = 4,
  GyroRv = 5,
};

/**
 * One decoded SHTP cargo. `payload` borrows the input span => valid only while
 * that span is in scope.
 */
struct Packet {
  uint8_t channel;
  uint8_t seq;
  ByteSpan payload;
};

/**
 * Serialized cargo ready to send.
 */
struct Encoded {
  std::array<uint8_t, MAX_TX_FRAME> bytes {};
  std::size_t len = 0;

  constexpr ByteSpan view() const { return {bytes.data(), len}; }
};

/**
 * @brief Split one SHTP cargo into channel, sequence, and payload.
 *
 * @param cargo  A full cargo: 4-byte header plus payload. Extra trailing bytes
 *   past the header's declared length are ignored.
 *
 * @return The decoded packet, or nullopt if the cargo is shorter than the
 *   header or shorter than the length the header declares. A header declaring
 *   length 0 (an empty read, i.e. no data pending) yields nullopt.
 */
constexpr std::optional<Packet> parse_packet(ByteSpan cargo) {
  if (cargo.size() < HEADER_LEN) {
    return std::nullopt;
  }
  const std::size_t total =
      plrs::read_u16_little_endian(cargo.subspan(0, 2)) & LENGTH_MASK;
  if (total < HEADER_LEN || total > cargo.size()) {
    return std::nullopt;
  }
  return Packet {
      .channel = cargo[2],
      .seq = cargo[3],
      .payload = cargo.subspan(HEADER_LEN, total - HEADER_LEN),
  };
}

/**
 * @brief Frame a payload into an SHTP cargo on a given channel.
 *
 * @param channel  Destination channel.
 * @param seq  Per-channel sequence number (the host tracks its own).
 * @param payload  Cargo payload. Must not exceed MAX_TX_PAYLOAD.
 *
 * @return The framed cargo, or nullopt if the payload is too long.
 */
constexpr std::optional<Encoded>
encode_packet(Channel channel, uint8_t seq, ByteSpan payload) {
  if (payload.size() > MAX_TX_PAYLOAD) {
    return std::nullopt;
  }
  const auto len = plrs::write_u16_little_endian(
      static_cast<uint16_t>(HEADER_LEN + payload.size()));

  Encoded e;
  std::size_t i = 0;
  e.bytes[i++] = len[0];
  e.bytes[i++] = len[1];
  e.bytes[i++] = static_cast<uint8_t>(channel);
  e.bytes[i++] = seq;
  for (const uint8_t byte : payload) {
    e.bytes[i++] = byte;
  }
  e.len = i;
  return e;
}

} // namespace shtp
