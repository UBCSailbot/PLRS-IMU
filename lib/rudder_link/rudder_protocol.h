/**
 * Rudder link protocol. COBS-framed, little-endian.
 *
 * See docs/rudder_link.md for the wire format.
 */

#pragma once

#include "common.h"
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace rudder {

/*
 * Constants and Definitions.
 */

constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint8_t FRAME_DELIMITER = 0x00;

constexpr std::size_t MAX_PAYLOAD = 64;
constexpr std::size_t HEADER_BYTES = 3; // ver, msg_id, seq
constexpr std::size_t CRC_BYTES = 2;
constexpr std::size_t MAX_DATA = HEADER_BYTES + MAX_PAYLOAD + CRC_BYTES;
// COBS adds at least one code byte per frame, plus the trailing delimiter.
constexpr std::size_t MAX_FRAME = MAX_DATA + MAX_DATA / 254 + 2;

using ByteSpan = std::span<const uint8_t>;
using Ms = std::chrono::milliseconds;

/**
 * Message IDs. Each selects a payload layout (see docs/rudder_link.md).
 */
enum class MsgId : uint8_t {
  Heading = 0x01,
};

/*
 * Integrity.
 */

constexpr uint16_t CRC16_POLY = 0x1021;
constexpr uint16_t CRC16_INIT = 0xFFFF;

/**
 * @brief CRC-16/CCITT-FALSE over a byte span.
 *
 * Poly 0x1021, init 0xFFFF, no input/output reflection, no final xor.
 *
 * @param data  Bytes to checksum.
 *
 * @return The 16-bit CRC.
 */
constexpr uint16_t crc16(ByteSpan data) {
  uint16_t crc = CRC16_INIT;
  for (uint8_t byte : data) {
    crc ^= static_cast<uint16_t>(byte << 8);
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ CRC16_POLY)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

/*
 * Framing.
 */

/**
 * A COBS-framed frame ready to send, terminated by FRAME_DELIMITER.
 */
struct Encoded {
  std::array<uint8_t, MAX_FRAME> bytes {};
  std::size_t len = 0;

  constexpr ByteSpan view() const { return {bytes.data(), len}; }
};

/**
 * @brief COBS-encode a byte span and append the frame delimiter.
 *
 * @param data  Bytes to encode; at most MAX_DATA.
 *
 * @return The encoded frame ending in FRAME_DELIMITER, or nullopt if data
 *   exceeds MAX_DATA.
 */
constexpr std::optional<Encoded> cobs_encode(ByteSpan data) {
  if (data.size() > MAX_DATA) {
    return std::nullopt;
  }

  Encoded e;
  std::size_t out = 0;
  std::size_t code_index = out++; // reserved; filled when the block closes
  uint8_t code = 1;

  for (uint8_t byte : data) {
    if (byte != FRAME_DELIMITER) {
      e.bytes[out++] = byte;
      code++;
      if (code != 0xFF) {
        continue;
      }
    }
    e.bytes[code_index] = code;
    code_index = out++;
    code = 1;
  }

  e.bytes[code_index] = code;
  e.bytes[out++] = FRAME_DELIMITER;
  e.len = out;
  return e;
}

/**
 * A COBS-decoded frame body (header, payload, CRC).
 */
struct Decoded {
  std::array<uint8_t, MAX_DATA> bytes {};
  std::size_t len = 0;

  constexpr ByteSpan view() const { return {bytes.data(), len}; }
};

/**
 * @brief COBS-decode a frame block with the trailing delimiter already
 * stripped.
 *
 * @param data  The encoded block, without FRAME_DELIMITER.
 *
 * @return The decoded bytes, or nullopt if the block is malformed or decodes to
 *   more than MAX_DATA bytes.
 */
constexpr std::optional<Decoded> cobs_decode(ByteSpan data) {
  Decoded d;
  std::size_t i = 0;
  while (i < data.size()) {
    uint8_t code = data[i++];
    if (code == FRAME_DELIMITER) {
      return std::nullopt; // a zero cannot occur inside a COBS block
    }
    for (uint8_t j = 1; j < code; j++) {
      if (i >= data.size() || d.len >= MAX_DATA) {
        return std::nullopt;
      }
      d.bytes[d.len++] = data[i++];
    }
    // a block shorter than 0xFF stands in for a zero, except the final block
    if (code != 0xFF && i < data.size()) {
      if (d.len >= MAX_DATA) {
        return std::nullopt;
      }
      d.bytes[d.len++] = FRAME_DELIMITER;
    }
  }
  return d;
}

} // namespace rudder
