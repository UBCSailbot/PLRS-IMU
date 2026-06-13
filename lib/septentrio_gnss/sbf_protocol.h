/**
 * Communication Library for Septentrio's SBF protocol.
 *
 * Note that all messages are little endian. Reads still go through bit_cast
 * over a byte array so misaligned offsets do not hardfault on Cortex-M0+.
 */

#pragma once

#include <array>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

namespace sbf {

/*
 * Constants and Definitions.
 */

constexpr uint8_t SYNC1 = 0x24; // '$'
constexpr uint8_t SYNC2 = 0x40; // '@'
constexpr std::size_t HEADER_LEN = 8;
constexpr std::size_t MAX_BODY = 504;
constexpr std::size_t MAX_BLOCK = HEADER_LEN + MAX_BODY;
constexpr std::size_t LENGTH_ALIGNMENT = 4; // Length is always a multiple of 4
constexpr uint16_t ID_BLOCK_MASK = 0x1FFF;  // bits 0-12
constexpr uint8_t ID_REV_SHIFT = 13;        // bits 13-15
constexpr uint16_t CRC_POLYNOMIAL = 0x1021;
constexpr uint16_t CRC_INIT = 0x0000;
constexpr uint16_t CRC_TOP_BIT = 0x8000;
constexpr std::size_t BITS_PER_BYTE = 8;

using ByteSpan = std::span<const uint8_t>;
using Ms = std::chrono::milliseconds;

/*
 * Helper functions.
 */

template <typename T>
constexpr T read_little_endian(ByteSpan b, std::size_t offset) {
  std::array<std::byte, sizeof(T)> tmp {};
  for (std::size_t i = 0; i < sizeof(T); i++) {
    tmp[i] = std::byte {b[offset + i]};
  }
  return std::bit_cast<T>(tmp);
}

/**
 * @brief CRC-CCITT/XMODEM variant: poly 0x1021, init 0, no reflection, no
 *        final XOR. Do not substitute another "CRC-CCITT" — the seed and
 *        reflection choices differ across variants and silently break framing.
 *
 * @param data: Bytes to checksum (ID + Length + Body for SBF blocks).
 *
 * @return The 16-bit CRC of `data`.
 */
constexpr uint16_t crc_ccitt(ByteSpan data, uint16_t init = CRC_INIT) {
  uint16_t crc = init;
  for (uint8_t byte : data) {
    crc ^= static_cast<uint16_t>(byte) << BITS_PER_BYTE;
    for (std::size_t i = 0; i < BITS_PER_BYTE; i++) {
      crc = (crc & CRC_TOP_BIT)
                ? static_cast<uint16_t>((crc << 1) ^ CRC_POLYNOMIAL)
                : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

/*
 * Block structure.
 */

struct Packet {
  uint16_t id = 0;
  uint16_t body_length = 0;
  std::array<uint8_t, MAX_BODY> data {};

  constexpr ByteSpan body() const { return {data.data(), body_length}; }
  constexpr uint16_t block_number() const { return id & ID_BLOCK_MASK; }
  constexpr uint8_t revision() const {
    return static_cast<uint8_t>(id >> ID_REV_SHIFT);
  }
};

} // namespace sbf
