/**
 * Test-only block-building helpers. Not part of the driver.
 *
 * The CRC here is table-driven and written independently of the production
 * bitwise crc_ccitt() so that a bug in one cannot mask a bug in the other.
 */

#pragma once
#include "sbf_protocol.h"
#include <array>
#include <cstdint>
#include <vector>

namespace stest {

namespace detail {

constexpr std::array<uint16_t, 256> build_crc_table() {
  std::array<uint16_t, 256> t{};
  for (uint16_t i = 0; i < 256; i++) {
    uint16_t c = static_cast<uint16_t>(i << 8);
    for (int j = 0; j < 8; j++) {
      c = (c & 0x8000) ? static_cast<uint16_t>((c << 1) ^ 0x1021)
                       : static_cast<uint16_t>(c << 1);
    }
    t[i] = c;
  }
  return t;
}

inline uint16_t crc_ccitt_xmodem(const std::vector<uint8_t> &data) {
  static constexpr auto TABLE = build_crc_table();
  uint16_t crc = 0;
  for (uint8_t b : data) {
    crc = static_cast<uint16_t>((crc << 8) ^ TABLE[(crc >> 8) ^ b]);
  }
  return crc;
}

} // namespace detail

/**
 * @brief Assemble a complete SBF block: sync + crc + id + length + body.
 *
 * Body is taken verbatim — caller is responsible for any padding to make
 * length a multiple of 4, so misalignment tests can pass unpadded bodies
 * deliberately.
 *
 * @param id    Full 16-bit ID field (block number in bits 0-12, revision
 *              in bits 13-15).
 * @param body  Block body bytes (zero or more).
 *
 * @return The wire bytes of the block.
 */
inline std::vector<uint8_t> make_block(uint16_t id,
                                       const std::vector<uint8_t> &body) {
  const uint16_t length = static_cast<uint16_t>(sbf::HEADER_LEN + body.size());

  std::vector<uint8_t> crc_input;
  crc_input.push_back(static_cast<uint8_t>(id & 0xFF));
  crc_input.push_back(static_cast<uint8_t>(id >> 8));
  crc_input.push_back(static_cast<uint8_t>(length & 0xFF));
  crc_input.push_back(static_cast<uint8_t>(length >> 8));
  for (uint8_t b : body)
    crc_input.push_back(b);
  const uint16_t crc = detail::crc_ccitt_xmodem(crc_input);

  std::vector<uint8_t> f;
  f.push_back(sbf::SYNC1);
  f.push_back(sbf::SYNC2);
  f.push_back(static_cast<uint8_t>(crc & 0xFF));
  f.push_back(static_cast<uint8_t>(crc >> 8));
  for (uint8_t b : crc_input)
    f.push_back(b);
  return f;
}

/**
 * @brief Variant of make_block that lets the caller force a specific length
 *        field, even if it mismatches body.size(). Used to test the parser's
 *        length-validation paths (not multiple of 4, < HEADER_LEN, >
 * MAX_BLOCK).
 */
inline std::vector<uint8_t>
make_block_with_length(uint16_t id, uint16_t length,
                       const std::vector<uint8_t> &body) {
  std::vector<uint8_t> crc_input;
  crc_input.push_back(static_cast<uint8_t>(id & 0xFF));
  crc_input.push_back(static_cast<uint8_t>(id >> 8));
  crc_input.push_back(static_cast<uint8_t>(length & 0xFF));
  crc_input.push_back(static_cast<uint8_t>(length >> 8));
  for (uint8_t b : body)
    crc_input.push_back(b);
  const uint16_t crc = detail::crc_ccitt_xmodem(crc_input);

  std::vector<uint8_t> f;
  f.push_back(sbf::SYNC1);
  f.push_back(sbf::SYNC2);
  f.push_back(static_cast<uint8_t>(crc & 0xFF));
  f.push_back(static_cast<uint8_t>(crc >> 8));
  for (uint8_t b : crc_input)
    f.push_back(b);
  return f;
}

} // namespace stest
