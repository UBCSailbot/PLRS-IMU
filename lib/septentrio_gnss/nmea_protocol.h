/**
 * NMEA 0183 sentence data type and constants.
 *
 * Parsing is performed by septentrio_gnss::Parser (wire_parser.h), which
 * dispatches between SBF and NMEA based on the byte after '$': '@' means
 * SBF, anything else means NMEA.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace nmea {

/*
 * Constants and Definitions.
 */

constexpr uint8_t START = 0x24;              // '$'
constexpr uint8_t CHECKSUM_DELIMITER = 0x2A; // '*'
constexpr uint8_t CR = 0x0D;
constexpr uint8_t LF = 0x0A;
constexpr uint8_t XOR_INIT = 0x00;
constexpr std::size_t MAX_BODY =
    128; // NMEA 0183 standard cap is 82; Septentrio can exceed
constexpr uint8_t HEX_LETTER_OFFSET = 10; // 'A' represents 10 in hex
constexpr uint8_t NIBBLE_BITS = 4;

/*
 * Sentence structure.
 *
 * Holds the bytes between '$' and '*' (i.e. excluding the start byte, the
 * checksum delimiter, the checksum itself, and the CR/LF terminator).
 */

struct Sentence {
  std::array<char, MAX_BODY> data{};
  std::size_t length = 0;

  /**
   * @brief View of the sentence body as a string, e.g.
   *        "GPGGA,123519,4807.038,N,...".
   */
  constexpr std::string_view text() const { return {data.data(), length}; }
};

/*
 * Helper functions.
 */

/**
 * @brief Convert one ASCII hex digit to its 0-15 value, or -1 if not a hex
 *        digit. Used by the wire parser when validating the checksum.
 */
constexpr int hex_to_nibble(uint8_t c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + HEX_LETTER_OFFSET;
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + HEX_LETTER_OFFSET;
  }
  return -1;
}

} // namespace nmea
