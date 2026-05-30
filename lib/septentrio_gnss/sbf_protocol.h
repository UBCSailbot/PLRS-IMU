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
  std::array<std::byte, sizeof(T)> tmp{};
  for (std::size_t i = 0; i < sizeof(T); i++) {
    tmp[i] = std::byte{b[offset + i]};
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
  std::array<uint8_t, MAX_BODY> data{};

  constexpr ByteSpan body() const { return {data.data(), body_length}; }
  constexpr uint16_t block_number() const { return id & ID_BLOCK_MASK; }
  constexpr uint8_t revision() const {
    return static_cast<uint8_t>(id >> ID_REV_SHIFT);
  }
};

/*
 * Receiving.
 */

class Parser {
public:
  /**
   * Mid-frame timeout. Bytes that arrive more than FRAME_TIMEOUT after the
   * previous byte cause the partial frame to be dropped before processing.
   *
   * At 115200 baud a full 512-byte block takes ~45 ms.
   */
  static constexpr Ms FRAME_TIMEOUT{250};

  /**
   * @brief Reset the parser and drop the current frame.
   */
  void reset() { _state = State::Sync1; }

  /**
   * @brief Check if the parser is mid frame.
   *
   * @return true if the parser is mid frame.
   */
  bool mid_frame() const { return _state != State::Sync1; }

  /**
   * @brief Feed one byte into the parser.
   *
   * @param byte. The current byte.
   * @param now. The current time.
   *
   * @return A completed packet, or nullopt if the frame isn't done. If a
   * frame exceeds FRAME_TIMEOUT then that frame is dropped and the next
   * frame is returned.
   */
  std::optional<Packet> feed(uint8_t byte, Ms now) {
    if (_state != State::Sync1 && (now - _last_advance) > FRAME_TIMEOUT) {
      reset();
    }
    _last_advance = now;

    switch (_state) {
    case State::Sync1:
      if (byte == SYNC1)
        _state = State::Sync2;
      break;
    case State::Sync2:
      if (byte == SYNC2)
        _state = State::CrcLo;
      else if (byte != SYNC1)
        reset();
      // else: $$… — stay in Sync2, the second $ might be the real start
      break;
    case State::CrcLo:
      _crc_low_byte = byte;
      _state = State::CrcHi;
      break;
    case State::CrcHi:
      _expected_crc =
          static_cast<uint16_t>((byte << BITS_PER_BYTE) | _crc_low_byte);
      _state = State::IdLo;
      break;
    case State::IdLo:
      _header[ID_FIELD_OFFSET] = byte;
      _state = State::IdHi;
      break;
    case State::IdHi:
      _header[ID_FIELD_OFFSET + 1] = byte;
      _state = State::LenLo;
      break;
    case State::LenLo:
      _header[LENGTH_FIELD_OFFSET] = byte;
      _state = State::LenHi;
      break;
    case State::LenHi:
      _header[LENGTH_FIELD_OFFSET + 1] = byte;
      return _enter_body();
    case State::Body:
      _buffer[_body_index++] = byte;
      if (_body_index >= _body_length) {
        return _verify();
      }
      break;
    }
    return std::nullopt;
  }

private:
  enum class State : uint8_t {
    Sync1,
    Sync2,
    CrcLo,
    CrcHi,
    IdLo,
    IdHi,
    LenLo,
    LenHi,
    Body,
  };

  static constexpr std::size_t ID_FIELD_OFFSET = 0;
  static constexpr std::size_t LENGTH_FIELD_OFFSET = 2;
  static constexpr std::size_t CRC_INPUT_HEADER_BYTES = 4;

  std::optional<Packet> _enter_body() {
    const uint16_t length = read_little_endian<uint16_t>(
        {_header.data(), _header.size()}, LENGTH_FIELD_OFFSET);
    if (length < HEADER_LEN || length % LENGTH_ALIGNMENT != 0 ||
        length > MAX_BLOCK) {
      reset();
      return std::nullopt;
    }
    _body_length = static_cast<uint16_t>(length - HEADER_LEN);
    _body_index = 0;
    if (_body_length == 0) {
      return _verify();
    }
    _state = State::Body;
    return std::nullopt;
  }

  std::optional<Packet> _verify() {
    reset();
    uint16_t computed = crc_ccitt({_header.data(), _header.size()});
    computed = crc_ccitt({_buffer.data(), _body_length}, computed);
    if (computed != _expected_crc) {
      return std::nullopt;
    }
    Packet p;
    p.id = read_little_endian<uint16_t>({_header.data(), _header.size()},
                                        ID_FIELD_OFFSET);
    p.body_length = _body_length;
    for (std::size_t i = 0; i < _body_length; i++) {
      p.data[i] = _buffer[i];
    }
    return p;
  }

  State _state = State::Sync1;
  Ms _last_advance{0};
  uint8_t _crc_low_byte = 0;
  uint16_t _expected_crc = 0;
  std::array<uint8_t, CRC_INPUT_HEADER_BYTES> _header{};
  uint16_t _body_length = 0;
  uint16_t _body_index = 0;
  std::array<uint8_t, MAX_BODY> _buffer{};
};

} // namespace sbf
