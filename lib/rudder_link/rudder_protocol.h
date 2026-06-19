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
#include <expected>
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

  constexpr ByteSpan view() const PLRS_LIFETIMEBOUND {
    return {bytes.data(), len};
  }
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

  constexpr ByteSpan view() const PLRS_LIFETIMEBOUND {
    return {bytes.data(), len};
  }
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

/**
 * @brief Frame a message: header, payload, CRC, then COBS.
 *
 * @param id  Message type.
 * @param seq  Sequence number.
 * @param payload  Message body; at most MAX_PAYLOAD.
 *
 * @return The framed bytes ready to send, or nullopt if payload exceeds
 *   MAX_PAYLOAD.
 */
constexpr std::optional<Encoded>
encode(MsgId id, uint8_t seq, ByteSpan payload) {
  if (payload.size() > MAX_PAYLOAD) {
    return std::nullopt;
  }

  std::array<uint8_t, MAX_DATA> body {};
  std::size_t n = 0;
  body[n++] = PROTOCOL_VERSION;
  body[n++] = static_cast<uint8_t>(id);
  body[n++] = seq;
  for (uint8_t b : payload) {
    body[n++] = b;
  }

  const auto crc = plrs::write_u16_little_endian(crc16({body.data(), n}));
  body[n++] = crc[0];
  body[n++] = crc[1];

  return cobs_encode({body.data(), n});
}

/*
 * Messages.
 */

/**
 * Heading message: compass heading in degrees (see docs/attitude.md).
 *
 * Each message type follows this shape: a static ID, a to_payload() that
 * serializes its body, and a from_payload() that parses one back.
 */
struct Heading {
  static constexpr MsgId ID = MsgId::Heading;
  float deg;

  constexpr std::array<uint8_t, sizeof(float)> to_payload() const {
    return plrs::write_f32_little_endian(deg);
  }

  static constexpr std::optional<Heading> from_payload(ByteSpan payload) {
    if (payload.size() != sizeof(float)) {
      return std::nullopt;
    }
    return Heading {.deg = plrs::read_f32_little_endian(payload)};
  }
};

/**
 * @brief Frame a typed message: header, payload, CRC, then COBS.
 *
 * @param seq  Sequence number.
 * @param msg  A message with a static ID and a to_payload().
 *
 * @return The framed bytes ready to send.
 */
template <class M> constexpr Encoded encode(uint8_t seq, const M &msg) {
  return *encode(M::ID, seq, msg.to_payload());
}

/*
 * Receiving.
 */

/**
 * Why a completed frame was rejected.
 */
enum class Error : uint8_t {
  Malformed,    // COBS decode failed, or the frame is shorter than a header+CRC
  WrongVersion, // ver byte does not match PROTOCOL_VERSION
  BadCrc,       // CRC mismatch
};

/**
 * A parsed frame. payload borrows the parser's buffer => valid only until the
 * next feed().
 */
struct Frame {
  MsgId id;
  uint8_t seq;
  ByteSpan payload;
};

/**
 * Accumulates bytes up to the COBS delimiter, then decodes, version-checks, and
 * CRC-checks one frame.
 */
class Parser {
public:
  /**
   * Drop a partial frame whose delimiter has not arrived within this long. COBS
   * resyncs at the next delimiter regardless; this only bounds staleness.
   */
  static constexpr Ms FRAME_TIMEOUT {50};

  /**
   * @brief Reset the parser and drop the current frame.
   */
  void reset() {
    _len = 0;
    _overflow = false;
  }

  /**
   * @brief Check if the parser is mid frame.
   *
   * @return true if bytes have accumulated since the last delimiter.
   */
  bool mid_frame() const { return _len > 0; }

  /**
   * @brief Feed one byte into the parser.
   *
   * @param byte  The current byte.
   * @param now  The current time.
   *
   * @return nullopt while the frame is still arriving; otherwise a completed
   *   frame, or the Error explaining why a completed frame was rejected.
   */
  std::optional<std::expected<Frame, Error>> feed(uint8_t byte, Ms now) {
    if (_len > 0 && (now - _last_advance) > FRAME_TIMEOUT) {
      reset();
    }
    _last_advance = now;

    if (byte != FRAME_DELIMITER) {
      if (_len < _block.size()) {
        _block[_len++] = byte;
      } else {
        _overflow = true;
      }
      return std::nullopt;
    }

    const bool overflowed = _overflow;
    const std::size_t n = _len;
    reset();
    if (n == 0) {
      return std::nullopt;
    }
    if (overflowed) {
      return std::expected<Frame, Error> {std::unexpected(Error::Malformed)};
    }
    return parse({_block.data(), n});
  }

private:
  std::expected<Frame, Error> parse(ByteSpan block) {
    auto decoded = cobs_decode(block);
    if (!decoded) {
      return std::unexpected(Error::Malformed);
    }
    _decoded = *decoded;

    ByteSpan body = _decoded.view();
    if (body.size() < HEADER_BYTES + CRC_BYTES) {
      return std::unexpected(Error::Malformed);
    }
    if (body[0] != PROTOCOL_VERSION) {
      return std::unexpected(Error::WrongVersion);
    }

    const std::size_t crc_at = body.size() - CRC_BYTES;
    const uint16_t want = crc16({body.data(), crc_at});
    const uint16_t got =
        plrs::read_u16_little_endian(body.subspan(crc_at, CRC_BYTES));
    if (want != got) {
      return std::unexpected(Error::BadCrc);
    }

    return Frame {
        .id = static_cast<MsgId>(body[1]),
        .seq = body[2],
        .payload = body.subspan(HEADER_BYTES, crc_at - HEADER_BYTES),
    };
  }

  Ms _last_advance {0};
  std::size_t _len = 0;
  bool _overflow = false;
  std::array<uint8_t, MAX_FRAME> _block {};
  Decoded _decoded {};
};

} // namespace rudder
