/**
 * Communication Library for XSENS' Xbus protocol
 *
 * Note that all messages are big endian.
 */

#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>

namespace xbus {

/*
 * Constants and Definitions.
 */

constexpr uint8_t PREAMBLE = 0xfa;
constexpr uint8_t LEN_EXTENDED = 0xff;
constexpr uint8_t BID_MASTER = 0xff;     // host/master bus address in Xbus
constexpr std::size_t MAX_PAYLOAD = 254; // for standard messages
constexpr std::size_t MAX_FRAME = MAX_PAYLOAD + 5;

using ByteSpan = std::span<const uint8_t>;
using Ms = std::chrono::milliseconds;

/**
 * Message IDs.
 */
enum class MID : uint8_t {
  WakeUp = 0x3e,
  WakeUpAck = 0x3f,
  GoToConfig = 0x30,
  GoToConfigAck = 0x31,
  GoToMeasurement = 0x10,
  GoToMeasAck = 0x11,
  Reset = 0x40,
  SetOptionFlags = 0x48,
  ReqOutputConfig = 0xc0,
  SetOutputConfig = 0xc0, // same message id, if len = 0 => "request"
  OutputConfigAck = 0xc1,
  MTData2 = 0x36,
  Error = 0x42,
};

/**
 * Data IDs.
 * Low nibble is the FORMAT (precision | coord system).
 * 0x0 precision = float32,
 * 0x0 coord = ENU.
 */
enum class DataId : uint16_t {
  PacketCounter = 0x1020,
  SampleTimeFine = 0x1060,
  Quaternion = 0x2010,
  Acceleration = 0x4020,
  FreeAccel = 0x4030,
  RateOfTurn = 0x8020,
  MagneticField = 0xc020,
  StatusWord = 0xe020,
};

/*
 * Messages and Validation.
 */

/**
 * Message structure.
 */
struct Packet {
  uint8_t bid = BID_MASTER;
  MID mid = MID::Error;
  uint8_t len = 0;
  std::array<uint8_t, MAX_PAYLOAD> data{};

  /**
   * @brief Return a copy of data.
   *
   * Avoids mutability issues.
   *
   * @return a copy of data.
   */
  constexpr ByteSpan payload() const { return {data.data(), len}; };

  /**
   * @brief Construct a packet to send.
   *
   * Do not put in payloads greater than the standard message maximum length
   * (254 bytes).
   *
   * @param mid  Message ID identifying the command type.
   * @param payload  Raw payload bytes. Must not exceed MAX_PAYLOAD (254 bytes).
   * @param bid  Bus ID of the target device. Defaults to BID_MASTER.
   *
   * @return A packet with all extra fields filled in, or an error string if
   *         the payload exceeds MAX_PAYLOAD.
   */
  static constexpr std::expected<Packet, const char *>
  command(MID mid, ByteSpan payload, uint8_t bid = BID_MASTER) {
    if (payload.size_bytes() > MAX_PAYLOAD) {
      return std::unexpected("Payload exceeds 254 bytes");
    };

    Packet p;
    p.bid = bid;
    p.mid = mid;
    p.len = static_cast<uint8_t>(payload.size());
    for (std::size_t i = 0; i < payload.size(); i++) {
      p.data[i] = payload[i];
    };

    return p;
  }
};

/**
 * @brief Compute the two's complement of a number.
 *
 * @param num: The number to be complemented.
 *
 * @return The two's complement of num.
 */
constexpr uint8_t twosComplement(const uint8_t num) {
  return static_cast<uint8_t>(0x100 - num);
}

/**
 * @brief Compute the checksum of a standard xbus message.
 *
 * @param bid: Bus ID.
 * @param mid: Message ID.
 * @param len: Message length.
 * @param *data: Message pointer.
 *
 * @return The sum of all message bytes excluding the preamble.
 *         If the lower byte value of the result is zero, the result is valid.
 */
constexpr uint8_t checksum(uint8_t bid, uint8_t mid,
                           std::span<const uint8_t> data) {
  uint8_t sum = bid + mid + static_cast<uint8_t>(data.size());
  for (const uint8_t byte : data) {
    sum += byte;
  };

  return twosComplement(sum);
}

/*
 * Sending.
 */

/**
 * Serialized form of a packet ready to send.
 */
struct Encoded {
  std::array<uint8_t, MAX_FRAME> bytes{};
  std::size_t len = 0;

  /**
   * @brief Return a copy of data.
   *
   * Avoids mutability issues.
   *
   * @return a copy of data.
   */
  constexpr ByteSpan view() const { return {bytes.data(), len}; }
};

/**
 * @brief Encode a packet to send.
 *
 * @param packet: The packet to encode.
 *
 * @return An optional packet, where a nullopt is return if the payload exceeds
 * the standard length cap.
 */
constexpr std::optional<Encoded> encode(const Packet &packet) {
  if (packet.len > MAX_PAYLOAD) {
    return std::nullopt;
  };

  Encoded e;
  std::size_t i = 0;
  e.bytes[i++] = PREAMBLE;
  e.bytes[i++] = packet.bid;
  e.bytes[i++] = static_cast<uint8_t>(packet.mid);
  e.bytes[i++] = packet.len;
  for (uint8_t byte : packet.payload()) {
    e.bytes[i++] = byte;
  };
  e.bytes[i++] =
      checksum(packet.bid, static_cast<uint8_t>(packet.mid), packet.payload());
  e.len = i;
  return e;
}

/*
 * Recieving.
 */

/**
 * Parser class.
 *
 * Deals with parsing incoming xbus messages. Standard messages only for now.
 */
class Parser {
public:
  /**
   * The value at which a packet is considered to be corrupted or lost and it is
   * abandoned to parse the next full frame.
   *
   * At 115200 baud a full 254-byte frame takes 22ms.
   */
  static constexpr Ms FRAME_TIMEOUT{50};

  /**
   * @brief Reset the parser and drop the current frame.
   */
  void reset() { _state = State::Preamble; }

  /**
   * @brief Check if the parser is mid frame.
   *
   * @return true if the parser is mid frame.
   */
  bool mid_frame() const { return _state != State::Preamble; }

  /**
   * @brief Feed one byte into the parser.
   *
   * @param byte. The current byte.
   * @param now. The current time.
   *
   * @return A completed packet, or nullopt if the frame isn't done. If a frame
   * exeeds FRAME_TIMEOUT then that frame is dropped and the next frame is
   * returned.
   */
  std::optional<Packet> feed(uint8_t byte, Ms now) {
    if (_state != State::Preamble && (now - _last_advance) > FRAME_TIMEOUT) {
      reset();
    }
    _last_advance = now;

    switch (_state) {
    case State::Preamble:
      if (byte == PREAMBLE)
        _state = State::Bid;
      break;
    case State::Bid:
      _bid = byte;
      _sum = byte;
      _state = State::Mid;
      break;
    case State::Mid:
      _mid = byte;
      _sum += byte;
      _state = State::Len;
      break;
    case State::Len:
      _length = byte;
      _sum += byte;
      _index = 0;
      // TODO: add support for extended length messages.
      if (byte == LEN_EXTENDED) {
        reset();
        break;
      }
      _state = (byte > 0) ? State::Data : State::Check;
      break;
    case State::Data:
      _buffer[_index++] = byte;
      _sum += byte;
      if (_index >= _length) {
        _state = State::Check;
      }
      break;
    case State::Check:
      reset();
      if (static_cast<uint8_t>(_sum + byte) == 0) {
        Packet p;
        p.bid = _bid;
        p.mid = static_cast<MID>(_mid);
        p.len = _length;
        for (uint8_t i = 0; i < _length; i++) {
          p.data[i] = _buffer[i];
        }
        return p;
      }
      break;
    }
    return std::nullopt;
  };

private:
  /**
   * States that the parser can be in.
   */
  enum class State : uint8_t {
    Preamble,
    Bid,
    Mid,
    Len,
    Data,
    Check,
  };

  State _state = State::Preamble;
  Ms _last_advance{0};
  uint8_t _bid = 0;
  uint8_t _mid = 0;
  uint8_t _length = 0;
  uint8_t _index = 0;
  uint8_t _sum = 0;
  std::array<uint8_t, MAX_PAYLOAD> _buffer{};
};

} // namespace xbus
