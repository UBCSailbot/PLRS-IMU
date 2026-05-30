/**
 * Wire-level parser for the Septentrio mosaic-go-H UART.
 *
 * SBF blocks and NMEA sentences share the same leading byte '$' (0x24).
 * The byte after '$' disambiguates: '@' (0x40) starts an SBF block,
 * anything else starts an NMEA sentence.
 */

#pragma once

#include "command.h"
#include "nmea_protocol.h"
#include "sbf_protocol.h"
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <variant>

namespace septentrio_gnss {

using Ms = std::chrono::milliseconds;
using Message = std::variant<sbf::Packet, nmea::Sentence, Reply>;

static_assert(sbf::SYNC1 == nmea::START,
              "Wire dispatch assumes SBF SYNC1 == NMEA START");

class Parser {
public:
  /**
   * Mid-frame timeout. Bytes arriving more than FRAME_TIMEOUT after the
   * previous byte cause the partial frame to be dropped before processing.
   */
  static constexpr Ms FRAME_TIMEOUT{250};

  /**
   * @brief Reset the parser and drop the current frame.
   */
  void reset() {
    _state = State::Idle;
    _sbf_body_length = 0;
    _sbf_body_index = 0;
    _sbf_expected_crc = 0;
    _sbf_crc_low_byte = 0;
    _sbf_header.fill(0);
    _nmea_length = 0;
    _nmea_running_xor = 0;
    _nmea_expected_xor = 0;
    _reply_kind = ReplyKind::Ok;
    _reply_length = 0;
    _reply_in_prompt_candidate = false;
    _reply_body_end_before_prompt = 0;
  }

  /**
   * @brief Check if the parser is mid frame.
   *
   * @return true if the parser is mid frame.
   */
  bool mid_frame() const { return _state != State::Idle; }

  /**
   * @brief Feed one byte into the parser.
   *
   * @param byte. The current byte.
   * @param now. The current time.
   *
   * @return A completed message (SBF Packet or NMEA Sentence), or nullopt
   * if the frame isn't done. If a frame exceeds FRAME_TIMEOUT then that
   * frame is dropped and the next frame is returned.
   */
  std::optional<Message> feed(uint8_t byte, Ms now) {
    if (_state != State::Idle && (now - _last_advance) > FRAME_TIMEOUT) {
      reset();
    }
    _last_advance = now;

    switch (_state) {
    case State::Idle:
      if (byte == sbf::SYNC1) {
        _state = State::AfterDollar;
      }
      break;
    case State::AfterDollar:
      if (byte == sbf::SYNC2) {
        _state = State::SbfCrcLo;
      } else if (byte == sbf::SYNC1) {
        // $$... stay in AfterDollar; the second '$' might still be the real
        // start
      } else if (byte == REPLY_MARKER_CHAR) {
        _state = State::AfterR;
      } else {
        // Start NMEA. The byte is the first body character.
        _nmea_length = 0;
        _nmea_running_xor = nmea::XOR_INIT;
        return _nmea_consume_body_byte(byte);
      }
      break;
    case State::AfterR:
      if (byte == REPLY_KIND_OK || byte == REPLY_KIND_INFO ||
          byte == REPLY_KIND_ERR) {
        _reply_kind = (byte == REPLY_KIND_OK)     ? ReplyKind::Ok
                      : (byte == REPLY_KIND_INFO) ? ReplyKind::Info
                                                  : ReplyKind::Err;
        _reply_length = 0;
        _reply_in_prompt_candidate = false;
        _reply_body_end_before_prompt = 0;
        _state = State::ReplyBody;
      } else {
        // Not a reply; treat as NMEA starting with 'R' then this byte.
        _nmea_length = 0;
        _nmea_running_xor = nmea::XOR_INIT;
        _nmea_running_xor ^= static_cast<uint8_t>(REPLY_MARKER_CHAR);
        _nmea_buffer[_nmea_length++] = REPLY_MARKER_CHAR;
        return _nmea_consume_body_byte(byte);
      }
      break;
    case State::ReplyBody:
      return _reply_consume_body_byte(byte);

    /*
     * SBF states.
     */
    case State::SbfCrcLo:
      _sbf_crc_low_byte = byte;
      _state = State::SbfCrcHi;
      break;
    case State::SbfCrcHi:
      _sbf_expected_crc = static_cast<uint16_t>((byte << sbf::BITS_PER_BYTE) |
                                                _sbf_crc_low_byte);
      _state = State::SbfIdLo;
      break;
    case State::SbfIdLo:
      _sbf_header[SBF_ID_OFFSET] = byte;
      _state = State::SbfIdHi;
      break;
    case State::SbfIdHi:
      _sbf_header[SBF_ID_OFFSET + 1] = byte;
      _state = State::SbfLenLo;
      break;
    case State::SbfLenLo:
      _sbf_header[SBF_LENGTH_OFFSET] = byte;
      _state = State::SbfLenHi;
      break;
    case State::SbfLenHi:
      _sbf_header[SBF_LENGTH_OFFSET + 1] = byte;
      return _sbf_enter_body();
    case State::SbfBody:
      _sbf_buffer[_sbf_body_index++] = byte;
      if (_sbf_body_index >= _sbf_body_length) {
        return _sbf_verify();
      }
      break;

    /*
     * NMEA states.
     */
    case State::NmeaBody:
      return _nmea_consume_body_byte(byte);
    case State::NmeaChecksumHi: {
      const int n = nmea::hex_to_nibble(byte);
      if (n < 0) {
        reset();
        break;
      }
      _nmea_expected_xor = static_cast<uint8_t>(n << nmea::NIBBLE_BITS);
      _state = State::NmeaChecksumLo;
      break;
    }
    case State::NmeaChecksumLo: {
      const int n = nmea::hex_to_nibble(byte);
      if (n < 0) {
        reset();
        break;
      }
      _nmea_expected_xor = static_cast<uint8_t>(_nmea_expected_xor | n);
      return _nmea_verify();
    }
    }
    return std::nullopt;
  }

private:
  enum class State : uint8_t {
    Idle,
    AfterDollar,
    AfterR,
    SbfCrcLo,
    SbfCrcHi,
    SbfIdLo,
    SbfIdHi,
    SbfLenLo,
    SbfLenHi,
    SbfBody,
    NmeaBody,
    NmeaChecksumHi,
    NmeaChecksumLo,
    ReplyBody,
  };

  static constexpr std::size_t SBF_ID_OFFSET = 0;
  static constexpr std::size_t SBF_LENGTH_OFFSET = 2;
  static constexpr std::size_t SBF_CRC_INPUT_HEADER_BYTES = 4;

  std::optional<Message> _sbf_enter_body() {
    const uint16_t length = sbf::read_little_endian<uint16_t>(
        {_sbf_header.data(), _sbf_header.size()}, SBF_LENGTH_OFFSET);
    if (length < sbf::HEADER_LEN || length % sbf::LENGTH_ALIGNMENT != 0 ||
        length > sbf::MAX_BLOCK) {
      reset();
      return std::nullopt;
    }
    _sbf_body_length = static_cast<uint16_t>(length - sbf::HEADER_LEN);
    _sbf_body_index = 0;
    if (_sbf_body_length == 0) {
      return _sbf_verify();
    }
    _state = State::SbfBody;
    return std::nullopt;
  }

  std::optional<Message> _sbf_verify() {
    const uint16_t expected = _sbf_expected_crc;
    const uint16_t body_length = _sbf_body_length;
    uint16_t computed =
        sbf::crc_ccitt({_sbf_header.data(), _sbf_header.size()});
    computed = sbf::crc_ccitt({_sbf_buffer.data(), body_length}, computed);
    sbf::Packet p;
    if (computed == expected) {
      p.id = sbf::read_little_endian<uint16_t>(
          {_sbf_header.data(), _sbf_header.size()}, SBF_ID_OFFSET);
      p.body_length = body_length;
      for (std::size_t i = 0; i < body_length; i++) {
        p.data[i] = _sbf_buffer[i];
      }
    }
    reset();
    if (computed != expected) {
      return std::nullopt;
    }
    return Message{p};
  }

  std::optional<Message> _nmea_consume_body_byte(uint8_t byte) {
    if (byte == sbf::SYNC1) {
      // Mid-line stray '$' restarts NMEA parsing from this byte's successor.
      _nmea_length = 0;
      _nmea_running_xor = nmea::XOR_INIT;
      _state = State::AfterDollar;
      return std::nullopt;
    }
    if (byte == nmea::CHECKSUM_DELIMITER) {
      _state = State::NmeaChecksumHi;
      return std::nullopt;
    }
    if (byte == nmea::CR || byte == nmea::LF) {
      // Terminator without checksum: malformed, drop.
      reset();
      return std::nullopt;
    }
    if (_nmea_length >= nmea::MAX_BODY) {
      reset();
      return std::nullopt;
    }
    _nmea_running_xor ^= byte;
    _nmea_buffer[_nmea_length++] = static_cast<char>(byte);
    _state = State::NmeaBody;
    return std::nullopt;
  }

  std::optional<Message> _nmea_verify() {
    nmea::Sentence s;
    const bool ok = _nmea_expected_xor == _nmea_running_xor;
    if (ok) {
      for (std::size_t i = 0; i < _nmea_length; i++) {
        s.data[i] = _nmea_buffer[i];
      }
      s.length = _nmea_length;
    }
    reset();
    if (!ok) {
      return std::nullopt;
    }
    return Message{s};
  }

  static constexpr bool is_prompt_char(uint8_t c) {
    return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
  }

  std::optional<Message> _reply_consume_body_byte(uint8_t byte) {
    // The prompt terminator is "\n<PORT>>" where <PORT> is letters/digits.
    // After a '\n', enter prompt-candidate mode and tentatively collect
    // prompt chars; if '>' arrives we strip them and deliver. Anything
    // non-prompt cancels the candidate.
    if (byte == '\n') {
      if (_reply_length >= MAX_REPLY_BODY) {
        reset();
        return std::nullopt;
      }
      _reply_data_buffer[_reply_length++] = static_cast<char>(byte);
      _reply_in_prompt_candidate = true;
      _reply_body_end_before_prompt = _reply_length;
      return std::nullopt;
    }
    if (_reply_in_prompt_candidate) {
      if (byte == '>' && _reply_length > _reply_body_end_before_prompt) {
        _reply_length = _reply_body_end_before_prompt;
        return _reply_finish();
      }
      if (is_prompt_char(byte)) {
        if (_reply_length >= MAX_REPLY_BODY) {
          reset();
          return std::nullopt;
        }
        _reply_data_buffer[_reply_length++] = static_cast<char>(byte);
        return std::nullopt;
      }
      _reply_in_prompt_candidate = false;
    }
    if (_reply_length >= MAX_REPLY_BODY) {
      reset();
      return std::nullopt;
    }
    _reply_data_buffer[_reply_length++] = static_cast<char>(byte);
    return std::nullopt;
  }

  std::optional<Message> _reply_finish() {
    Reply r;
    r.kind = _reply_kind;
    for (std::size_t i = 0; i < _reply_length; i++) {
      r.data[i] = _reply_data_buffer[i];
    }
    r.length = _reply_length;
    reset();
    return Message{r};
  }

  State _state = State::Idle;
  Ms _last_advance{0};

  // SBF state.
  uint8_t _sbf_crc_low_byte = 0;
  uint16_t _sbf_expected_crc = 0;
  std::array<uint8_t, SBF_CRC_INPUT_HEADER_BYTES> _sbf_header{};
  uint16_t _sbf_body_length = 0;
  uint16_t _sbf_body_index = 0;
  std::array<uint8_t, sbf::MAX_BODY> _sbf_buffer{};

  // NMEA state.
  std::size_t _nmea_length = 0;
  uint8_t _nmea_running_xor = 0;
  uint8_t _nmea_expected_xor = 0;
  std::array<char, nmea::MAX_BODY> _nmea_buffer{};

  // Reply state.
  ReplyKind _reply_kind = ReplyKind::Ok;
  std::size_t _reply_length = 0;
  bool _reply_in_prompt_candidate = false;
  std::size_t _reply_body_end_before_prompt = 0;
  std::array<char, MAX_REPLY_BODY> _reply_data_buffer{};
};

} // namespace septentrio_gnss
