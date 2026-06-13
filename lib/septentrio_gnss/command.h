/**
 * ASCII command builder and reply types for the Septentrio command line.
 *
 * Commands are short ASCII strings terminated with '\r' that configure the
 * receiver (e.g. "setSBFOutput,Stream1,COM1,PVTGeodetic,sec1"). Replies are
 * the receiver's response, prefixed with "$R:" (ok), "$R!" (formatted info),
 * or "$R?" (error), and terminated by the next "<port>>" prompt.
 *
 * Reply parsing happens in wire_parser.h; this file defines the command and
 * reply types and the typed builders that assemble specific commands.
 */

#pragma once

#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string_view>

namespace septentrio_gnss {

/*
 * Constants and Definitions.
 */

constexpr std::size_t MAX_COMMAND_LEN = 256;
constexpr std::size_t MAX_REPLY_BODY = 512;
constexpr char COMMAND_TERMINATOR = '\r';

constexpr char REPLY_MARKER_CHAR = 'R'; // second byte after '$' in a reply
constexpr char REPLY_KIND_OK = ':';
constexpr char REPLY_KIND_INFO = '!';
constexpr char REPLY_KIND_ERR = '?';

enum class ReplyKind : uint8_t {
  Ok,   // "$R:" - command echoed and accepted
  Info, // "$R!" - formatted info response
  Err,  // "$R?" - error
};

/*
 * Command structure.
 */

struct Command {
  std::array<char, MAX_COMMAND_LEN> data {};
  std::size_t length = 0;

  /**
   * @brief View of the wire bytes (body + COMMAND_TERMINATOR).
   */
  constexpr std::string_view view() const { return {data.data(), length}; }

  /**
   * @brief Build a command from a body string, appending COMMAND_TERMINATOR.
   *
   * @param body  ASCII command body, e.g. "setSBFOutput,Stream1,COM1,...".
   *              Must not exceed MAX_COMMAND_LEN - 1 bytes to leave room
   *              for the terminator.
   *
   * @return The ready-to-send Command, or an error string if the body is
   *         too long.
   */
  static constexpr std::expected<Command, const char *>
  build(std::string_view body) {
    if (body.size() >= MAX_COMMAND_LEN) {
      return std::unexpected("Command body exceeds MAX_COMMAND_LEN - 1");
    }
    Command c;
    for (std::size_t i = 0; i < body.size(); i++) {
      c.data[i] = body[i];
    }
    c.data[body.size()] = COMMAND_TERMINATOR;
    c.length = body.size() + 1;
    return c;
  }
};

/*
 * Reply structure.
 */

struct Reply {
  ReplyKind kind = ReplyKind::Ok;
  std::array<char, MAX_REPLY_BODY> data {};
  std::size_t length = 0;

  /**
   * @brief View of the reply body (excluding the "$R[:?!]" prefix and the
   *        trailing prompt).
   */
  constexpr std::string_view text() const { return {data.data(), length}; }
};

/*
 * Typed command builders.
 *
 * Tokens follow the mosaic-X5 command reference, e.g.
 * "setSBFOutput,Stream1,COM1,AttEuler+AttCovEuler,msec100". The string-only
 * builders are constexpr so framing can be static_asserted; set_attitude_offset
 * formats floats and is runtime-only.
 */

enum class GnssAttitudeMode : uint8_t { None, MovingBase };
enum class Connection : uint8_t { COM1, COM2, COM3, USB1, USB2 };
enum class SbfStream : uint8_t { Stream1, Stream2, Stream3, Stream4 };
enum class SbfInterval : uint8_t { Msec100, Msec200, Msec500, Sec1 };
enum class SbfBlock : uint8_t {
  AttEuler,
  AttCovEuler,
  PVTGeodetic,
  PosCovGeodetic,
};

constexpr std::string_view to_token(GnssAttitudeMode mode) {
  switch (mode) {
  case GnssAttitudeMode::None:
    return "none";
  case GnssAttitudeMode::MovingBase:
    return "MovingBase";
  }
  return {};
}

constexpr std::string_view to_token(Connection conn) {
  switch (conn) {
  case Connection::COM1:
    return "COM1";
  case Connection::COM2:
    return "COM2";
  case Connection::COM3:
    return "COM3";
  case Connection::USB1:
    return "USB1";
  case Connection::USB2:
    return "USB2";
  }
  return {};
}

constexpr std::string_view to_token(SbfStream stream) {
  switch (stream) {
  case SbfStream::Stream1:
    return "Stream1";
  case SbfStream::Stream2:
    return "Stream2";
  case SbfStream::Stream3:
    return "Stream3";
  case SbfStream::Stream4:
    return "Stream4";
  }
  return {};
}

constexpr std::string_view to_token(SbfInterval interval) {
  switch (interval) {
  case SbfInterval::Msec100:
    return "msec100";
  case SbfInterval::Msec200:
    return "msec200";
  case SbfInterval::Msec500:
    return "msec500";
  case SbfInterval::Sec1:
    return "sec1";
  }
  return {};
}

constexpr std::string_view to_token(SbfBlock block) {
  switch (block) {
  case SbfBlock::AttEuler:
    return "AttEuler";
  case SbfBlock::AttCovEuler:
    return "AttCovEuler";
  case SbfBlock::PVTGeodetic:
    return "PVTGeodetic";
  case SbfBlock::PosCovGeodetic:
    return "PosCovGeodetic";
  }
  return {};
}

/*
 * Fixed-capacity body assembler. Appends tokens into a stack buffer and flags
 * overflow instead of writing past the end; finish() turns the buffer into a
 * terminated Command.
 */
struct CommandBuilder {
  std::array<char, MAX_COMMAND_LEN> buffer {};
  std::size_t length = 0;
  bool overflowed = false;

  constexpr void put(char c) {
    if (length >= buffer.size()) {
      overflowed = true;
      return;
    }
    buffer[length++] = c;
  }

  constexpr void put(std::string_view s) {
    for (char c : s) {
      put(c);
    }
  }

  void put(float value) {
    std::array<char, 32> tmp {};
    const auto result =
        std::to_chars(tmp.data(), tmp.data() + tmp.size(), value);
    put(std::string_view(tmp.data(), result.ptr - tmp.data()));
  }

  constexpr std::expected<Command, const char *> finish() const {
    if (overflowed) {
      return std::unexpected("Command body exceeds MAX_COMMAND_LEN - 1");
    }
    return Command::build({buffer.data(), length});
  }
};

/**
 * @brief Build "setGNSSAttitude,<mode>", selecting the attitude source.
 */
constexpr std::expected<Command, const char *>
set_gnss_attitude(GnssAttitudeMode mode) {
  CommandBuilder b;
  b.put("setGNSSAttitude,");
  b.put(to_token(mode));
  return b.finish();
}

/**
 * @brief Build "setSBFOutput,<stream>,<conn>,<blocks>,<interval>".
 *
 * @param blocks One or more SBF blocks, emitted '+'-joined.
 */
constexpr std::expected<Command, const char *>
set_sbf_output(SbfStream stream,
               Connection conn,
               std::span<const SbfBlock> blocks,
               SbfInterval interval) {
  CommandBuilder b;
  b.put("setSBFOutput,");
  b.put(to_token(stream));
  b.put(',');
  b.put(to_token(conn));
  b.put(',');
  for (std::size_t i = 0; i < blocks.size(); i++) {
    if (i != 0) {
      b.put('+');
    }
    b.put(to_token(blocks[i]));
  }
  b.put(',');
  b.put(to_token(interval));
  return b.finish();
}

/**
 * @brief Build "setAttitudeOffset,<heading>,<pitch>" in degrees.
 *
 * Left at zero when the baseline-to-boat offset is applied in the fusion
 * bridge instead of on the receiver.
 */
inline std::expected<Command, const char *>
set_attitude_offset(float heading_deg, float pitch_deg) {
  CommandBuilder b;
  b.put("setAttitudeOffset,");
  b.put(heading_deg);
  b.put(',');
  b.put(pitch_deg);
  return b.finish();
}

} // namespace septentrio_gnss
