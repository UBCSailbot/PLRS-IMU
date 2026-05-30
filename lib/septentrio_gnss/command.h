/**
 * ASCII command builder and reply types for the Septentrio command line.
 *
 * Commands are short ASCII strings terminated with '\r' that configure the
 * receiver (e.g. "setSBFOutput,Stream1,COM1,PVTGeodetic,sec1"). Replies are
 * the receiver's response, prefixed with "$R:" (ok), "$R!" (formatted info),
 * or "$R?" (error), and terminated by the next "<port>>" prompt.
 *
 * Reply parsing happens in wire_parser.h; this file just defines the types.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
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
  std::array<char, MAX_COMMAND_LEN> data{};
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
  std::array<char, MAX_REPLY_BODY> data{};
  std::size_t length = 0;

  /**
   * @brief View of the reply body (excluding the "$R[:?!]" prefix and the
   *        trailing prompt).
   */
  constexpr std::string_view text() const { return {data.data(), length}; }
};

} // namespace septentrio_gnss
