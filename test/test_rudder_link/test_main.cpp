/**
 * Host (native) unit tests for the rudder link protocol core.
 */

#include "rudder_protocol.h"
#include <unity.h>
#include <vector>

using namespace rudder;

// ---------------------------------------------------------------------------
// Compile-time regression tests.
// ---------------------------------------------------------------------------
namespace {
// CRC-16/CCITT-FALSE check value: crc16("123456789") == 0x29B1.
constexpr std::array<uint8_t, 9> kCheck {
    '1', '2', '3', '4', '5', '6', '7', '8', '9'};
static_assert(crc16(kCheck) == 0x29B1);
static_assert(crc16(std::array<uint8_t, 0> {}) == CRC16_INIT);

// COBS of 11 22 00 33 is 03 11 22 02 33 00 (the docs/rudder_link.md example).
constexpr std::array<uint8_t, 4> kCobsIn {0x11, 0x22, 0x00, 0x33};
constexpr auto kCobsOut = cobs_encode(kCobsIn);
static_assert(kCobsOut.has_value());
static_assert(kCobsOut->len == 6);
static_assert(kCobsOut->bytes[0] == 0x03);
static_assert(kCobsOut->bytes[3] == 0x02);
static_assert(kCobsOut->bytes[5] == FRAME_DELIMITER);

// A typed message is encodable in a constant expression.
constexpr auto kHeadingFrame = encode(1, Heading {.deg = 90.0f});
static_assert(kHeadingFrame.len > 0);

constexpr auto kAttitudeFrame = encode(1,
                                       Attitude {.heading_deg = 90.0f,
                                                 .roll_deg = 5.0f,
                                                 .pitch_deg = -2.0f,
                                                 .yaw_rate_dps = 3.0f});
static_assert(kAttitudeFrame.len > 0);

// Decode one COBS block (delimiter stripped) into the encoded body. Returns the
// decoded bytes, or an empty vector on failure.
std::vector<uint8_t> decode_block(ByteSpan encoded_with_delimiter) {
  ByteSpan block {encoded_with_delimiter.data(),
                  encoded_with_delimiter.size() - 1};
  auto d = cobs_decode(block);
  if (!d) {
    return {};
  }
  return {d->bytes.begin(), d->bytes.begin() + d->len};
}

// A parsed frame with the borrowed payload copied out, so it survives later
// feed() calls.
struct Got {
  MsgId id;
  uint8_t seq;
  std::vector<uint8_t> payload;
};

// Feed every byte and collect one verdict per completed frame (skipping the
// nullopt "still arriving" returns).
std::vector<std::expected<Got, Error>>
feed_all(Parser &p, ByteSpan bytes, Ms t = Ms {0}) {
  std::vector<std::expected<Got, Error>> out;
  for (uint8_t b : bytes) {
    auto r = p.feed(b, t);
    if (!r) {
      continue;
    }
    if (r->has_value()) {
      const Frame &f = r->value();
      out.push_back(Got {f.id, f.seq, {f.payload.begin(), f.payload.end()}});
    } else {
      out.push_back(std::unexpected(r->error()));
    }
  }
  return out;
}
} // namespace

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// crc16()
// ---------------------------------------------------------------------------

/** @brief CCITT-FALSE standard check value. */
void test_crc16_known_answer() {
  TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16(kCheck));
}

/** @brief Empty input returns the init value. */
void test_crc16_empty() {
  TEST_ASSERT_EQUAL_HEX16(CRC16_INIT, crc16(std::array<uint8_t, 0> {}));
}

// ---------------------------------------------------------------------------
// cobs_encode()
// ---------------------------------------------------------------------------

/** @brief Matches the worked example in the protocol docs. */
void test_cobs_encode_example() {
  auto e = cobs_encode(kCobsIn);
  TEST_ASSERT_TRUE(e.has_value());
  const std::vector<uint8_t> want {0x03, 0x11, 0x22, 0x02, 0x33, 0x00};
  TEST_ASSERT_EQUAL_size_t(want.size(), e->len);
  for (std::size_t i = 0; i < want.size(); i++) {
    TEST_ASSERT_EQUAL_HEX8(want[i], e->bytes[i]);
  }
}

/** @brief The encoded block never contains the delimiter except at the end. */
void test_cobs_encode_no_zero_in_block() {
  const std::vector<std::vector<uint8_t>> cases {
      {0x00},
      {0x00, 0x00, 0x00},
      {0x01, 0x00, 0x02, 0x00, 0x03},
      std::vector<uint8_t>(MAX_DATA, 0x00)};
  for (const auto &in : cases) {
    auto e = cobs_encode(in);
    TEST_ASSERT_TRUE(e.has_value());
    for (std::size_t i = 0; i + 1 < e->len; i++) {
      TEST_ASSERT_NOT_EQUAL(FRAME_DELIMITER, e->bytes[i]);
    }
    TEST_ASSERT_EQUAL_HEX8(FRAME_DELIMITER, e->bytes[e->len - 1]);
  }
}

/** @brief A payload larger than MAX_DATA is rejected. */
void test_cobs_encode_oversize_rejected() {
  std::vector<uint8_t> big(MAX_DATA + 1, 0xAB);
  TEST_ASSERT_FALSE(cobs_encode(big).has_value());
}

// ---------------------------------------------------------------------------
// cobs_decode()
// ---------------------------------------------------------------------------

/** @brief Encode then decode reproduces the input across edge cases. */
void test_cobs_roundtrip() {
  const std::vector<std::vector<uint8_t>> cases {
      {},
      {0x00},
      {0x11, 0x22, 0x00, 0x33},
      {0x00, 0x00, 0x00},
      {0x01, 0x02, 0x03, 0x04, 0x05},
      std::vector<uint8_t>(MAX_DATA, 0xAB),
      std::vector<uint8_t>(MAX_DATA, 0x00)};
  for (const auto &in : cases) {
    auto e = cobs_encode(in);
    TEST_ASSERT_TRUE(e.has_value());
    TEST_ASSERT_EQUAL_size_t(in.size(), decode_block(e->view()).size());
    TEST_ASSERT_TRUE(in == decode_block(e->view()));
  }
}

/** @brief A zero inside a block is malformed and rejected. */
void test_cobs_decode_rejects_zero_in_block() {
  const std::array<uint8_t, 3> bad {0x02, 0x00, 0x11};
  TEST_ASSERT_FALSE(cobs_decode(bad).has_value());
}

/** @brief A code byte pointing past the end is rejected. */
void test_cobs_decode_rejects_truncated() {
  const std::array<uint8_t, 2> bad {0x05, 0x11};
  TEST_ASSERT_FALSE(cobs_decode(bad).has_value());
}

// ---------------------------------------------------------------------------
// encode()
// ---------------------------------------------------------------------------

/** @brief A framed message decodes to ver, msg_id, seq, payload, valid CRC. */
void test_encode_frame_layout() {
  const std::vector<uint8_t> payload {0xDE, 0xAD, 0xBE, 0xEF};
  auto e = encode(MsgId::Heading, 0x2A, payload);
  TEST_ASSERT_TRUE(e.has_value());

  const auto body = decode_block(e->view());
  TEST_ASSERT_EQUAL_HEX8(PROTOCOL_VERSION, body[0]);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(MsgId::Heading), body[1]);
  TEST_ASSERT_EQUAL_HEX8(0x2A, body[2]);
  for (std::size_t i = 0; i < payload.size(); i++) {
    TEST_ASSERT_EQUAL_HEX8(payload[i], body[HEADER_BYTES + i]);
  }

  const std::size_t crc_at = body.size() - CRC_BYTES;
  const uint16_t want = crc16({body.data(), crc_at});
  const uint16_t got =
      plrs::read_u16_little_endian({body.data() + crc_at, CRC_BYTES});
  TEST_ASSERT_EQUAL_HEX16(want, got);
}

/** @brief A payload larger than MAX_PAYLOAD is rejected. */
void test_encode_oversize_rejected() {
  std::vector<uint8_t> big(MAX_PAYLOAD + 1, 0x55);
  TEST_ASSERT_FALSE(encode(MsgId::Heading, 0, big).has_value());
}

/** @brief A Heading frame carries the angle as little-endian float32. */
void test_encode_heading_payload() {
  const float angle = -137.5f;
  const auto body = decode_block(encode(7, Heading {.deg = angle}).view());
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(MsgId::Heading), body[1]);
  TEST_ASSERT_EQUAL_HEX8(7, body[2]);
  const float got =
      plrs::read_f32_little_endian({body.data() + HEADER_BYTES, sizeof(float)});
  TEST_ASSERT_EQUAL_FLOAT(angle, got);
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

/** @brief A Heading frame parses back to its id, seq, and angle. */
void test_parser_roundtrip_heading() {
  Parser p;
  auto out = feed_all(p, encode(9, Heading {.deg = 42.0f}).view());
  TEST_ASSERT_EQUAL_size_t(1, out.size());
  TEST_ASSERT_TRUE(out[0].has_value());
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(MsgId::Heading),
                         static_cast<uint8_t>(out[0]->id));
  TEST_ASSERT_EQUAL_HEX8(9, out[0]->seq);
  TEST_ASSERT_EQUAL_FLOAT(42.0f, plrs::read_f32_little_endian(out[0]->payload));
}

/** @brief A generic frame round-trips its seq and payload. */
void test_parser_generic_roundtrip() {
  const std::vector<uint8_t> payload {1, 2, 3};
  auto e = encode(MsgId::Heading, 5, payload);
  TEST_ASSERT_TRUE(e.has_value());
  Parser p;
  auto out = feed_all(p, e->view());
  TEST_ASSERT_EQUAL_size_t(1, out.size());
  TEST_ASSERT_TRUE(out[0].has_value());
  TEST_ASSERT_EQUAL_HEX8(5, out[0]->seq);
  TEST_ASSERT_TRUE(payload == out[0]->payload);
}

/** @brief Two concatenated frames both parse. */
void test_parser_back_to_back() {
  std::vector<uint8_t> stream;
  for (const auto &f :
       {encode(1, Heading {.deg = 10.0f}), encode(2, Heading {.deg = 20.0f})}) {
    const auto v = f.view();
    stream.insert(stream.end(), v.begin(), v.end());
  }
  Parser p;
  auto out = feed_all(p, stream);
  TEST_ASSERT_EQUAL_size_t(2, out.size());
  TEST_ASSERT_TRUE(out[0].has_value() && out[1].has_value());
  TEST_ASSERT_EQUAL_HEX8(1, out[0]->seq);
  TEST_ASSERT_EQUAL_HEX8(2, out[1]->seq);
}

/** @brief A bad CRC is reported, and the next frame still parses. */
void test_parser_bad_crc_then_resync() {
  std::vector<uint8_t> data {PROTOCOL_VERSION,
                             static_cast<uint8_t>(MsgId::Heading),
                             1,
                             0xAA,
                             0xBB,
                             0xCC,
                             0xDD};
  const auto bad = plrs::write_u16_little_endian(crc16(data) ^ 0xFFFF);
  data.push_back(bad[0]);
  data.push_back(bad[1]);

  std::vector<uint8_t> stream;
  const auto bad_frame = cobs_encode(data);
  const auto bv = bad_frame->view();
  stream.insert(stream.end(), bv.begin(), bv.end());
  const auto good_frame = encode(2, Heading {.deg = 20.0f});
  const auto gv = good_frame.view();
  stream.insert(stream.end(), gv.begin(), gv.end());

  Parser p;
  auto out = feed_all(p, stream);
  TEST_ASSERT_EQUAL_size_t(2, out.size());
  TEST_ASSERT_FALSE(out[0].has_value());
  TEST_ASSERT_TRUE(out[0].error() == Error::BadCrc);
  TEST_ASSERT_TRUE(out[1].has_value());
  TEST_ASSERT_EQUAL_HEX8(2, out[1]->seq);
}

/** @brief A valid frame with the wrong version is reported as WrongVersion. */
void test_parser_wrong_version() {
  std::vector<uint8_t> data {
      99, static_cast<uint8_t>(MsgId::Heading), 1, 0xAA, 0xBB, 0xCC, 0xDD};
  const auto crc = plrs::write_u16_little_endian(crc16(data));
  data.push_back(crc[0]);
  data.push_back(crc[1]);

  Parser p;
  auto out = feed_all(p, cobs_encode(data)->view());
  TEST_ASSERT_EQUAL_size_t(1, out.size());
  TEST_ASSERT_FALSE(out[0].has_value());
  TEST_ASSERT_TRUE(out[0].error() == Error::WrongVersion);
}

/** @brief A gap past FRAME_TIMEOUT drops the partial frame. */
void test_parser_timeout_drops_partial() {
  const auto frame = encode(1, Heading {.deg = 10.0f});
  const auto v = frame.view();
  Parser p;
  for (std::size_t i = 0; i < 3; i++) {
    p.feed(v[i], Ms {0});
  }
  bool got_frame = false;
  for (std::size_t i = 3; i < v.size(); i++) {
    auto r = p.feed(v[i], Ms {100});
    if (r && r->has_value()) {
      got_frame = true;
    }
  }
  TEST_ASSERT_FALSE(got_frame);
}

/** @brief Steady sub-timeout byte gaps do not drop a frame. */
void test_parser_no_false_timeout() {
  const auto frame = encode(3, Heading {.deg = -5.0f});
  const auto v = frame.view();
  Parser p;
  std::optional<std::expected<Frame, Error>> last;
  Ms t {0};
  for (uint8_t b : v) {
    auto r = p.feed(b, t);
    if (r) {
      last = r;
    }
    t += Ms {10};
  }
  TEST_ASSERT_TRUE(last.has_value());
  TEST_ASSERT_TRUE(last->has_value());
  TEST_ASSERT_EQUAL_HEX8(3, (*last)->seq);
}

// ---------------------------------------------------------------------------
// Heading message
// ---------------------------------------------------------------------------

/** @brief A payload that is not 4 bytes is rejected. */
void test_heading_from_payload_wrong_size() {
  const std::array<uint8_t, 3> three {1, 2, 3};
  TEST_ASSERT_FALSE(Heading::from_payload(three).has_value());
}

/** @brief encode -> Parser -> Heading::from_payload recovers the angle. */
void test_heading_end_to_end() {
  const float angle = 123.25f;
  const auto frame = encode(4, Heading {.deg = angle});
  Parser p;
  auto out = feed_all(p, frame.view());
  TEST_ASSERT_EQUAL_size_t(1, out.size());
  TEST_ASSERT_TRUE(out[0].has_value());
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(MsgId::Heading),
                         static_cast<uint8_t>(out[0]->id));
  const auto h = Heading::from_payload(out[0]->payload);
  TEST_ASSERT_TRUE(h.has_value());
  TEST_ASSERT_EQUAL_FLOAT(angle, h->deg);
}

// ---------------------------------------------------------------------------
// Attitude message
// ---------------------------------------------------------------------------

/** @brief A payload that is not 16 bytes is rejected. */
void test_attitude_from_payload_wrong_size() {
  const std::array<uint8_t, 12> twelve {};
  TEST_ASSERT_FALSE(Attitude::from_payload(twelve).has_value());
}

/** @brief An Attitude frame carries four little-endian floats then a flag byte.
 */
void test_encode_attitude_payload() {
  const Attitude msg {.heading_deg = -90.0f,
                      .roll_deg = 12.5f,
                      .pitch_deg = -3.0f,
                      .yaw_rate_dps = 7.25f,
                      .heading_valid = true};
  const auto body = decode_block(encode(3, msg).view());
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(MsgId::Attitude), body[1]);
  TEST_ASSERT_EQUAL_HEX8(3, body[2]);
  TEST_ASSERT_EQUAL_FLOAT(
      msg.heading_deg,
      plrs::read_f32_little_endian({body.data() + HEADER_BYTES, 4}));
  TEST_ASSERT_EQUAL_FLOAT(
      msg.roll_deg,
      plrs::read_f32_little_endian({body.data() + HEADER_BYTES + 4, 4}));
  TEST_ASSERT_EQUAL_FLOAT(
      msg.pitch_deg,
      plrs::read_f32_little_endian({body.data() + HEADER_BYTES + 8, 4}));
  TEST_ASSERT_EQUAL_FLOAT(
      msg.yaw_rate_dps,
      plrs::read_f32_little_endian({body.data() + HEADER_BYTES + 12, 4}));
  TEST_ASSERT_EQUAL_HEX8(1, body[HEADER_BYTES + 16]);
}

/** @brief encode -> Parser -> Attitude::from_payload recovers all five fields.
 */
void test_attitude_end_to_end() {
  const Attitude msg {.heading_deg = 45.0f,
                      .roll_deg = -8.0f,
                      .pitch_deg = 2.5f,
                      .yaw_rate_dps = -1.5f,
                      .heading_valid = false};
  const auto frame = encode(5, msg);
  Parser p;
  auto out = feed_all(p, frame.view());
  TEST_ASSERT_EQUAL_size_t(1, out.size());
  TEST_ASSERT_TRUE(out[0].has_value());
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(MsgId::Attitude),
                         static_cast<uint8_t>(out[0]->id));
  const auto a = Attitude::from_payload(out[0]->payload);
  TEST_ASSERT_TRUE(a.has_value());
  TEST_ASSERT_EQUAL_FLOAT(msg.heading_deg, a->heading_deg);
  TEST_ASSERT_EQUAL_FLOAT(msg.roll_deg, a->roll_deg);
  TEST_ASSERT_EQUAL_FLOAT(msg.pitch_deg, a->pitch_deg);
  TEST_ASSERT_EQUAL_FLOAT(msg.yaw_rate_dps, a->yaw_rate_dps);
  TEST_ASSERT_FALSE(a->heading_valid);
}

// ---------------------------------------------------------------------------
// RawAttitude message
// ---------------------------------------------------------------------------

/** @brief A payload that is not 8 bytes is rejected. */
void test_raw_attitude_from_payload_wrong_size() {
  const std::array<uint8_t, 4> four {};
  TEST_ASSERT_FALSE(RawAttitude::from_payload(four).has_value());
}

/** @brief A RawAttitude frame carries heel then yaw rate as little-endian
 * float32s. */
void test_encode_raw_attitude_payload() {
  const RawAttitude msg {.heel_deg = 12.5f, .yaw_rate_dps = -7.25f};
  const auto body = decode_block(encode(3, msg).view());
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(MsgId::RawAttitude), body[1]);
  TEST_ASSERT_EQUAL_HEX8(3, body[2]);
  TEST_ASSERT_EQUAL_FLOAT(
      msg.heel_deg,
      plrs::read_f32_little_endian({body.data() + HEADER_BYTES, 4}));
  TEST_ASSERT_EQUAL_FLOAT(
      msg.yaw_rate_dps,
      plrs::read_f32_little_endian({body.data() + HEADER_BYTES + 4, 4}));
}

/** @brief encode -> Parser -> RawAttitude::from_payload recovers both fields.
 */
void test_raw_attitude_end_to_end() {
  const RawAttitude msg {.heel_deg = -8.0f, .yaw_rate_dps = 1.5f};
  const auto frame = encode(5, msg);
  Parser p;
  auto out = feed_all(p, frame.view());
  TEST_ASSERT_EQUAL_size_t(1, out.size());
  TEST_ASSERT_TRUE(out[0].has_value());
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(MsgId::RawAttitude),
                         static_cast<uint8_t>(out[0]->id));
  const auto a = RawAttitude::from_payload(out[0]->payload);
  TEST_ASSERT_TRUE(a.has_value());
  TEST_ASSERT_EQUAL_FLOAT(msg.heel_deg, a->heel_deg);
  TEST_ASSERT_EQUAL_FLOAT(msg.yaw_rate_dps, a->yaw_rate_dps);
}

// ---------------------------------------------------------------------------
// Sender
// ---------------------------------------------------------------------------

/** @brief Successive frames carry 0, 1, 2, ... as their seq. */
void test_sender_increments_seq() {
  Sender s;
  Parser p;
  for (uint8_t expected = 0; expected < 3; expected++) {
    const auto frame = s.next(Heading {.deg = 1.0f});
    auto out = feed_all(p, frame.view());
    TEST_ASSERT_EQUAL_size_t(1, out.size());
    TEST_ASSERT_TRUE(out[0].has_value());
    TEST_ASSERT_EQUAL_HEX8(expected, out[0]->seq);
  }
}

/** @brief The seq counter wraps from 255 back to 0. */
void test_sender_seq_wraps() {
  Sender s;
  Parser p;
  uint8_t last = 0xFF;
  for (int i = 0; i < 257; i++) {
    const auto frame = s.next(Heading {.deg = 0.0f});
    auto out = feed_all(p, frame.view());
    TEST_ASSERT_EQUAL_size_t(1, out.size());
    last = out[0]->seq;
  }
  TEST_ASSERT_EQUAL_HEX8(0, last);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_crc16_known_answer);
  RUN_TEST(test_crc16_empty);
  RUN_TEST(test_cobs_encode_example);
  RUN_TEST(test_cobs_encode_no_zero_in_block);
  RUN_TEST(test_cobs_encode_oversize_rejected);
  RUN_TEST(test_cobs_roundtrip);
  RUN_TEST(test_cobs_decode_rejects_zero_in_block);
  RUN_TEST(test_cobs_decode_rejects_truncated);
  RUN_TEST(test_encode_frame_layout);
  RUN_TEST(test_encode_oversize_rejected);
  RUN_TEST(test_encode_heading_payload);
  RUN_TEST(test_parser_roundtrip_heading);
  RUN_TEST(test_parser_generic_roundtrip);
  RUN_TEST(test_parser_back_to_back);
  RUN_TEST(test_parser_bad_crc_then_resync);
  RUN_TEST(test_parser_wrong_version);
  RUN_TEST(test_parser_timeout_drops_partial);
  RUN_TEST(test_parser_no_false_timeout);
  RUN_TEST(test_heading_from_payload_wrong_size);
  RUN_TEST(test_heading_end_to_end);
  RUN_TEST(test_attitude_from_payload_wrong_size);
  RUN_TEST(test_encode_attitude_payload);
  RUN_TEST(test_attitude_end_to_end);
  RUN_TEST(test_raw_attitude_from_payload_wrong_size);
  RUN_TEST(test_encode_raw_attitude_payload);
  RUN_TEST(test_raw_attitude_end_to_end);
  RUN_TEST(test_sender_increments_seq);
  RUN_TEST(test_sender_seq_wraps);
  return UNITY_END();
}
