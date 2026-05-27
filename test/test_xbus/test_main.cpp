/**
 * Host (native) unit tests for the Xbus protocol core.
 */

#include "xbus_protocol.h"
#include <unity.h>

using namespace xbus;

// ---------------------------------------------------------------------------
// Compile-time regression tests.
//
// Verifies GoToConfig frames to FA FF 30 00 D1 (MT0101P spec), and guards
// against payload() losing constexpr — encode() calls payload() internally,
// so the block fails to compile if constexpr is dropped.
// ---------------------------------------------------------------------------
namespace {
constexpr auto kGoToConfigCmd = Packet::command(MID::GoToConfig, {});
static_assert(kGoToConfigCmd.has_value());
constexpr auto kGoToConfig = encode(*kGoToConfigCmd);
static_assert(kGoToConfig.has_value());
static_assert(kGoToConfig->len == 5);
static_assert(kGoToConfig->bytes[0] == 0xFA); // preamble
static_assert(kGoToConfig->bytes[1] == 0xFF); // BID_MASTER
static_assert(kGoToConfig->bytes[2] == 0x30); // MID::GoToConfig
static_assert(kGoToConfig->bytes[3] == 0x00); // len = 0
static_assert(kGoToConfig->bytes[4] == 0xD1); // checksum

// Oversize payloads are also rejected at compile time.
constexpr std::array<uint8_t, MAX_PAYLOAD + 1> kBig{};
constexpr auto kOversize =
    Packet::command(MID::SetOutputConfig, ByteSpan(kBig.data(), kBig.size()));
static_assert(!kOversize.has_value());

// Feed all bytes in a span into the parser at a fixed timestamp. Returns the
// result from the final byte (the only one that can carry a complete packet).
std::optional<Packet> feed_frame(Parser &p, ByteSpan frame, Ms t = Ms{0}) {
  std::optional<Packet> result;
  for (uint8_t b : frame)
    result = p.feed(b, t);
  return result;
}
} // namespace

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// checksum()
// ---------------------------------------------------------------------------

/** @brief Known checksum value and receiver invariant: BID+MID+LEN+CHK == 0. */
void test_checksum_gotoconfig() {
  uint8_t chk = checksum(0xFF, static_cast<uint8_t>(MID::GoToConfig), {});
  TEST_ASSERT_EQUAL_HEX8(0xD1, chk);

  uint8_t sum = 0xFF;
  sum += static_cast<uint8_t>(MID::GoToConfig);
  sum += 0x00; // len
  sum += chk;
  TEST_ASSERT_EQUAL_HEX8(0x00, sum);
}

/** @brief Checksum accumulates payload bytes; receiver invariant holds. */
void test_checksum_with_payload() {
  const uint8_t payload[] = {0x20, 0x10, 0x00, 0x64};
  uint8_t chk = checksum(0xFF, static_cast<uint8_t>(MID::SetOutputConfig),
                         ByteSpan(payload, sizeof payload));
  TEST_ASSERT_EQUAL_HEX8(0xA9, chk);

  uint8_t sum = 0xFF;
  sum += static_cast<uint8_t>(MID::SetOutputConfig);
  sum += static_cast<uint8_t>(sizeof payload);
  for (uint8_t b : payload)
    sum += b;
  sum += chk;
  TEST_ASSERT_EQUAL_HEX8(0x00, sum);
}

// ---------------------------------------------------------------------------
// Packet::command()
// ---------------------------------------------------------------------------

/** @brief Empty payload: correct mid, default bid == BID_MASTER, len == 0. */
void test_command_empty_payload() {
  auto result = Packet::command(MID::GoToConfig, {});
  TEST_ASSERT_TRUE(result.has_value());
  TEST_ASSERT(result->mid == MID::GoToConfig);
  TEST_ASSERT_EQUAL_HEX8(BID_MASTER, result->bid);
  TEST_ASSERT_EQUAL_size_t(0, result->len);
}

/** @brief Payload bytes are copied into data[] and len is set to match. */
void test_command_payload_copied() {
  const uint8_t payload[] = {0x20, 0x10, 0x00, 0x64};
  auto result =
      Packet::command(MID::SetOutputConfig, ByteSpan(payload, sizeof payload));
  TEST_ASSERT_TRUE(result.has_value());
  TEST_ASSERT_EQUAL_size_t(sizeof payload, result->len);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, result->data.data(), sizeof payload);
}

/** @brief Explicit bid overrides BID_MASTER and is stored in the packet. */
void test_command_custom_bid() {
  auto result = Packet::command(MID::GoToConfig, {}, 0x01);
  TEST_ASSERT_TRUE(result.has_value());
  TEST_ASSERT_EQUAL_HEX8(0x01, result->bid);
}

/** @brief Exactly MAX_PAYLOAD bytes is accepted (boundary). */
void test_command_accepts_max_payload() {
  std::array<uint8_t, MAX_PAYLOAD> payload{};
  auto result = Packet::command(MID::SetOutputConfig,
                                ByteSpan(payload.data(), payload.size()));
  TEST_ASSERT_TRUE(result.has_value());
  TEST_ASSERT_EQUAL_size_t(MAX_PAYLOAD, result->len);
}

/** @brief MAX_PAYLOAD + 1 bytes is rejected; extended-length frames are out of
 * scope. */
void test_command_rejects_oversize() {
  std::array<uint8_t, MAX_PAYLOAD + 1> payload{};
  auto result = Packet::command(MID::SetOutputConfig,
                                ByteSpan(payload.data(), payload.size()));
  TEST_ASSERT_FALSE(result.has_value());
}

// ---------------------------------------------------------------------------
// encode()
// ---------------------------------------------------------------------------

/** @brief GoToConfig encodes to FA FF 30 00 D1 (MT0101P spec). */
void test_encode_gotoconfig_bytes() {
  auto enc = encode(Packet::command(MID::GoToConfig, {}).value());
  TEST_ASSERT_TRUE(enc.has_value());
  const uint8_t expected[] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};
  TEST_ASSERT_EQUAL_size_t(sizeof expected, enc->len);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, enc->bytes.data(), sizeof expected);
}

/** @brief SetOutputConfig with 4-byte payload produces a correct 9-byte frame.
 */
void test_encode_with_payload_bytes() {
  const uint8_t payload[] = {0x20, 0x10, 0x00, 0x64};
  auto enc = encode(
      Packet::command(MID::SetOutputConfig, ByteSpan(payload, sizeof payload))
          .value());
  TEST_ASSERT_TRUE(enc.has_value());
  const uint8_t expected[] = {0xFA, 0xFF, 0xC0, 0x04, 0x20,
                              0x10, 0x00, 0x64, 0xA9};
  TEST_ASSERT_EQUAL_size_t(sizeof expected, enc->len);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, enc->bytes.data(), sizeof expected);
}

/** @brief encode() returns nullopt for a packet with a manually overflowed len.
 */
void test_encode_rejects_oversize() {
  Packet p;
  p.mid = MID::SetOutputConfig;
  p.len = MAX_PAYLOAD + 1;
  TEST_ASSERT_FALSE(encode(p).has_value());
}

// ---------------------------------------------------------------------------
// read_u16_big_endian()
// ---------------------------------------------------------------------------

/** @brief Reassembles a known big-endian 16-bit value correctly. */
void test_read_u16_big_endian() {
  const uint8_t b[] = {0x20, 0x10};
  TEST_ASSERT_EQUAL_HEX16(0x2010, read_u16_big_endian(ByteSpan(b, sizeof b)));
}

/** @brief High and low byte extremes (0xFF, 0x00). */
void test_read_u16_big_endian_boundaries() {
  const uint8_t hi[] = {0xFF, 0xFF};
  TEST_ASSERT_EQUAL_HEX16(0xFFFF, read_u16_big_endian(ByteSpan(hi, 2)));
  const uint8_t lo[] = {0x00, 0x00};
  TEST_ASSERT_EQUAL_HEX16(0x0000, read_u16_big_endian(ByteSpan(lo, 2)));
}

// ---------------------------------------------------------------------------
// read_f32_big_endian()
// ---------------------------------------------------------------------------

/** @brief IEEE 754 1.0f = 0x3F800000 big-endian. */
void test_read_f32_big_endian_one() {
  const uint8_t b[] = {0x3F, 0x80, 0x00, 0x00};
  TEST_ASSERT_EQUAL_FLOAT(1.0f, read_f32_big_endian(ByteSpan(b, sizeof b)));
}

/** @brief Negative value: -1.0f = 0xBF800000 big-endian. */
void test_read_f32_big_endian_negative() {
  const uint8_t b[] = {0xBF, 0x80, 0x00, 0x00};
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, read_f32_big_endian(ByteSpan(b, sizeof b)));
}

/** @brief Zero: all bytes zero. */
void test_read_f32_big_endian_zero() {
  const uint8_t b[] = {0x00, 0x00, 0x00, 0x00};
  TEST_ASSERT_EQUAL_FLOAT(0.0f, read_f32_big_endian(ByteSpan(b, sizeof b)));
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

/** @brief Happy path: a clean frame decodes to the correct packet. */
void test_parse_clean_frame() {
  Parser p;
  const uint8_t frame[] = {0xFA, 0xFF, 0x30, 0x00, 0xD1}; // GoToConfig
  auto pkt = feed_frame(p, frame);
  TEST_ASSERT_TRUE(pkt.has_value());
  TEST_ASSERT(pkt->mid == MID::GoToConfig);
  TEST_ASSERT_EQUAL_HEX8(BID_MASTER, pkt->bid);
  TEST_ASSERT_EQUAL_size_t(0, pkt->len);
}

/** @brief Frame with payload: bytes are copied, len matches. */
void test_parse_with_payload() {
  Parser p;
  // SetOutputConfig, Quaternion@100Hz: FA FF C0 04 20 10 00 64 A9
  const uint8_t frame[] = {0xFA, 0xFF, 0xC0, 0x04, 0x20,
                           0x10, 0x00, 0x64, 0xA9};
  auto pkt = feed_frame(p, frame);
  TEST_ASSERT_TRUE(pkt.has_value());
  TEST_ASSERT(pkt->mid == MID::SetOutputConfig);
  TEST_ASSERT_EQUAL_size_t(4, pkt->len);
  const uint8_t expected[] = {0x20, 0x10, 0x00, 0x64};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, pkt->data.data(), 4);
}

/** @brief Bad checksum yields nullopt; parser resyncs and accepts the next
 * frame. */
void test_bad_checksum_rejected_then_resyncs() {
  Parser p;
  const uint8_t bad[] = {0xFA, 0xFF, 0x30, 0x00, 0xD2}; // D2 instead of D1
  TEST_ASSERT_FALSE(feed_frame(p, bad).has_value());

  const uint8_t good[] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};
  auto pkt = feed_frame(p, good);
  TEST_ASSERT_TRUE(pkt.has_value());
  TEST_ASSERT(pkt->mid == MID::GoToConfig);
}

/** @brief Junk bytes before the preamble are silently ignored. */
void test_junk_before_frame_ignored() {
  Parser p;
  const uint8_t junk[] = {0x00, 0x01, 0x02};
  for (uint8_t b : junk)
    TEST_ASSERT_FALSE(p.feed(b, Ms{0}).has_value());

  const uint8_t frame[] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};
  auto pkt = feed_frame(p, frame);
  TEST_ASSERT_TRUE(pkt.has_value());
  TEST_ASSERT(pkt->mid == MID::GoToConfig);
}

/** @brief A stalled frame is dropped when the next byte arrives past
 * FRAME_TIMEOUT. */
void test_timeout_drops_stalled_frame() {
  Parser p;
  // Feed preamble + bid + mid at t=0; parser is mid-frame in Len state.
  p.feed(0xFA, Ms{0});
  p.feed(0xFF, Ms{0});
  p.feed(0x30, Ms{0});
  TEST_ASSERT_TRUE(p.mid_frame());

  // 60ms > FRAME_TIMEOUT(50ms): the 0xFA triggers a reset then starts a new
  // frame.
  p.feed(0xFA, Ms{60});
  p.feed(0xFF, Ms{60});
  p.feed(0x30, Ms{60});
  p.feed(0x00, Ms{60});
  auto pkt = p.feed(0xD1, Ms{60});
  TEST_ASSERT_TRUE(pkt.has_value());
  TEST_ASSERT(pkt->mid == MID::GoToConfig);
}

/** @brief A slow but uninterrupted frame (40ms between bytes) is not timed out.
 */
void test_no_false_timeout() {
  Parser p;
  const uint8_t frame[] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};
  std::optional<Packet> pkt;
  for (std::size_t i = 0; i < sizeof frame; ++i)
    pkt = p.feed(frame[i], Ms{static_cast<long long>(i * 40)});
  TEST_ASSERT_TRUE(pkt.has_value());
  TEST_ASSERT(pkt->mid == MID::GoToConfig);
}

/** @brief LEN == 0xFF (extended frame) is rejected; parser resets and resyncs.
 */
void test_extended_length_rejected() {
  Parser p;
  p.feed(0xFA, Ms{0});
  p.feed(0xFF, Ms{0});
  p.feed(0x36, Ms{0}); // MTData2
  TEST_ASSERT_TRUE(p.mid_frame());

  p.feed(0xFF, Ms{0}); // LEN_EXTENDED triggers reset
  TEST_ASSERT_FALSE(p.mid_frame());

  const uint8_t frame[] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};
  TEST_ASSERT_TRUE(feed_frame(p, frame).has_value());
}

/** @brief Two back-to-back frames in one buffer both decode. */
void test_back_to_back_frames() {
  Parser p;
  const uint8_t two_frames[] = {
      0xFA, 0xFF, 0x30, 0x00, 0xD1, // GoToConfig
      0xFA, 0xFF, 0x30, 0x00, 0xD1, // GoToConfig again
  };
  int count = 0;
  for (uint8_t b : two_frames)
    if (p.feed(b, Ms{0}).has_value())
      ++count;
  TEST_ASSERT_EQUAL(2, count);
}

// ---------------------------------------------------------------------------
// find_data()
// ---------------------------------------------------------------------------

/** @brief Matching sub-packet is found and bytes are correct. */
void test_find_data_found() {
  // DataId::Quaternion (0x2010), len=4, value=1.0f (0x3F800000 big-endian)
  const uint8_t raw[] = {0x20, 0x10, 0x04, 0x3F, 0x80, 0x00, 0x00};
  Packet p;
  p.mid = MID::MTData2;
  p.len = sizeof raw;
  for (std::size_t i = 0; i < sizeof raw; ++i)
    p.data[i] = raw[i];

  auto dp = find_data(p, DataId::Quaternion);
  TEST_ASSERT_TRUE(dp.has_value());
  TEST_ASSERT(dp->id == DataId::Quaternion);
  TEST_ASSERT_EQUAL_size_t(4, dp->bytes.size());
  TEST_ASSERT_EQUAL_FLOAT(1.0f, read_f32_big_endian(dp->bytes));
}

/** @brief Non-matching DataId returns nullopt. */
void test_find_data_not_found() {
  const uint8_t raw[] = {0x20, 0x10, 0x04, 0x3F,
                         0x80, 0x00, 0x00}; // Quaternion only
  Packet p;
  p.mid = MID::MTData2;
  p.len = sizeof raw;
  for (std::size_t i = 0; i < sizeof raw; ++i)
    p.data[i] = raw[i];

  TEST_ASSERT_FALSE(find_data(p, DataId::Acceleration).has_value());
}

/** @brief Truncated sub-packet (stated len > remaining bytes) bails safely. */
void test_find_data_truncated_safe() {
  // Header claims len=16 but only 2 data bytes follow.
  const uint8_t raw[] = {0x20, 0x10, 0x10, 0x3F, 0x80};
  Packet p;
  p.mid = MID::MTData2;
  p.len = sizeof raw;
  for (std::size_t i = 0; i < sizeof raw; ++i)
    p.data[i] = raw[i];

  TEST_ASSERT_FALSE(find_data(p, DataId::Quaternion).has_value());
}

/** @brief With multiple sub-packets, the correct one is found by DataId. */
void test_find_data_multiple_subpackets() {
  const uint8_t raw[] = {
      0x20, 0x10, 0x04, 0x3F, 0x80, 0x00, 0x00, // Quaternion: 1.0f
      0x40, 0x20, 0x04, 0x40, 0x00, 0x00, 0x00, // Acceleration: 2.0f
  };
  Packet p;
  p.mid = MID::MTData2;
  p.len = sizeof raw;
  for (std::size_t i = 0; i < sizeof raw; ++i)
    p.data[i] = raw[i];

  auto dp = find_data(p, DataId::Acceleration);
  TEST_ASSERT_TRUE(dp.has_value());
  TEST_ASSERT(dp->id == DataId::Acceleration);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, read_f32_big_endian(dp->bytes));
}

// ---------------------------------------------------------------------------

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_checksum_gotoconfig);
  RUN_TEST(test_checksum_with_payload);
  RUN_TEST(test_command_empty_payload);
  RUN_TEST(test_command_payload_copied);
  RUN_TEST(test_command_custom_bid);
  RUN_TEST(test_command_accepts_max_payload);
  RUN_TEST(test_command_rejects_oversize);
  RUN_TEST(test_encode_gotoconfig_bytes);
  RUN_TEST(test_encode_with_payload_bytes);
  RUN_TEST(test_encode_rejects_oversize);
  RUN_TEST(test_read_u16_big_endian);
  RUN_TEST(test_read_u16_big_endian_boundaries);
  RUN_TEST(test_read_f32_big_endian_one);
  RUN_TEST(test_read_f32_big_endian_negative);
  RUN_TEST(test_read_f32_big_endian_zero);
  RUN_TEST(test_parse_clean_frame);
  RUN_TEST(test_parse_with_payload);
  RUN_TEST(test_bad_checksum_rejected_then_resyncs);
  RUN_TEST(test_junk_before_frame_ignored);
  RUN_TEST(test_timeout_drops_stalled_frame);
  RUN_TEST(test_no_false_timeout);
  RUN_TEST(test_extended_length_rejected);
  RUN_TEST(test_back_to_back_frames);
  RUN_TEST(test_find_data_found);
  RUN_TEST(test_find_data_not_found);
  RUN_TEST(test_find_data_truncated_safe);
  RUN_TEST(test_find_data_multiple_subpackets);
  return UNITY_END();
}
