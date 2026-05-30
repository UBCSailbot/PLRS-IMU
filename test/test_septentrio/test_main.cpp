/**
 * Host (native) unit tests for the SBF protocol core.
 */

#include "sbf_protocol.h"
#include "test_helpers.h"
#include <unity.h>
#include <vector>

using namespace sbf;

// ---------------------------------------------------------------------------
// Compile-time regression tests.
//
// Pins the CRC-CCITT/XMODEM variant against the canonical check vector
// ("123456789" -> 0x31C3, per the CRC catalogue). Also guards against
// constexpr loss in crc_ccitt and read_little_endian.
// ---------------------------------------------------------------------------
namespace {
constexpr std::array<uint8_t, 9> kCheckBytes = {'1', '2', '3', '4', '5',
                                                '6', '7', '8', '9'};
constexpr uint16_t kCheckCrc =
    crc_ccitt({kCheckBytes.data(), kCheckBytes.size()});
static_assert(kCheckCrc == 0x31C3);

constexpr std::array<uint8_t, 8> kLeBytes = {0x78, 0x56, 0x34, 0x12,
                                             0x21, 0x43, 0x65, 0x87};
static_assert(read_little_endian<uint16_t>({kLeBytes.data(), kLeBytes.size()},
                                           0) == 0x5678);
static_assert(read_little_endian<uint32_t>({kLeBytes.data(), kLeBytes.size()},
                                           0) == 0x12345678);
static_assert(read_little_endian<uint64_t>({kLeBytes.data(), kLeBytes.size()},
                                           0) == 0x8765432112345678ULL);

// Feed all bytes from `frame` into the parser at a fixed timestamp. Returns
// the result of the final byte (the only one that can carry a complete
// packet).
std::optional<Packet> feed_block(Parser &p, const std::vector<uint8_t> &frame,
                                 Ms t = Ms{0}) {
  std::optional<Packet> result;
  for (uint8_t b : frame) {
    result = p.feed(b, t);
  }
  return result;
}
} // namespace

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// crc_ccitt()
// ---------------------------------------------------------------------------

/** @brief Canonical CRC-CCITT/XMODEM check vector. */
void test_crc_ccitt_check_vector() {
  const std::vector<uint8_t> data = {'1', '2', '3', '4', '5',
                                     '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX16(0x31C3, crc_ccitt({data.data(), data.size()}));
}

/** @brief Empty input with default init returns the seed (0). */
void test_crc_ccitt_empty() { TEST_ASSERT_EQUAL_HEX16(0x0000, crc_ccitt({})); }

/** @brief crc_ccitt(b2, crc_ccitt(b1)) == crc_ccitt(b1 ++ b2). */
void test_crc_ccitt_chains_with_init() {
  const std::vector<uint8_t> left = {0xDE, 0xAD};
  const std::vector<uint8_t> right = {0xBE, 0xEF};
  std::vector<uint8_t> joined = left;
  joined.insert(joined.end(), right.begin(), right.end());

  uint16_t chained = crc_ccitt({left.data(), left.size()});
  chained = crc_ccitt({right.data(), right.size()}, chained);

  TEST_ASSERT_EQUAL_HEX16(crc_ccitt({joined.data(), joined.size()}), chained);
}

// ---------------------------------------------------------------------------
// read_little_endian()
// ---------------------------------------------------------------------------

/** @brief LE byte order for u16/u32/u64 reads. */
void test_read_le_known_bytes() {
  const std::array<uint8_t, 8> b = {0x78, 0x56, 0x34, 0x12,
                                    0x21, 0x43, 0x65, 0x87};
  TEST_ASSERT_EQUAL_HEX16(
      0x5678, read_little_endian<uint16_t>({b.data(), b.size()}, 0));
  TEST_ASSERT_EQUAL_HEX32(
      0x12345678, read_little_endian<uint32_t>({b.data(), b.size()}, 0));
  TEST_ASSERT_EQUAL_HEX64(0x8765432112345678ULL, read_little_endian<uint64_t>(
                                                     {b.data(), b.size()}, 0));
}

/** @brief f32/f64 round-trip via bit_cast at unaligned offsets. */
void test_read_le_floats() {
  const float fv = -3.5f;
  const double dv = 1234567.89;
  const uint32_t fu = std::bit_cast<uint32_t>(fv);
  const uint64_t du = std::bit_cast<uint64_t>(dv);

  // Place values at offset 1 (unaligned for u32/u64 access on Cortex-M0+).
  std::array<uint8_t, 1 + sizeof(double)> buf{};
  for (std::size_t i = 0; i < sizeof(float); i++) {
    buf[1 + i] = static_cast<uint8_t>(fu >> (i * BITS_PER_BYTE));
  }
  TEST_ASSERT_EQUAL_FLOAT(
      fv, read_little_endian<float>({buf.data(), buf.size()}, 1));

  for (std::size_t i = 0; i < sizeof(double); i++) {
    buf[1 + i] = static_cast<uint8_t>(du >> (i * BITS_PER_BYTE));
  }
  TEST_ASSERT_EQUAL_DOUBLE(
      dv, read_little_endian<double>({buf.data(), buf.size()}, 1));
}

/** @brief Reads at non-aligned offsets do not corrupt the value. */
void test_read_le_unaligned_offset() {
  const std::array<uint8_t, 7> b = {0xAA, 0x78, 0x56, 0x34, 0x12, 0xCC, 0xDD};
  TEST_ASSERT_EQUAL_HEX32(
      0x12345678, read_little_endian<uint32_t>({b.data(), b.size()}, 1));
}

// ---------------------------------------------------------------------------
// Packet
// ---------------------------------------------------------------------------

/** @brief Default-constructed Packet has empty body and zero id. */
void test_packet_default() {
  Packet p;
  TEST_ASSERT_EQUAL_UINT16(0, p.id);
  TEST_ASSERT_EQUAL_UINT16(0, p.body_length);
  TEST_ASSERT_EQUAL_size_t(0, p.body().size());
  TEST_ASSERT_EQUAL_UINT16(0, p.block_number());
  TEST_ASSERT_EQUAL_UINT8(0, p.revision());
}

/** @brief block_number masks the low 13 bits; revision is the top 3. */
void test_packet_id_decoding() {
  Packet p;
  // Block number 4007 (PVTGeodetic), revision 2.
  p.id = static_cast<uint16_t>(4007 | (2 << ID_REV_SHIFT));
  TEST_ASSERT_EQUAL_UINT16(4007, p.block_number());
  TEST_ASSERT_EQUAL_UINT8(2, p.revision());
}

// ---------------------------------------------------------------------------
// Parser - framing happy path
// ---------------------------------------------------------------------------

/** @brief Clean block: 4-byte body, correct CRC, parses to a Packet. */
void test_parse_clean_block() {
  Parser p;
  const std::vector<uint8_t> body = {0x12, 0x34, 0x56, 0x78};
  auto frame = stest::make_block(5921, body); // EndOfPVT
  auto pkt = feed_block(p, frame);

  TEST_ASSERT_TRUE(pkt.has_value());
  TEST_ASSERT_EQUAL_UINT16(5921, pkt->id);
  TEST_ASSERT_EQUAL_UINT16(body.size(), pkt->body_length);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(body.data(), pkt->data.data(), body.size());
}

/** @brief Zero-body block (length = HEADER_LEN) skips Body state. */
void test_parse_zero_body_block() {
  Parser p;
  auto frame = stest::make_block(5921, {});
  auto pkt = feed_block(p, frame);

  TEST_ASSERT_TRUE(pkt.has_value());
  TEST_ASSERT_EQUAL_UINT16(0, pkt->body_length);
}

/** @brief Two complete blocks fed back-to-back both decode. */
void test_back_to_back_blocks() {
  Parser p;
  auto first = stest::make_block(5921, {0x01, 0x02, 0x03, 0x04});
  auto second = stest::make_block(5938, {0x05, 0x06, 0x07, 0x08});

  std::optional<Packet> a, b;
  for (uint8_t byte : first) {
    a = p.feed(byte, Ms{0});
  }
  for (uint8_t byte : second) {
    b = p.feed(byte, Ms{0});
  }

  TEST_ASSERT_TRUE(a.has_value());
  TEST_ASSERT_EQUAL_UINT16(5921, a->id);
  TEST_ASSERT_TRUE(b.has_value());
  TEST_ASSERT_EQUAL_UINT16(5938, b->id);
}

/** @brief Block body containing the SYNC1+SYNC2 byte pair still parses
 *         end-to-end (Body state counts bytes; it does not look for sync). */
void test_dollar_at_in_body_does_not_false_trigger() {
  Parser p;
  const std::vector<uint8_t> body = {SYNC1, SYNC2, 0xCA, 0xFE};
  auto frame = stest::make_block(5921, body);
  auto pkt = feed_block(p, frame);

  TEST_ASSERT_TRUE(pkt.has_value());
  TEST_ASSERT_EQUAL_UINT16(body.size(), pkt->body_length);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(body.data(), pkt->data.data(), body.size());
}

// ---------------------------------------------------------------------------
// Parser - resync paths
// ---------------------------------------------------------------------------

/** @brief SYNC1 followed by non-SYNC2 resets and recovers on next frame. */
void test_sync1_then_non_sync2_resyncs() {
  Parser p;
  const std::vector<uint8_t> garbage = {SYNC1, 'X', 'Y', 'Z'};
  feed_block(p, garbage);
  TEST_ASSERT_FALSE(p.mid_frame());

  auto frame = stest::make_block(5921, {0xAA, 0xBB, 0xCC, 0xDD});
  auto pkt = feed_block(p, frame);
  TEST_ASSERT_TRUE(pkt.has_value());
}

/** @brief Repeated SYNC1 before SYNC2 still starts a frame ($$@...). */
void test_repeated_sync1_then_sync2_starts_frame() {
  Parser p;
  auto frame = stest::make_block(5921, {0x11, 0x22, 0x33, 0x44});
  std::vector<uint8_t> with_extra = {SYNC1, SYNC1, SYNC1};
  with_extra.insert(with_extra.end(), frame.begin(), frame.end());

  auto pkt = feed_block(p, with_extra);
  TEST_ASSERT_TRUE(pkt.has_value());
  TEST_ASSERT_EQUAL_UINT16(5921, pkt->id);
}

/** @brief Corrupted CRC byte rejects the block, parser resyncs cleanly. */
void test_bad_crc_rejected_then_resyncs() {
  Parser p;
  auto bad = stest::make_block(5921, {0xAA, 0xBB, 0xCC, 0xDD});
  bad[2] ^= 0xFF; // flip a CRC bit
  auto pkt_bad = feed_block(p, bad);
  TEST_ASSERT_FALSE(pkt_bad.has_value());

  auto good = stest::make_block(5938, {0x10, 0x20, 0x30, 0x40});
  auto pkt_good = feed_block(p, good);
  TEST_ASSERT_TRUE(pkt_good.has_value());
  TEST_ASSERT_EQUAL_UINT16(5938, pkt_good->id);
}

// ---------------------------------------------------------------------------
// Parser - length validation
// ---------------------------------------------------------------------------

/** @brief Length not a multiple of 4 is rejected; parser resyncs. */
void test_length_not_multiple_of_4_rejected() {
  Parser p;
  auto bad = stest::make_block_with_length(5921, 11, {0x00, 0x00, 0x00});
  auto pkt_bad = feed_block(p, bad);
  TEST_ASSERT_FALSE(pkt_bad.has_value());
  TEST_ASSERT_FALSE(p.mid_frame());

  auto good = stest::make_block(5921, {0xAA, 0xBB, 0xCC, 0xDD});
  auto pkt_good = feed_block(p, good);
  TEST_ASSERT_TRUE(pkt_good.has_value());
}

/** @brief Length below HEADER_LEN is rejected. */
void test_length_below_header_rejected() {
  Parser p;
  auto bad = stest::make_block_with_length(5921, 4, {});
  auto pkt_bad = feed_block(p, bad);
  TEST_ASSERT_FALSE(pkt_bad.has_value());
  TEST_ASSERT_FALSE(p.mid_frame());
}

/** @brief Length > MAX_BLOCK rejected at the header before any body read. */
void test_length_exceeds_max_rejected() {
  Parser p;
  auto bad =
      stest::make_block_with_length(5921, MAX_BLOCK + LENGTH_ALIGNMENT, {});
  auto pkt_bad = feed_block(p, bad);
  TEST_ASSERT_FALSE(pkt_bad.has_value());
  TEST_ASSERT_FALSE(p.mid_frame());

  auto good = stest::make_block(5921, {0xAA, 0xBB, 0xCC, 0xDD});
  auto pkt_good = feed_block(p, good);
  TEST_ASSERT_TRUE(pkt_good.has_value());
}

// ---------------------------------------------------------------------------
// Parser - timeout
// ---------------------------------------------------------------------------

/** @brief Bytes arriving more than FRAME_TIMEOUT apart drop the partial
 *         frame; a fresh frame on the stalled byte still parses. */
void test_timeout_drops_stalled_block() {
  Parser p;
  auto first = stest::make_block(5921, {0x01, 0x02, 0x03, 0x04});

  // Feed first half of the original frame, then stall.
  const std::size_t half = first.size() / 2;
  for (std::size_t i = 0; i < half; i++) {
    p.feed(first[i], Ms{0});
  }
  TEST_ASSERT_TRUE(p.mid_frame());

  // After the timeout, the parser resets before processing the next byte.
  // Feed a fresh frame starting from the stall instant.
  auto fresh = stest::make_block(5938, {0xAA, 0xBB, 0xCC, 0xDD});
  std::optional<Packet> pkt;
  for (std::size_t i = 0; i < fresh.size(); i++) {
    pkt = p.feed(fresh[i], Parser::FRAME_TIMEOUT + Ms{1});
  }
  TEST_ASSERT_TRUE(pkt.has_value());
  TEST_ASSERT_EQUAL_UINT16(5938, pkt->id);
}

/** @brief Bytes within FRAME_TIMEOUT do not trigger spurious resets. */
void test_no_timeout_when_under_threshold() {
  Parser p;
  auto frame = stest::make_block(5921, {0xDE, 0xAD, 0xBE, 0xEF});

  // Feed each byte 100ms apart (< FRAME_TIMEOUT of 250ms).
  std::optional<Packet> pkt;
  for (std::size_t i = 0; i < frame.size(); i++) {
    pkt = p.feed(frame[i], Ms{100} * static_cast<long>(i));
  }
  TEST_ASSERT_TRUE(pkt.has_value());
  TEST_ASSERT_EQUAL_UINT16(5921, pkt->id);
}

// ---------------------------------------------------------------------------

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_crc_ccitt_check_vector);
  RUN_TEST(test_crc_ccitt_empty);
  RUN_TEST(test_crc_ccitt_chains_with_init);
  RUN_TEST(test_read_le_known_bytes);
  RUN_TEST(test_read_le_floats);
  RUN_TEST(test_read_le_unaligned_offset);
  RUN_TEST(test_packet_default);
  RUN_TEST(test_packet_id_decoding);
  RUN_TEST(test_parse_clean_block);
  RUN_TEST(test_parse_zero_body_block);
  RUN_TEST(test_back_to_back_blocks);
  RUN_TEST(test_dollar_at_in_body_does_not_false_trigger);
  RUN_TEST(test_sync1_then_non_sync2_resyncs);
  RUN_TEST(test_repeated_sync1_then_sync2_starts_frame);
  RUN_TEST(test_bad_crc_rejected_then_resyncs);
  RUN_TEST(test_length_not_multiple_of_4_rejected);
  RUN_TEST(test_length_below_header_rejected);
  RUN_TEST(test_length_exceeds_max_rejected);
  RUN_TEST(test_timeout_drops_stalled_block);
  RUN_TEST(test_no_timeout_when_under_threshold);
  return UNITY_END();
}
