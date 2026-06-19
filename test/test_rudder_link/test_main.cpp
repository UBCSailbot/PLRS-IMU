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
  return UNITY_END();
}
