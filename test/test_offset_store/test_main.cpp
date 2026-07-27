/**
 * Host (native) unit tests for the mag-offset persistence policy and blob.
 */

#include "offset_store.h"
#include <array>
#include <unity.h>

using namespace offset_store;
using fusion::Ms;

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// Blob encode / validate
// ---------------------------------------------------------------------------

void test_encode_validate_roundtrip() {
  auto blob = encode(-15.5f);
  auto got = validate(ByteSpan(blob.data(), blob.size()));
  TEST_ASSERT_TRUE(got.has_value());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, -15.5f, *got);
}

void test_reject_short_blob() {
  std::array<uint8_t, 4> tiny {};
  TEST_ASSERT_FALSE(validate(ByteSpan(tiny.data(), tiny.size())).has_value());
}

void test_reject_erased_sector() {
  // A fresh flash sector reads as all-0xFF: wrong magic, so rejected.
  std::array<uint8_t, BLOB_BYTES> erased;
  erased.fill(0xff);
  TEST_ASSERT_FALSE(
      validate(ByteSpan(erased.data(), erased.size())).has_value());
}

void test_reject_bad_magic() {
  auto blob = encode(10.0f);
  blob[0] ^= 0xff;
  TEST_ASSERT_FALSE(validate(ByteSpan(blob.data(), blob.size())).has_value());
}

void test_reject_wrong_version() {
  auto blob = encode(10.0f);
  blob[4] = 0xfe; // bump the version byte; CRC no longer matters, version gate
  TEST_ASSERT_FALSE(validate(ByteSpan(blob.data(), blob.size())).has_value());
}

void test_reject_corrupted_payload() {
  auto blob = encode(10.0f);
  blob[8] ^= 0xff; // flip an offset byte, leaving the CRC stale
  TEST_ASSERT_FALSE(validate(ByteSpan(blob.data(), blob.size())).has_value());
}

void test_reject_out_of_range_offset() {
  auto blob = encode(200.0f); // beyond +-180, re-CRC'd but non-physical
  TEST_ASSERT_FALSE(validate(ByteSpan(blob.data(), blob.size())).has_value());
}

// ---------------------------------------------------------------------------
// Persist policy
// ---------------------------------------------------------------------------

void test_first_write_when_validated() {
  OffsetPersistPolicy p;
  // GNSS-validated (low variance): the first write is due immediately.
  TEST_ASSERT_TRUE(p.due(10.0f, 1.0f, Ms {1000}));
}

void test_not_due_while_variance_high() {
  OffsetPersistPolicy p;
  // Not yet GNSS-observed: variance above the gate, so never persist.
  TEST_ASSERT_FALSE(p.due(10.0f, 100.0f, Ms {1000}));
}

void test_not_due_before_min_interval() {
  OffsetPersistPolicy p;
  p.mark_written(10.0f, Ms {1000});
  // A big change but only 10 s later: the interval gate blocks it.
  TEST_ASSERT_FALSE(p.due(30.0f, 1.0f, Ms {11000}));
}

void test_not_due_without_meaningful_change() {
  OffsetPersistPolicy p;
  p.mark_written(10.0f, Ms {1000});
  // Well past the interval but the offset barely moved: skip the write.
  TEST_ASSERT_FALSE(p.due(10.2f, 1.0f, Ms {1'000'000}));
}

void test_due_after_interval_and_change() {
  OffsetPersistPolicy p;
  p.mark_written(10.0f, Ms {1000});
  TEST_ASSERT_TRUE(p.due(15.0f, 1.0f, Ms {1'000'000}));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_encode_validate_roundtrip);
  RUN_TEST(test_reject_short_blob);
  RUN_TEST(test_reject_erased_sector);
  RUN_TEST(test_reject_bad_magic);
  RUN_TEST(test_reject_wrong_version);
  RUN_TEST(test_reject_corrupted_payload);
  RUN_TEST(test_reject_out_of_range_offset);
  RUN_TEST(test_first_write_when_validated);
  RUN_TEST(test_not_due_while_variance_high);
  RUN_TEST(test_not_due_before_min_interval);
  RUN_TEST(test_not_due_without_meaningful_change);
  RUN_TEST(test_due_after_interval_and_change);
  return UNITY_END();
}
