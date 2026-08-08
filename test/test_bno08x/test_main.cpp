/**
 * Host (native) unit tests for the BNO085 SHTP framing and SH-2 report decode.
 */

#include "sh2_reports.h"
#include "shtp_protocol.h"
#include "test_helpers.h"
#include <unity.h>
#include <vector>

using namespace btest;

// ---------------------------------------------------------------------------
// Compile-time regression tests.
//
// A 100 Hz Rotation-Vector Set-Feature payload, framed onto the control
// channel, must land on the exact bytes the SH-2 firmware expects. Also guards
// build_set_feature / encode_packet against losing constexpr.
// ---------------------------------------------------------------------------
namespace {
constexpr auto kRvFeature = sh2::build_set_feature(sh2::Report::RotationVector,
                                                   10000); // 100 Hz
static_assert(kRvFeature[0] == 0xFD); // Set-Feature request
static_assert(kRvFeature[1] == 0x05); // Rotation Vector
static_assert(kRvFeature[5] == 0x10); // 10000 us = 0x00002710, LE
static_assert(kRvFeature[6] == 0x27);
static_assert(kRvFeature[7] == 0x00);
static_assert(kRvFeature[8] == 0x00);

constexpr auto kFrame =
    shtp::encode_packet(shtp::Channel::Control,
                        0,
                        shtp::ByteSpan(kRvFeature.data(), kRvFeature.size()));
static_assert(kFrame.has_value());
static_assert(kFrame->len == shtp::HEADER_LEN + sh2::SET_FEATURE_LEN);
static_assert(kFrame->bytes[0] == 21); // total length incl. header, LE
static_assert(kFrame->bytes[1] == 0);
static_assert(kFrame->bytes[2] == 2); // control channel

// The Save-DCD command request must land on the exact bytes the SH-2 firmware
// expects: report 0xF2, command 0x06, nine zero parameters. Without this a
// converged mag calibration is never persisted across reboots.
constexpr auto kSaveDcd = sh2::build_save_dcd();
static_assert(kSaveDcd.size() == 12);
static_assert(kSaveDcd[0] == 0xF2); // command request report id
static_assert(kSaveDcd[2] == 0x06); // Save DCD
static_assert(kSaveDcd[1] == 0x00);
static_assert(kSaveDcd[3] == 0x00 && kSaveDcd[11] == 0x00); // params zeroed
} // namespace

// ---------------------------------------------------------------------------
// SHTP framing.
// ---------------------------------------------------------------------------

void test_parse_packet_basic() {
  auto cargo = make_cargo(3, 7, {0xAA, 0xBB, 0xCC});
  auto p = shtp::parse_packet(cargo);
  TEST_ASSERT_TRUE(p.has_value());
  TEST_ASSERT_EQUAL_UINT8(3, p->channel);
  TEST_ASSERT_EQUAL_UINT8(7, p->seq);
  TEST_ASSERT_EQUAL_size_t(3, p->payload.size());
  TEST_ASSERT_EQUAL_UINT8(0xAA, p->payload[0]);
  TEST_ASSERT_EQUAL_UINT8(0xCC, p->payload[2]);
}

void test_parse_masks_continuation_bit() {
  auto cargo = make_cargo(2, 0, {0x01, 0x02});
  cargo[1] |= 0x80; // set the continuation flag in the length MSB
  auto p = shtp::parse_packet(cargo);
  TEST_ASSERT_TRUE(p.has_value());
  TEST_ASSERT_EQUAL_UINT8(2, p->channel);
  TEST_ASSERT_EQUAL_size_t(2, p->payload.size());
}

void test_parse_rejects_short_header() {
  std::vector<uint8_t> two {0x02, 0x00};
  TEST_ASSERT_FALSE(shtp::parse_packet(two).has_value());
}

void test_parse_rejects_truncated() {
  auto cargo = make_cargo(3, 0, {0x01, 0x02, 0x03, 0x04});
  cargo.resize(cargo.size() - 2); // header still claims the full length
  TEST_ASSERT_FALSE(shtp::parse_packet(cargo).has_value());
}

void test_parse_rejects_zero_length() {
  std::vector<uint8_t> empty {0x00, 0x00, 0x03, 0x00};
  TEST_ASSERT_FALSE(shtp::parse_packet(empty).has_value());
}

void test_parse_empty_payload_ok() {
  auto cargo = make_cargo(1, 5, {});
  auto p = shtp::parse_packet(cargo);
  TEST_ASSERT_TRUE(p.has_value());
  TEST_ASSERT_EQUAL_size_t(0, p->payload.size());
}

void test_encode_roundtrips_through_parse() {
  std::vector<uint8_t> payload {0x10, 0x20, 0x30, 0x40};
  auto enc =
      shtp::encode_packet(shtp::Channel::Control,
                          9,
                          shtp::ByteSpan(payload.data(), payload.size()));
  TEST_ASSERT_TRUE(enc.has_value());
  auto p = shtp::parse_packet(enc->view());
  TEST_ASSERT_TRUE(p.has_value());
  TEST_ASSERT_EQUAL_UINT8(2, p->channel);
  TEST_ASSERT_EQUAL_UINT8(9, p->seq);
  TEST_ASSERT_EQUAL_size_t(4, p->payload.size());
  TEST_ASSERT_EQUAL_UINT8(0x40, p->payload[3]);
}

void test_encode_rejects_oversize() {
  std::vector<uint8_t> big(shtp::MAX_TX_PAYLOAD + 1, 0);
  auto enc = shtp::encode_packet(
      shtp::Channel::Control, 0, shtp::ByteSpan(big.data(), big.size()));
  TEST_ASSERT_FALSE(enc.has_value());
}

// ---------------------------------------------------------------------------
// SH-2 fixed-point + Set-Feature.
// ---------------------------------------------------------------------------

void test_q_to_float() {
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0f, sh2::q_to_float(1 << 8, 8));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, -2.0f, sh2::q_to_float(-(2 << 8), 8));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.5f, sh2::q_to_float(1 << 13, 14));
}

void test_build_set_feature_layout() {
  auto f = sh2::build_set_feature(sh2::Report::Accelerometer, 5000);
  TEST_ASSERT_EQUAL_size_t(17, f.size());
  TEST_ASSERT_EQUAL_UINT8(0xFD, f[0]);
  TEST_ASSERT_EQUAL_UINT8(0x01, f[1]);
  TEST_ASSERT_EQUAL_UINT8(0x00, f[2]);
  // 5000 = 0x1388, little endian over bytes 5..8.
  TEST_ASSERT_EQUAL_UINT8(0x88, f[5]);
  TEST_ASSERT_EQUAL_UINT8(0x13, f[6]);
  TEST_ASSERT_EQUAL_UINT8(0x00, f[7]);
  TEST_ASSERT_EQUAL_UINT8(0x00, f[8]);
}

// ---------------------------------------------------------------------------
// SH-2 report decode.
// ---------------------------------------------------------------------------

void test_read_accel() {
  std::vector<uint8_t> batch = base_timestamp();
  append(batch, vec3_report(0x01, 1.0f, -2.0f, 0.5f, sh2::ACCEL_Q));
  auto v = sh2::read_accel(shtp::ByteSpan(batch.data(), batch.size()));
  TEST_ASSERT_TRUE(v.has_value());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 1.0f, v->x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -2.0f, v->y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.5f, v->z);
}

void test_read_gyro_and_mag() {
  std::vector<uint8_t> batch = base_timestamp();
  append(batch, vec3_report(0x02, 0.25f, -0.5f, 1.0f, sh2::GYRO_Q));
  append(batch, vec3_report(0x03, 10.0f, -20.0f, 5.0f, sh2::MAG_Q));
  auto span = shtp::ByteSpan(batch.data(), batch.size());
  auto g = sh2::read_gyro(span);
  auto m = sh2::read_mag(span);
  TEST_ASSERT_TRUE(g.has_value());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.25f, g->x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 1.0f, g->z);
  TEST_ASSERT_TRUE(m.has_value());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 10.0f, m->x);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, -20.0f, m->y);
}

void test_read_rotation_vector_mapping() {
  std::vector<uint8_t> batch = base_timestamp();
  append(batch, rotation_report(0.5f, 0.5f, 0.5f, 0.5f)); // w, x, y, z
  auto q =
      sh2::read_rotation_vector(shtp::ByteSpan(batch.data(), batch.size()));
  TEST_ASSERT_TRUE(q.has_value());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.5f, q->w);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.5f, q->x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.5f, q->y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.5f, q->z);
}

void test_full_batch_all_reports() {
  std::vector<uint8_t> batch = base_timestamp();
  append(batch, vec3_report(0x01, 0.0f, 0.0f, 9.81f, sh2::ACCEL_Q));
  append(batch, vec3_report(0x02, 0.1f, 0.0f, 0.0f, sh2::GYRO_Q));
  append(batch, vec3_report(0x03, 25.0f, 0.0f, 40.0f, sh2::MAG_Q));
  append(batch, rotation_report(1.0f, 0.0f, 0.0f, 0.0f));
  auto span = shtp::ByteSpan(batch.data(), batch.size());
  TEST_ASSERT_TRUE(sh2::read_accel(span).has_value());
  TEST_ASSERT_TRUE(sh2::read_gyro(span).has_value());
  TEST_ASSERT_TRUE(sh2::read_mag(span).has_value());
  auto q = sh2::read_rotation_vector(span);
  TEST_ASSERT_TRUE(q.has_value());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 1.0f, q->w);
  TEST_ASSERT_FLOAT_WITHIN(
      1e-2, 9.81f, sh2::read_accel(span)->z); // Q8 res ~0.004
}

void test_absent_report_is_nullopt() {
  std::vector<uint8_t> batch = base_timestamp();
  append(batch, vec3_report(0x01, 1.0f, 1.0f, 1.0f, sh2::ACCEL_Q));
  auto span = shtp::ByteSpan(batch.data(), batch.size());
  TEST_ASSERT_TRUE(sh2::read_accel(span).has_value());
  TEST_ASSERT_FALSE(sh2::read_gyro(span).has_value());
}

void test_unknown_report_stops_walk() {
  // An unknown id has unknown length, so the walker cannot advance past it:
  // reports before it are found, reports after it are not.
  std::vector<uint8_t> batch = base_timestamp();
  append(batch, vec3_report(0x01, 1.0f, 1.0f, 1.0f, sh2::ACCEL_Q));
  append(batch, make_report(0x99, {0x00, 0x00}));
  append(batch, vec3_report(0x02, 1.0f, 1.0f, 1.0f, sh2::GYRO_Q));
  auto span = shtp::ByteSpan(batch.data(), batch.size());
  TEST_ASSERT_TRUE(sh2::read_accel(span).has_value());
  TEST_ASSERT_FALSE(sh2::read_gyro(span).has_value());
}

void test_truncated_report_is_safe() {
  std::vector<uint8_t> batch = base_timestamp();
  auto accel = vec3_report(0x01, 1.0f, 1.0f, 1.0f, sh2::ACCEL_Q);
  accel.resize(accel.size() - 3); // chop mid-report
  append(batch, accel);
  auto span = shtp::ByteSpan(batch.data(), batch.size());
  TEST_ASSERT_FALSE(sh2::read_accel(span).has_value());
}

// Regression: heading validity gates on the Rotation Vector's calibration
// accuracy (status bits 0-1), so an uncalibrated mag cannot steer the boat.
void test_read_report_accuracy() {
  std::vector<uint8_t> batch = base_timestamp();
  // status 0x0B = 0b1011: accuracy 3 in the low two bits, other bits set.
  append(batch, rotation_report(1.0f, 0.0f, 0.0f, 0.0f, 0x0B));
  auto span = shtp::ByteSpan(batch.data(), batch.size());
  auto acc = sh2::read_report_accuracy(span, sh2::Report::RotationVector);
  TEST_ASSERT_TRUE(acc.has_value());
  TEST_ASSERT_EQUAL_UINT8(3, *acc); // masked to the accuracy bits
  // A report absent from the batch yields nullopt, not a false 0.
  TEST_ASSERT_FALSE(
      sh2::read_report_accuracy(span, sh2::Report::MagCalibrated).has_value());
}

// Regression: the quaternion decode still reads correctly through a nonzero
// status byte (the accuracy refactor must not shift the data offset).
void test_rotation_vector_decodes_with_status_set() {
  std::vector<uint8_t> batch = base_timestamp();
  append(batch, rotation_report(1.0f, 0.0f, 0.0f, 0.0f, 0x02));
  auto q =
      sh2::read_rotation_vector(shtp::ByteSpan(batch.data(), batch.size()));
  TEST_ASSERT_TRUE(q.has_value());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 1.0f, q->w);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_parse_packet_basic);
  RUN_TEST(test_parse_masks_continuation_bit);
  RUN_TEST(test_parse_rejects_short_header);
  RUN_TEST(test_parse_rejects_truncated);
  RUN_TEST(test_parse_rejects_zero_length);
  RUN_TEST(test_parse_empty_payload_ok);
  RUN_TEST(test_encode_roundtrips_through_parse);
  RUN_TEST(test_encode_rejects_oversize);
  RUN_TEST(test_q_to_float);
  RUN_TEST(test_build_set_feature_layout);
  RUN_TEST(test_read_accel);
  RUN_TEST(test_read_gyro_and_mag);
  RUN_TEST(test_read_rotation_vector_mapping);
  RUN_TEST(test_full_batch_all_reports);
  RUN_TEST(test_absent_report_is_nullopt);
  RUN_TEST(test_unknown_report_stops_walk);
  RUN_TEST(test_truncated_report_is_safe);
  RUN_TEST(test_read_report_accuracy);
  RUN_TEST(test_rotation_vector_decodes_with_status_set);
  return UNITY_END();
}
