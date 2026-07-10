/**
 * Host (native) unit tests for the SBF protocol core.
 */

#include "command.h"
#include "nmea_protocol.h"
#include "sbf_blocks.h"
#include "sbf_protocol.h"
#include "test_helpers.h"
#include "wire_parser.h"
#include <unity.h>
#include <variant>
#include <vector>

using namespace sbf;
using septentrio_gnss::Command;
using septentrio_gnss::Message;
using septentrio_gnss::Parser;
using septentrio_gnss::Reply;
using septentrio_gnss::ReplyKind;

// ---------------------------------------------------------------------------
// Compile-time regression tests.
//
// Pins the CRC-CCITT/XMODEM variant against the canonical check vector
// ("123456789" -> 0x31C3, per the CRC catalogue). Also guards against
// constexpr loss in crc_ccitt and read_little_endian.
// ---------------------------------------------------------------------------
namespace {
constexpr std::array<uint8_t, 9> kCheckBytes = {
    '1', '2', '3', '4', '5', '6', '7', '8', '9'};
constexpr uint16_t kCheckCrc =
    crc_ccitt({kCheckBytes.data(), kCheckBytes.size()});
static_assert(kCheckCrc == 0x31C3);

constexpr std::array<uint8_t, 8> kLeBytes = {
    0x78, 0x56, 0x34, 0x12, 0x21, 0x43, 0x65, 0x87};
static_assert(read_little_endian<uint16_t>({kLeBytes.data(), kLeBytes.size()},
                                           0) == 0x5678);
static_assert(read_little_endian<uint32_t>({kLeBytes.data(), kLeBytes.size()},
                                           0) == 0x12345678);
static_assert(read_little_endian<uint64_t>({kLeBytes.data(), kLeBytes.size()},
                                           0) == 0x8765432112345678ULL);

// Feed all bytes into the parser at a fixed timestamp; return the SBF
// Packet variant if the last byte completed one, else nullopt.
std::optional<Packet>
feed_block(Parser &p, const std::vector<uint8_t> &frame, Ms t = Ms {0}) {
  std::optional<Packet> result;
  for (uint8_t b : frame) {
    auto msg = p.feed(b, t);
    if (msg && std::holds_alternative<Packet>(*msg)) {
      result = std::get<Packet>(*msg);
    }
  }
  return result;
}

// NMEA counterpart.
std::optional<nmea::Sentence>
feed_sentence(Parser &p, const std::vector<uint8_t> &frame, Ms t = Ms {0}) {
  std::optional<nmea::Sentence> result;
  for (uint8_t b : frame) {
    auto msg = p.feed(b, t);
    if (msg && std::holds_alternative<nmea::Sentence>(*msg)) {
      result = std::get<nmea::Sentence>(*msg);
    }
  }
  return result;
}

// Reply counterpart.
std::optional<Reply>
feed_reply(Parser &p, const std::vector<uint8_t> &frame, Ms t = Ms {0}) {
  std::optional<Reply> result;
  for (uint8_t b : frame) {
    auto msg = p.feed(b, t);
    if (msg && std::holds_alternative<Reply>(*msg)) {
      result = std::get<Reply>(*msg);
    }
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
  const std::vector<uint8_t> data = {
      '1', '2', '3', '4', '5', '6', '7', '8', '9'};
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
  const std::array<uint8_t, 8> b = {
      0x78, 0x56, 0x34, 0x12, 0x21, 0x43, 0x65, 0x87};
  TEST_ASSERT_EQUAL_HEX16(
      0x5678, read_little_endian<uint16_t>({b.data(), b.size()}, 0));
  TEST_ASSERT_EQUAL_HEX32(
      0x12345678, read_little_endian<uint32_t>({b.data(), b.size()}, 0));
  TEST_ASSERT_EQUAL_HEX64(
      0x8765432112345678ULL,
      read_little_endian<uint64_t>({b.data(), b.size()}, 0));
}

/** @brief f32/f64 round-trip via bit_cast at unaligned offsets. */
void test_read_le_floats() {
  const float fv = -3.5f;
  const double dv = 1234567.89;
  const uint32_t fu = std::bit_cast<uint32_t>(fv);
  const uint64_t du = std::bit_cast<uint64_t>(dv);

  // Place values at offset 1 (unaligned for u32/u64 access on Cortex-M0+).
  std::array<uint8_t, 1 + sizeof(double)> buf {};
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

  auto a = feed_block(p, first);
  auto b = feed_block(p, second);

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

/** @brief Garbage starting with '$' (which the wire parser interprets as
 *         in-progress NMEA) does not block a subsequent valid SBF block
 *         from parsing. */
void test_garbage_then_sbf_recovers() {
  Parser p;
  const std::vector<uint8_t> garbage = {SYNC1, 'X', 'Y', 'Z'};
  feed_block(p, garbage);

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
    p.feed(first[i], Ms {0});
  }
  TEST_ASSERT_TRUE(p.mid_frame());

  // After the timeout, the parser resets before processing the next byte.
  // Feed a fresh frame starting from the stall instant.
  auto fresh = stest::make_block(5938, {0xAA, 0xBB, 0xCC, 0xDD});
  auto pkt = feed_block(p, fresh, Parser::FRAME_TIMEOUT + Ms {1});
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
    auto m = p.feed(frame[i], Ms {100} * static_cast<long>(i));
    if (m && std::holds_alternative<Packet>(*m)) {
      pkt = std::get<Packet>(*m);
    }
  }
  TEST_ASSERT_TRUE(pkt.has_value());
  TEST_ASSERT_EQUAL_UINT16(5921, pkt->id);
}

// ---------------------------------------------------------------------------
// parse_pvt_geodetic()
// ---------------------------------------------------------------------------

namespace {
sbf::PVTGeodetic make_sentinel_pvt() {
  return sbf::PVTGeodetic {
      .tow = 123456789,
      .wnc = 2345,
      .mode = 0x01,
      .error = 0,
      .latitude = 0.85,
      .longitude = -2.13,
      .height = 42.5,
      .undulation = -17.25f,
      .v_north = 1.5f,
      .v_east = -2.5f,
      .v_up = 0.25f,
      .cog = 123.75f,
      .rx_clk_bias = 0.001234,
      .rx_clk_drift = 0.05f,
      .time_system = 0,
      .datum = 0,
      .nr_sv = 14,
      .wa_corr_info = 0x03,
      .reference_id = 4321,
      .mean_corr_age = 50,
      .signal_info = 0xCAFEBABE,
      .alert_flag = 0x01,
      .nr_bases = 1,
      .ppp_info = 0x1234,
      .latency = 100,
      .h_accuracy = 250,
      .v_accuracy = 350,
      .misc = 0x42,
  };
}

sbf::Packet make_packet(const std::vector<uint8_t> &body, uint16_t id) {
  sbf::Packet p;
  p.id = id;
  p.body_length = static_cast<uint16_t>(body.size());
  for (std::size_t i = 0; i < body.size(); i++) {
    p.data[i] = body[i];
  }
  return p;
}
} // namespace

/** @brief Round-trip: every field comes back bit-exact for a known body. */
void test_parse_pvt_geodetic_round_trip() {
  const auto expected = make_sentinel_pvt();
  const auto body = stest::make_pvt_geodetic_body(expected);
  const auto pkt = make_packet(body, sbf::pvt_geodetic_layout::BLOCK_NUMBER);

  const auto out = sbf::parse_pvt_geodetic(pkt);
  TEST_ASSERT_TRUE(out.has_value());
  TEST_ASSERT_EQUAL_UINT32(expected.tow, out->tow);
  TEST_ASSERT_EQUAL_UINT16(expected.wnc, out->wnc);
  TEST_ASSERT_EQUAL_UINT8(expected.mode, out->mode);
  TEST_ASSERT_EQUAL_UINT8(expected.error, out->error);
  TEST_ASSERT_EQUAL_DOUBLE(expected.latitude, out->latitude);
  TEST_ASSERT_EQUAL_DOUBLE(expected.longitude, out->longitude);
  TEST_ASSERT_EQUAL_DOUBLE(expected.height, out->height);
  TEST_ASSERT_EQUAL_FLOAT(expected.undulation, out->undulation);
  TEST_ASSERT_EQUAL_FLOAT(expected.v_north, out->v_north);
  TEST_ASSERT_EQUAL_FLOAT(expected.v_east, out->v_east);
  TEST_ASSERT_EQUAL_FLOAT(expected.v_up, out->v_up);
  TEST_ASSERT_EQUAL_FLOAT(expected.cog, out->cog);
  TEST_ASSERT_EQUAL_DOUBLE(expected.rx_clk_bias, out->rx_clk_bias);
  TEST_ASSERT_EQUAL_FLOAT(expected.rx_clk_drift, out->rx_clk_drift);
  TEST_ASSERT_EQUAL_UINT8(expected.time_system, out->time_system);
  TEST_ASSERT_EQUAL_UINT8(expected.datum, out->datum);
  TEST_ASSERT_EQUAL_UINT8(expected.nr_sv, out->nr_sv);
  TEST_ASSERT_EQUAL_UINT8(expected.wa_corr_info, out->wa_corr_info);
  TEST_ASSERT_EQUAL_UINT16(expected.reference_id, out->reference_id);
  TEST_ASSERT_EQUAL_UINT16(expected.mean_corr_age, out->mean_corr_age);
  TEST_ASSERT_EQUAL_UINT32(expected.signal_info, out->signal_info);
  TEST_ASSERT_EQUAL_UINT8(expected.alert_flag, out->alert_flag);
  TEST_ASSERT_EQUAL_UINT8(expected.nr_bases, out->nr_bases);
  TEST_ASSERT_EQUAL_UINT16(expected.ppp_info, out->ppp_info);
  TEST_ASSERT_EQUAL_UINT16(expected.latency, out->latency);
  TEST_ASSERT_EQUAL_UINT16(expected.h_accuracy, out->h_accuracy);
  TEST_ASSERT_EQUAL_UINT16(expected.v_accuracy, out->v_accuracy);
  TEST_ASSERT_EQUAL_UINT8(expected.misc, out->misc);
}

/** @brief Wrong block number returns nullopt even if body shape is right. */
void test_parse_pvt_geodetic_wrong_block_number() {
  const auto body = stest::make_pvt_geodetic_body(make_sentinel_pvt());
  const auto pkt = make_packet(body, /*EndOfPVT*/ 5921);
  TEST_ASSERT_FALSE(sbf::parse_pvt_geodetic(pkt).has_value());
}

/** @brief Body shorter than the Rev 2 layout is rejected. */
void test_parse_pvt_geodetic_short_body_rejected() {
  std::vector<uint8_t> body(sbf::pvt_geodetic_layout::MIN_BODY - 1, 0);
  const auto pkt = make_packet(body, sbf::pvt_geodetic_layout::BLOCK_NUMBER);
  TEST_ASSERT_FALSE(sbf::parse_pvt_geodetic(pkt).has_value());
}

/** @brief Body longer than the Rev 2 layout still parses (forward compat). */
void test_parse_pvt_geodetic_forward_compat() {
  auto body = stest::make_pvt_geodetic_body(make_sentinel_pvt());
  body.insert(body.end(), {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE});
  const auto pkt = make_packet(body, sbf::pvt_geodetic_layout::BLOCK_NUMBER);

  const auto out = sbf::parse_pvt_geodetic(pkt);
  TEST_ASSERT_TRUE(out.has_value());
  TEST_ASSERT_EQUAL_UINT8(0x42, out->misc); // last Rev 2 field still correct
}

/** @brief Do-Not-Use sentinels survive parsing unchanged. */
void test_parse_pvt_geodetic_dnu_preserved() {
  sbf::PVTGeodetic dnu = make_sentinel_pvt();
  dnu.tow = sbf::DNU_U4;
  dnu.wnc = sbf::DNU_U2;
  dnu.latitude = sbf::DNU_F8;
  dnu.undulation = sbf::DNU_F4;

  const auto body = stest::make_pvt_geodetic_body(dnu);
  const auto pkt = make_packet(body, sbf::pvt_geodetic_layout::BLOCK_NUMBER);

  const auto out = sbf::parse_pvt_geodetic(pkt);
  TEST_ASSERT_TRUE(out.has_value());
  TEST_ASSERT_EQUAL_UINT32(sbf::DNU_U4, out->tow);
  TEST_ASSERT_EQUAL_UINT16(sbf::DNU_U2, out->wnc);
  TEST_ASSERT_EQUAL_DOUBLE(sbf::DNU_F8, out->latitude);
  TEST_ASSERT_EQUAL_FLOAT(sbf::DNU_F4, out->undulation);
}

// ---------------------------------------------------------------------------
// parse_pos_cov_geodetic() / parse_vel_cov_geodetic() / parse_att_euler() /
// parse_att_cov_euler()
// ---------------------------------------------------------------------------

/** @brief PosCovGeodetic round-trip: every field comes back bit-exact. */
void test_parse_pos_cov_geodetic_round_trip() {
  const sbf::PosCovGeodetic expected {
      .tow = 111,
      .wnc = 222,
      .mode = 0x05,
      .error = 0,
      .cov_latlat = 0.01f,
      .cov_lonlon = 0.02f,
      .cov_hgthgt = 0.04f,
      .cov_bb = 1e-6f,
      .cov_latlon = 0.001f,
      .cov_lathgt = -0.002f,
      .cov_latb = 0.003f,
      .cov_lonhgt = -0.004f,
      .cov_lonb = 0.005f,
      .cov_hb = -0.006f,
  };
  const auto body = stest::make_pos_cov_geodetic_body(expected);
  const auto pkt =
      make_packet(body, sbf::pos_cov_geodetic_layout::BLOCK_NUMBER);

  const auto out = sbf::parse_pos_cov_geodetic(pkt);
  TEST_ASSERT_TRUE(out.has_value());
  TEST_ASSERT_EQUAL_UINT32(expected.tow, out->tow);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_latlat, out->cov_latlat);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_lonlon, out->cov_lonlon);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_hgthgt, out->cov_hgthgt);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_bb, out->cov_bb);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_latlon, out->cov_latlon);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_lathgt, out->cov_lathgt);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_latb, out->cov_latb);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_lonhgt, out->cov_lonhgt);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_lonb, out->cov_lonb);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_hb, out->cov_hb);
}

/** @brief VelCovGeodetic round-trip: every field comes back bit-exact. */
void test_parse_vel_cov_geodetic_round_trip() {
  const sbf::VelCovGeodetic expected {
      .tow = 333,
      .wnc = 444,
      .mode = 0x05,
      .error = 0,
      .cov_vnvn = 0.1f,
      .cov_veve = 0.2f,
      .cov_vuvu = 0.4f,
      .cov_dtdt = 1e-3f,
      .cov_vnve = 0.01f,
      .cov_vnvu = -0.02f,
      .cov_vndt = 0.03f,
      .cov_vevu = -0.04f,
      .cov_vedt = 0.05f,
      .cov_vudt = -0.06f,
  };
  const auto body = stest::make_vel_cov_geodetic_body(expected);
  const auto pkt =
      make_packet(body, sbf::vel_cov_geodetic_layout::BLOCK_NUMBER);

  const auto out = sbf::parse_vel_cov_geodetic(pkt);
  TEST_ASSERT_TRUE(out.has_value());
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_vnvn, out->cov_vnvn);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_veve, out->cov_veve);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_vuvu, out->cov_vuvu);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_dtdt, out->cov_dtdt);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_vnve, out->cov_vnve);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_vnvu, out->cov_vnvu);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_vndt, out->cov_vndt);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_vevu, out->cov_vevu);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_vedt, out->cov_vedt);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_vudt, out->cov_vudt);
}

/** @brief AttEuler round-trip: heading/pitch/roll + rates come back exact. */
void test_parse_att_euler_round_trip() {
  const sbf::AttEuler expected {
      .tow = 555,
      .wnc = 666,
      .nr_sv = 10,
      .error = 0,
      .mode = 4, // heading + pitch + roll, fixed ambiguities
      .reserved = 0,
      .heading = 90.5f,
      .pitch = 2.25f,
      .roll = -1.5f,
      .pitch_dot = 0.1f,
      .roll_dot = -0.2f,
      .heading_dot = 0.3f,
  };
  const auto body = stest::make_att_euler_body(expected);
  const auto pkt = make_packet(body, sbf::att_euler_layout::BLOCK_NUMBER);

  const auto out = sbf::parse_att_euler(pkt);
  TEST_ASSERT_TRUE(out.has_value());
  TEST_ASSERT_EQUAL_UINT32(expected.tow, out->tow);
  TEST_ASSERT_EQUAL_UINT16(expected.mode, out->mode);
  TEST_ASSERT_EQUAL_FLOAT(expected.heading, out->heading);
  TEST_ASSERT_EQUAL_FLOAT(expected.pitch, out->pitch);
  TEST_ASSERT_EQUAL_FLOAT(expected.roll, out->roll);
  TEST_ASSERT_EQUAL_FLOAT(expected.pitch_dot, out->pitch_dot);
  TEST_ASSERT_EQUAL_FLOAT(expected.roll_dot, out->roll_dot);
  TEST_ASSERT_EQUAL_FLOAT(expected.heading_dot, out->heading_dot);
}

/** @brief AttCovEuler round-trip: covariance fields come back exact. */
void test_parse_att_cov_euler_round_trip() {
  const sbf::AttCovEuler expected {
      .tow = 777,
      .wnc = 888,
      .reserved = 0,
      .error = 0,
      .cov_headhead = 0.5f,
      .cov_pitchpitch = 0.25f,
      .cov_rollroll = 0.125f,
      .cov_headpitch = sbf::DNU_F4,
      .cov_headroll = sbf::DNU_F4,
      .cov_pitchroll = sbf::DNU_F4,
  };
  const auto body = stest::make_att_cov_euler_body(expected);
  const auto pkt = make_packet(body, sbf::att_cov_euler_layout::BLOCK_NUMBER);

  const auto out = sbf::parse_att_cov_euler(pkt);
  TEST_ASSERT_TRUE(out.has_value());
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_headhead, out->cov_headhead);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_pitchpitch, out->cov_pitchpitch);
  TEST_ASSERT_EQUAL_FLOAT(expected.cov_rollroll, out->cov_rollroll);
  TEST_ASSERT_EQUAL_FLOAT(sbf::DNU_F4, out->cov_headpitch);
  TEST_ASSERT_EQUAL_FLOAT(sbf::DNU_F4, out->cov_headroll);
  TEST_ASSERT_EQUAL_FLOAT(sbf::DNU_F4, out->cov_pitchroll);
}

// ---------------------------------------------------------------------------
// NMEA + wire dispatch
// ---------------------------------------------------------------------------

/** @brief Clean NMEA sentence parses to a Sentence with the right body. */
void test_parse_clean_nmea() {
  Parser p;
  auto frame = stest::make_nmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9");
  auto s = feed_sentence(p, frame);
  TEST_ASSERT_TRUE(s.has_value());
  TEST_ASSERT_EQUAL_STRING_LEN("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9",
                               s->data.data(),
                               s->length);
}

/** @brief Wrong NMEA checksum returns nullopt; parser resyncs. */
void test_nmea_bad_checksum_rejected() {
  Parser p;
  auto bad = stest::make_nmea("GPGGA,foo");
  // The two checksum chars are at indices size-4 and size-3 (before CR LF).
  bad[bad.size() - 4] ^= 0x01;
  auto s = feed_sentence(p, bad);
  TEST_ASSERT_FALSE(s.has_value());

  auto good = stest::make_nmea("GPGGA,foo");
  auto s2 = feed_sentence(p, good);
  TEST_ASSERT_TRUE(s2.has_value());
}

/** @brief NMEA followed by SBF on the same byte stream: both decode as
 *         their own variant alternative. */
void test_nmea_then_sbf_interleaved() {
  Parser p;
  auto nmea_frame = stest::make_nmea("GPGGA,42");
  auto sbf_frame = stest::make_block(5921, {0xAA, 0xBB, 0xCC, 0xDD});

  std::optional<nmea::Sentence> nmea_out;
  std::optional<Packet> sbf_out;
  for (uint8_t b : nmea_frame) {
    auto m = p.feed(b, Ms {0});
    if (m && std::holds_alternative<nmea::Sentence>(*m)) {
      nmea_out = std::get<nmea::Sentence>(*m);
    }
  }
  for (uint8_t b : sbf_frame) {
    auto m = p.feed(b, Ms {0});
    if (m && std::holds_alternative<Packet>(*m)) {
      sbf_out = std::get<Packet>(*m);
    }
  }
  TEST_ASSERT_TRUE(nmea_out.has_value());
  TEST_ASSERT_TRUE(sbf_out.has_value());
  TEST_ASSERT_EQUAL_UINT16(5921, sbf_out->id);
}

/** @brief Stray '$' mid NMEA line resets dispatch; a following '$@' starts
 *         a fresh SBF block instead of being consumed as NMEA body bytes. */
void test_dollar_mid_nmea_then_sbf_dispatches() {
  Parser p;
  // Begin a NMEA line (no terminator), then drop a clean SBF block.
  std::vector<uint8_t> stream = {'$', 'G', 'P', 'G', 'G', 'A'};
  auto sbf_frame = stest::make_block(5938, {0x10, 0x20, 0x30, 0x40});
  stream.insert(stream.end(), sbf_frame.begin(), sbf_frame.end());

  auto pkt = feed_block(p, stream);
  TEST_ASSERT_TRUE(pkt.has_value());
  TEST_ASSERT_EQUAL_UINT16(5938, pkt->id);
}

// ---------------------------------------------------------------------------
// Command::build()
// ---------------------------------------------------------------------------

/** @brief Command body is copied verbatim and terminated with '\r'. */
void test_command_build_appends_cr() {
  auto cmd = Command::build("setSBFOutput,Stream1,COM1,PVTGeodetic,sec1");
  TEST_ASSERT_TRUE(cmd.has_value());
  const auto view = cmd->view();
  TEST_ASSERT_EQUAL_size_t(43, view.size()); // 42 chars + '\r'
  TEST_ASSERT_EQUAL_HEX8('\r', view.back());
}

/** @brief Body that would not leave room for the terminator is rejected. */
void test_command_build_rejects_oversize() {
  std::array<char, septentrio_gnss::MAX_COMMAND_LEN> body {};
  body.fill('x');
  auto cmd = Command::build(std::string_view(body.data(), body.size()));
  TEST_ASSERT_FALSE(cmd.has_value());
}

// ---------------------------------------------------------------------------
// Wire parser - Reply
// ---------------------------------------------------------------------------

/** @brief "$R: ...\nCOM1>" parses to Reply{kind=Ok, text=body}. */
void test_parse_clean_reply_ok() {
  Parser p;
  auto frame =
      stest::make_reply(septentrio_gnss::REPLY_KIND_OK, " setSBFOutput\n  ack");
  auto r = feed_reply(p, frame);
  TEST_ASSERT_TRUE(r.has_value());
  TEST_ASSERT_EQUAL(static_cast<int>(ReplyKind::Ok), static_cast<int>(r->kind));
  TEST_ASSERT_EQUAL_STRING_LEN(
      " setSBFOutput\n  ack\n", r->data.data(), r->length);
}

/** @brief "$R?" is the Err kind. */
void test_parse_reply_err() {
  Parser p;
  auto frame = stest::make_reply(septentrio_gnss::REPLY_KIND_ERR,
                                 " SBFOutput: Not authorized!");
  auto r = feed_reply(p, frame);
  TEST_ASSERT_TRUE(r.has_value());
  TEST_ASSERT_EQUAL(static_cast<int>(ReplyKind::Err),
                    static_cast<int>(r->kind));
}

/** @brief "$R!" is the Info kind. */
void test_parse_reply_info() {
  Parser p;
  auto frame = stest::make_reply(septentrio_gnss::REPLY_KIND_INFO, " LogIn");
  auto r = feed_reply(p, frame);
  TEST_ASSERT_TRUE(r.has_value());
  TEST_ASSERT_EQUAL(static_cast<int>(ReplyKind::Info),
                    static_cast<int>(r->kind));
}

/** @brief "$R" followed by a non-kind char is NMEA, not Reply. */
void test_dollar_R_without_kind_char_is_nmea() {
  Parser p;
  // Build an NMEA sentence whose body starts with 'R'. The first char
  // after '$R' is 'M' which is not a reply kind, so the wire parser
  // should fall back to NMEA.
  auto frame = stest::make_nmea("RMC,123519,A,4807.038,N");
  auto s = feed_sentence(p, frame);
  TEST_ASSERT_TRUE(s.has_value());
  TEST_ASSERT_EQUAL_STRING_LEN(
      "RMC,123519,A,4807.038,N", s->data.data(), s->length);
}

// ---------------------------------------------------------------------------
// Config command builders
// ---------------------------------------------------------------------------

namespace {
using septentrio_gnss::Connection;
using septentrio_gnss::GnssAttitudeMode;
using septentrio_gnss::SbfBlock;
using septentrio_gnss::SbfInterval;
using septentrio_gnss::SbfStream;
using septentrio_gnss::set_gnss_attitude;
using septentrio_gnss::set_sbf_output;

constexpr std::array<SbfBlock, 2> kAttBlocks {SbfBlock::AttEuler,
                                              SbfBlock::AttCovEuler};

// Framing is pinned at compile time; the runtime tests below just exercise
// the same builders through the suite.
constexpr auto kAttitudeCmd = set_gnss_attitude(GnssAttitudeMode::MultiAntenna);
static_assert(kAttitudeCmd.has_value());
static_assert(kAttitudeCmd->view() == "setGNSSAttitude,MultiAntenna\r");

constexpr auto kSbfOutputCmd = set_sbf_output(
    SbfStream::Stream1, Connection::COM1, kAttBlocks, SbfInterval::Msec100);
static_assert(kSbfOutputCmd.has_value());
static_assert(kSbfOutputCmd->view() ==
              "setSBFOutput,Stream1,COM1,AttEuler+AttCovEuler,msec100\r");
} // namespace

/** @brief setGNSSAttitude selects the multi-antenna attitude source. */
void test_set_gnss_attitude_multi_antenna() {
  auto cmd = set_gnss_attitude(GnssAttitudeMode::MultiAntenna);
  TEST_ASSERT_TRUE(cmd.has_value());
  TEST_ASSERT_TRUE(cmd->view() == "setGNSSAttitude,MultiAntenna\r");
}

/** @brief setSBFOutput joins the requested blocks with '+'. */
void test_set_sbf_output_joins_blocks() {
  auto cmd = set_sbf_output(
      SbfStream::Stream1, Connection::COM1, kAttBlocks, SbfInterval::Msec100);
  TEST_ASSERT_TRUE(cmd.has_value());
  TEST_ASSERT_TRUE(cmd->view() ==
                   "setSBFOutput,Stream1,COM1,AttEuler+AttCovEuler,msec100\r");
}

/** @brief setAttitudeOffset formats its float arguments. */
void test_set_attitude_offset_formats_floats() {
  auto cmd = septentrio_gnss::set_attitude_offset(0.0f, -1.5f);
  TEST_ASSERT_TRUE(cmd.has_value());
  TEST_ASSERT_TRUE(cmd->view() == "setAttitudeOffset,0,-1.5\r");
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
  RUN_TEST(test_garbage_then_sbf_recovers);
  RUN_TEST(test_repeated_sync1_then_sync2_starts_frame);
  RUN_TEST(test_bad_crc_rejected_then_resyncs);
  RUN_TEST(test_length_not_multiple_of_4_rejected);
  RUN_TEST(test_length_below_header_rejected);
  RUN_TEST(test_length_exceeds_max_rejected);
  RUN_TEST(test_timeout_drops_stalled_block);
  RUN_TEST(test_no_timeout_when_under_threshold);
  RUN_TEST(test_parse_pvt_geodetic_round_trip);
  RUN_TEST(test_parse_pvt_geodetic_wrong_block_number);
  RUN_TEST(test_parse_pvt_geodetic_short_body_rejected);
  RUN_TEST(test_parse_pvt_geodetic_forward_compat);
  RUN_TEST(test_parse_pvt_geodetic_dnu_preserved);
  RUN_TEST(test_parse_pos_cov_geodetic_round_trip);
  RUN_TEST(test_parse_vel_cov_geodetic_round_trip);
  RUN_TEST(test_parse_att_euler_round_trip);
  RUN_TEST(test_parse_att_cov_euler_round_trip);
  RUN_TEST(test_parse_clean_nmea);
  RUN_TEST(test_nmea_bad_checksum_rejected);
  RUN_TEST(test_nmea_then_sbf_interleaved);
  RUN_TEST(test_dollar_mid_nmea_then_sbf_dispatches);
  RUN_TEST(test_command_build_appends_cr);
  RUN_TEST(test_command_build_rejects_oversize);
  RUN_TEST(test_set_gnss_attitude_multi_antenna);
  RUN_TEST(test_set_sbf_output_joins_blocks);
  RUN_TEST(test_set_attitude_offset_formats_floats);
  RUN_TEST(test_parse_clean_reply_ok);
  RUN_TEST(test_parse_reply_err);
  RUN_TEST(test_parse_reply_info);
  RUN_TEST(test_dollar_R_without_kind_char_is_nmea);
  return UNITY_END();
}
