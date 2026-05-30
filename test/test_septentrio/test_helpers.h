/**
 * Test-only block-building helpers. Not part of the driver.
 *
 * The CRC here is table-driven and written independently of the production
 * bitwise crc_ccitt() so that a bug in one cannot mask a bug in the other.
 */

#pragma once
#include "sbf_blocks.h"
#include "sbf_protocol.h"
#include <array>
#include <bit>
#include <cstdint>
#include <vector>

namespace stest {

/**
 * @brief Append a trivially-copyable value to `v` in little-endian order.
 *        Used by block-body builders to deposit u16/u32/u64/f32/f64 fields.
 */
template <typename T> inline void push_le(std::vector<uint8_t> &v, T x) {
  const auto bytes = std::bit_cast<std::array<std::byte, sizeof(T)>>(x);
  for (auto b : bytes) {
    v.push_back(static_cast<uint8_t>(b));
  }
}

namespace detail {

constexpr std::array<uint16_t, 256> build_crc_table() {
  std::array<uint16_t, 256> t{};
  for (uint16_t i = 0; i < 256; i++) {
    uint16_t c = static_cast<uint16_t>(i << 8);
    for (int j = 0; j < 8; j++) {
      c = (c & 0x8000) ? static_cast<uint16_t>((c << 1) ^ 0x1021)
                       : static_cast<uint16_t>(c << 1);
    }
    t[i] = c;
  }
  return t;
}

inline uint16_t crc_ccitt_xmodem(const std::vector<uint8_t> &data) {
  static constexpr auto TABLE = build_crc_table();
  uint16_t crc = 0;
  for (uint8_t b : data) {
    crc = static_cast<uint16_t>((crc << 8) ^ TABLE[(crc >> 8) ^ b]);
  }
  return crc;
}

} // namespace detail

/**
 * @brief Assemble a complete SBF block: sync + crc + id + length + body.
 *
 * Body is taken verbatim — caller is responsible for any padding to make
 * length a multiple of 4, so misalignment tests can pass unpadded bodies
 * deliberately.
 *
 * @param id    Full 16-bit ID field (block number in bits 0-12, revision
 *              in bits 13-15).
 * @param body  Block body bytes (zero or more).
 *
 * @return The wire bytes of the block.
 */
inline std::vector<uint8_t> make_block(uint16_t id,
                                       const std::vector<uint8_t> &body) {
  const uint16_t length = static_cast<uint16_t>(sbf::HEADER_LEN + body.size());

  std::vector<uint8_t> crc_input;
  crc_input.push_back(static_cast<uint8_t>(id & 0xFF));
  crc_input.push_back(static_cast<uint8_t>(id >> 8));
  crc_input.push_back(static_cast<uint8_t>(length & 0xFF));
  crc_input.push_back(static_cast<uint8_t>(length >> 8));
  for (uint8_t b : body)
    crc_input.push_back(b);
  const uint16_t crc = detail::crc_ccitt_xmodem(crc_input);

  std::vector<uint8_t> f;
  f.push_back(sbf::SYNC1);
  f.push_back(sbf::SYNC2);
  f.push_back(static_cast<uint8_t>(crc & 0xFF));
  f.push_back(static_cast<uint8_t>(crc >> 8));
  for (uint8_t b : crc_input)
    f.push_back(b);
  return f;
}

/**
 * @brief Variant of make_block that lets the caller force a specific length
 *        field, even if it mismatches body.size(). Used to test the parser's
 *        length-validation paths (not multiple of 4, < HEADER_LEN, >
 * MAX_BLOCK).
 */
inline std::vector<uint8_t>
make_block_with_length(uint16_t id, uint16_t length,
                       const std::vector<uint8_t> &body) {
  std::vector<uint8_t> crc_input;
  crc_input.push_back(static_cast<uint8_t>(id & 0xFF));
  crc_input.push_back(static_cast<uint8_t>(id >> 8));
  crc_input.push_back(static_cast<uint8_t>(length & 0xFF));
  crc_input.push_back(static_cast<uint8_t>(length >> 8));
  for (uint8_t b : body)
    crc_input.push_back(b);
  const uint16_t crc = detail::crc_ccitt_xmodem(crc_input);

  std::vector<uint8_t> f;
  f.push_back(sbf::SYNC1);
  f.push_back(sbf::SYNC2);
  f.push_back(static_cast<uint8_t>(crc & 0xFF));
  f.push_back(static_cast<uint8_t>(crc >> 8));
  for (uint8_t b : crc_input)
    f.push_back(b);
  return f;
}

/**
 * @brief Serialize a PVTGeodetic struct into its on-wire body bytes.
 *        Field order must mirror sbf::pvt_geodetic_layout exactly; a wrong
 *        offset on the production side shows up as a swapped field.
 */
inline std::vector<uint8_t> make_pvt_geodetic_body(const sbf::PVTGeodetic &v) {
  std::vector<uint8_t> b;
  push_le(b, v.tow);
  push_le(b, v.wnc);
  b.push_back(v.mode);
  b.push_back(v.error);
  push_le(b, v.latitude);
  push_le(b, v.longitude);
  push_le(b, v.height);
  push_le(b, v.undulation);
  push_le(b, v.v_north);
  push_le(b, v.v_east);
  push_le(b, v.v_up);
  push_le(b, v.cog);
  push_le(b, v.rx_clk_bias);
  push_le(b, v.rx_clk_drift);
  b.push_back(v.time_system);
  b.push_back(v.datum);
  b.push_back(v.nr_sv);
  b.push_back(v.wa_corr_info);
  push_le(b, v.reference_id);
  push_le(b, v.mean_corr_age);
  push_le(b, v.signal_info);
  b.push_back(v.alert_flag);
  b.push_back(v.nr_bases);
  push_le(b, v.ppp_info);
  push_le(b, v.latency);
  push_le(b, v.h_accuracy);
  push_le(b, v.v_accuracy);
  b.push_back(v.misc);
  return b;
}

inline std::vector<uint8_t>
make_pos_cov_geodetic_body(const sbf::PosCovGeodetic &v) {
  std::vector<uint8_t> b;
  push_le(b, v.tow);
  push_le(b, v.wnc);
  b.push_back(v.mode);
  b.push_back(v.error);
  push_le(b, v.cov_latlat);
  push_le(b, v.cov_lonlon);
  push_le(b, v.cov_hgthgt);
  push_le(b, v.cov_bb);
  push_le(b, v.cov_latlon);
  push_le(b, v.cov_lathgt);
  push_le(b, v.cov_latb);
  push_le(b, v.cov_lonhgt);
  push_le(b, v.cov_lonb);
  push_le(b, v.cov_hb);
  return b;
}

inline std::vector<uint8_t>
make_vel_cov_geodetic_body(const sbf::VelCovGeodetic &v) {
  std::vector<uint8_t> b;
  push_le(b, v.tow);
  push_le(b, v.wnc);
  b.push_back(v.mode);
  b.push_back(v.error);
  push_le(b, v.cov_vnvn);
  push_le(b, v.cov_veve);
  push_le(b, v.cov_vuvu);
  push_le(b, v.cov_dtdt);
  push_le(b, v.cov_vnve);
  push_le(b, v.cov_vnvu);
  push_le(b, v.cov_vndt);
  push_le(b, v.cov_vevu);
  push_le(b, v.cov_vedt);
  push_le(b, v.cov_vudt);
  return b;
}

inline std::vector<uint8_t> make_att_euler_body(const sbf::AttEuler &v) {
  std::vector<uint8_t> b;
  push_le(b, v.tow);
  push_le(b, v.wnc);
  b.push_back(v.nr_sv);
  b.push_back(v.error);
  push_le(b, v.mode);
  push_le(b, v.reserved);
  push_le(b, v.heading);
  push_le(b, v.pitch);
  push_le(b, v.roll);
  push_le(b, v.pitch_dot);
  push_le(b, v.roll_dot);
  push_le(b, v.heading_dot);
  return b;
}

inline std::vector<uint8_t> make_att_cov_euler_body(const sbf::AttCovEuler &v) {
  std::vector<uint8_t> b;
  push_le(b, v.tow);
  push_le(b, v.wnc);
  b.push_back(v.reserved);
  b.push_back(v.error);
  push_le(b, v.cov_headhead);
  push_le(b, v.cov_pitchpitch);
  push_le(b, v.cov_rollroll);
  push_le(b, v.cov_headpitch);
  push_le(b, v.cov_headroll);
  push_le(b, v.cov_pitchroll);
  return b;
}

} // namespace stest
