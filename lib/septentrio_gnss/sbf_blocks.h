/**
 * Typed SBF block decoders.
 *
 * Each parse_X returns nullopt if the packet's block number mismatches or
 * its body is shorter than the layout expects. Bodies longer than expected
 * (future SBF revisions) parse successfully; trailing bytes are ignored.
 * Do-Not-Use sentinels are preserved verbatim in the returned struct.
 */

#pragma once

#include "sbf_protocol.h"
#include <cstdint>
#include <optional>

namespace sbf {

/*
 * Do-Not-Use sentinel values.
 *
 * Per the spec, fields that the receiver could not populate (e.g. position
 * before fix) are set to a fixed sentinel rather than left undefined.
 * Compare with these constants to detect.
 */

constexpr uint8_t DNU_U1 = 0xFF;
constexpr uint16_t DNU_U2 = 0xFFFF;
constexpr uint32_t DNU_U4 = 0xFFFFFFFF;
constexpr float DNU_F4 = -2.0e10f;
constexpr double DNU_F8 = -2.0e10;

/*
 * PVTGeodetic, block 4007, mosaic-X5 Reference Guide section 4.2.10.
 */

namespace pvt_geodetic_layout {
constexpr uint16_t BLOCK_NUMBER = 4007;
constexpr std::size_t TOW = 0;
constexpr std::size_t WNC = 4;
constexpr std::size_t MODE = 6;
constexpr std::size_t ERROR = 7;
constexpr std::size_t LATITUDE = 8;
constexpr std::size_t LONGITUDE = 16;
constexpr std::size_t HEIGHT = 24;
constexpr std::size_t UNDULATION = 32;
constexpr std::size_t V_NORTH = 36;
constexpr std::size_t V_EAST = 40;
constexpr std::size_t V_UP = 44;
constexpr std::size_t COG = 48;
constexpr std::size_t RX_CLK_BIAS = 52;
constexpr std::size_t RX_CLK_DRIFT = 60;
constexpr std::size_t TIME_SYSTEM = 64;
constexpr std::size_t DATUM = 65;
constexpr std::size_t NR_SV = 66;
constexpr std::size_t WA_CORR_INFO = 67;
constexpr std::size_t REFERENCE_ID = 68;
constexpr std::size_t MEAN_CORR_AGE = 70;
constexpr std::size_t SIGNAL_INFO = 72;
constexpr std::size_t ALERT_FLAG = 76;
constexpr std::size_t NR_BASES = 77;
constexpr std::size_t PPP_INFO = 78;
constexpr std::size_t LATENCY = 80;
constexpr std::size_t H_ACCURACY = 82;
constexpr std::size_t V_ACCURACY = 84;
constexpr std::size_t MISC = 86;
constexpr std::size_t MIN_BODY = MISC + sizeof(uint8_t);
} // namespace pvt_geodetic_layout

/**
 * PVT solution in geodetic coordinates plus receiver clock state.
 * Spec: section 4.2.10, block 4007.
 *
 * Field units and Do-Not-Use values come straight from the spec. Compare
 * with DNU_F8 / DNU_F4 / DNU_U2 / DNU_U4 to detect unavailable fields.
 */
struct PVTGeodetic {
  uint32_t tow;           // 0.001 s, DNU = DNU_U4
  uint16_t wnc;           // weeks, DNU = DNU_U2
  uint8_t mode;           // bit field, PVT mode and flags
  uint8_t error;          // error code, 0 = no error
  double latitude;        // rad, [-pi/2, pi/2], DNU = DNU_F8
  double longitude;       // rad, [-pi, pi], DNU = DNU_F8
  double height;          // m, ellipsoidal, DNU = DNU_F8
  float undulation;       // m, geoid undulation, DNU = DNU_F4
  float v_north;          // m/s, DNU = DNU_F4
  float v_east;           // m/s, DNU = DNU_F4
  float v_up;             // m/s, DNU = DNU_F4
  float cog;              // degrees, course over ground, DNU = DNU_F4
  double rx_clk_bias;     // ms, DNU = DNU_F8
  float rx_clk_drift;     // ppm, DNU = DNU_F4
  uint8_t time_system;    // time system identifier, DNU = DNU_U1
  uint8_t datum;          // datum identifier, DNU = DNU_U1
  uint8_t nr_sv;          // satellites used, DNU = DNU_U1
  uint8_t wa_corr_info;   // bit field, applied corrections
  uint16_t reference_id;  // base station or geostationary satellite ID
  uint16_t mean_corr_age; // 0.01 s, DNU = DNU_U2
  uint32_t signal_info;   // bit field, signal types used
  uint8_t alert_flag;     // bit field, integrity information
  uint8_t nr_bases;       // number of base stations used
  uint16_t ppp_info;      // bit field, PPP-related info (Rev 1+)
  uint16_t latency;       // 0.0001 s, processing latency (Rev 2+), DNU = DNU_U2
  uint16_t h_accuracy;    // 0.01 m, 2DRMS horizontal accuracy (Rev 2+)
  uint16_t v_accuracy;    // 0.01 m, 2-sigma vertical accuracy (Rev 2+)
  uint8_t misc;           // bit field, miscellaneous flags (Rev 2+)
};

/**
 * @brief Decode a PVTGeodetic block from a parsed Packet.
 *
 * @param p The packet whose block_number must equal 4007.
 *
 * @return The decoded struct, or nullopt if the block number mismatches or
 *         the body is shorter than the full Rev 2 layout.
 */
inline std::optional<PVTGeodetic> parse_pvt_geodetic(const Packet &p) {
  namespace o = pvt_geodetic_layout;
  if (p.block_number() != o::BLOCK_NUMBER) {
    return std::nullopt;
  }
  if (p.body_length < o::MIN_BODY) {
    return std::nullopt;
  }
  const ByteSpan b = p.body();
  return PVTGeodetic{
      .tow = read_little_endian<uint32_t>(b, o::TOW),
      .wnc = read_little_endian<uint16_t>(b, o::WNC),
      .mode = b[o::MODE],
      .error = b[o::ERROR],
      .latitude = read_little_endian<double>(b, o::LATITUDE),
      .longitude = read_little_endian<double>(b, o::LONGITUDE),
      .height = read_little_endian<double>(b, o::HEIGHT),
      .undulation = read_little_endian<float>(b, o::UNDULATION),
      .v_north = read_little_endian<float>(b, o::V_NORTH),
      .v_east = read_little_endian<float>(b, o::V_EAST),
      .v_up = read_little_endian<float>(b, o::V_UP),
      .cog = read_little_endian<float>(b, o::COG),
      .rx_clk_bias = read_little_endian<double>(b, o::RX_CLK_BIAS),
      .rx_clk_drift = read_little_endian<float>(b, o::RX_CLK_DRIFT),
      .time_system = b[o::TIME_SYSTEM],
      .datum = b[o::DATUM],
      .nr_sv = b[o::NR_SV],
      .wa_corr_info = b[o::WA_CORR_INFO],
      .reference_id = read_little_endian<uint16_t>(b, o::REFERENCE_ID),
      .mean_corr_age = read_little_endian<uint16_t>(b, o::MEAN_CORR_AGE),
      .signal_info = read_little_endian<uint32_t>(b, o::SIGNAL_INFO),
      .alert_flag = b[o::ALERT_FLAG],
      .nr_bases = b[o::NR_BASES],
      .ppp_info = read_little_endian<uint16_t>(b, o::PPP_INFO),
      .latency = read_little_endian<uint16_t>(b, o::LATENCY),
      .h_accuracy = read_little_endian<uint16_t>(b, o::H_ACCURACY),
      .v_accuracy = read_little_endian<uint16_t>(b, o::V_ACCURACY),
      .misc = b[o::MISC],
  };
}

} // namespace sbf
