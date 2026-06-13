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
  return PVTGeodetic {
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

/*
 * PosCovGeodetic, block 5906, mosaic-X5 Reference Guide section 4.2.10.
 */

namespace pos_cov_geodetic_layout {
constexpr uint16_t BLOCK_NUMBER = 5906;
constexpr std::size_t TOW = 0;
constexpr std::size_t WNC = 4;
constexpr std::size_t MODE = 6;
constexpr std::size_t ERROR = 7;
constexpr std::size_t COV_LATLAT = 8;
constexpr std::size_t COV_LONLON = 12;
constexpr std::size_t COV_HGTHGT = 16;
constexpr std::size_t COV_BB = 20;
constexpr std::size_t COV_LATLON = 24;
constexpr std::size_t COV_LATHGT = 28;
constexpr std::size_t COV_LATB = 32;
constexpr std::size_t COV_LONHGT = 36;
constexpr std::size_t COV_LONB = 40;
constexpr std::size_t COV_HB = 44;
constexpr std::size_t MIN_BODY = COV_HB + sizeof(float);
} // namespace pos_cov_geodetic_layout

/**
 * Position variance-covariance matrix in geodetic coordinates.
 * Spec: section 4.2.10, block 5906. All units are m^2.
 */
struct PosCovGeodetic {
  uint32_t tow;     // 0.001 s, DNU = DNU_U4
  uint16_t wnc;     // weeks, DNU = DNU_U2
  uint8_t mode;     // bit field, PVT mode and flags
  uint8_t error;    // error code, 0 = no error
  float cov_latlat; // m^2, DNU = DNU_F4
  float cov_lonlon; // m^2, DNU = DNU_F4
  float cov_hgthgt; // m^2, DNU = DNU_F4
  float cov_bb;     // m^2, clock-bias variance, DNU = DNU_F4
  float cov_latlon; // m^2, DNU = DNU_F4
  float cov_lathgt; // m^2, DNU = DNU_F4
  float cov_latb;   // m^2, DNU = DNU_F4
  float cov_lonhgt; // m^2, DNU = DNU_F4
  float cov_lonb;   // m^2, DNU = DNU_F4
  float cov_hb;     // m^2, DNU = DNU_F4
};

/**
 * @brief Decode a PosCovGeodetic block from a parsed Packet.
 *
 * @param p The packet whose block_number must equal 5906.
 *
 * @return The decoded struct, or nullopt on block-number mismatch or
 *         short body.
 */
inline std::optional<PosCovGeodetic> parse_pos_cov_geodetic(const Packet &p) {
  namespace o = pos_cov_geodetic_layout;
  if (p.block_number() != o::BLOCK_NUMBER) {
    return std::nullopt;
  }
  if (p.body_length < o::MIN_BODY) {
    return std::nullopt;
  }
  const ByteSpan b = p.body();
  return PosCovGeodetic {
      .tow = read_little_endian<uint32_t>(b, o::TOW),
      .wnc = read_little_endian<uint16_t>(b, o::WNC),
      .mode = b[o::MODE],
      .error = b[o::ERROR],
      .cov_latlat = read_little_endian<float>(b, o::COV_LATLAT),
      .cov_lonlon = read_little_endian<float>(b, o::COV_LONLON),
      .cov_hgthgt = read_little_endian<float>(b, o::COV_HGTHGT),
      .cov_bb = read_little_endian<float>(b, o::COV_BB),
      .cov_latlon = read_little_endian<float>(b, o::COV_LATLON),
      .cov_lathgt = read_little_endian<float>(b, o::COV_LATHGT),
      .cov_latb = read_little_endian<float>(b, o::COV_LATB),
      .cov_lonhgt = read_little_endian<float>(b, o::COV_LONHGT),
      .cov_lonb = read_little_endian<float>(b, o::COV_LONB),
      .cov_hb = read_little_endian<float>(b, o::COV_HB),
  };
}

/*
 * VelCovGeodetic, block 5908, mosaic-X5 Reference Guide section 4.2.10.
 */

namespace vel_cov_geodetic_layout {
constexpr uint16_t BLOCK_NUMBER = 5908;
constexpr std::size_t TOW = 0;
constexpr std::size_t WNC = 4;
constexpr std::size_t MODE = 6;
constexpr std::size_t ERROR = 7;
constexpr std::size_t COV_VNVN = 8;
constexpr std::size_t COV_VEVE = 12;
constexpr std::size_t COV_VUVU = 16;
constexpr std::size_t COV_DTDT = 20;
constexpr std::size_t COV_VNVE = 24;
constexpr std::size_t COV_VNVU = 28;
constexpr std::size_t COV_VNDT = 32;
constexpr std::size_t COV_VEVU = 36;
constexpr std::size_t COV_VEDT = 40;
constexpr std::size_t COV_VUDT = 44;
constexpr std::size_t MIN_BODY = COV_VUDT + sizeof(float);
} // namespace vel_cov_geodetic_layout

/**
 * Velocity variance-covariance matrix in geodetic (NEU) coordinates.
 * Spec: section 4.2.10, block 5908. All units are m^2/s^2.
 */
struct VelCovGeodetic {
  uint32_t tow;   // 0.001 s, DNU = DNU_U4
  uint16_t wnc;   // weeks, DNU = DNU_U2
  uint8_t mode;   // bit field, PVT mode and flags
  uint8_t error;  // error code, 0 = no error
  float cov_vnvn; // (m/s)^2, north velocity variance, DNU = DNU_F4
  float cov_veve; // (m/s)^2, east velocity variance, DNU = DNU_F4
  float cov_vuvu; // (m/s)^2, up velocity variance, DNU = DNU_F4
  float cov_dtdt; // (m/s)^2, clock drift variance, DNU = DNU_F4
  float cov_vnve; // (m/s)^2, DNU = DNU_F4
  float cov_vnvu; // (m/s)^2, DNU = DNU_F4
  float cov_vndt; // (m/s)^2, DNU = DNU_F4
  float cov_vevu; // (m/s)^2, DNU = DNU_F4
  float cov_vedt; // (m/s)^2, DNU = DNU_F4
  float cov_vudt; // (m/s)^2, DNU = DNU_F4
};

/**
 * @brief Decode a VelCovGeodetic block from a parsed Packet.
 *
 * @param p The packet whose block_number must equal 5908.
 *
 * @return The decoded struct, or nullopt on block-number mismatch or
 *         short body.
 */
inline std::optional<VelCovGeodetic> parse_vel_cov_geodetic(const Packet &p) {
  namespace o = vel_cov_geodetic_layout;
  if (p.block_number() != o::BLOCK_NUMBER) {
    return std::nullopt;
  }
  if (p.body_length < o::MIN_BODY) {
    return std::nullopt;
  }
  const ByteSpan b = p.body();
  return VelCovGeodetic {
      .tow = read_little_endian<uint32_t>(b, o::TOW),
      .wnc = read_little_endian<uint16_t>(b, o::WNC),
      .mode = b[o::MODE],
      .error = b[o::ERROR],
      .cov_vnvn = read_little_endian<float>(b, o::COV_VNVN),
      .cov_veve = read_little_endian<float>(b, o::COV_VEVE),
      .cov_vuvu = read_little_endian<float>(b, o::COV_VUVU),
      .cov_dtdt = read_little_endian<float>(b, o::COV_DTDT),
      .cov_vnve = read_little_endian<float>(b, o::COV_VNVE),
      .cov_vnvu = read_little_endian<float>(b, o::COV_VNVU),
      .cov_vndt = read_little_endian<float>(b, o::COV_VNDT),
      .cov_vevu = read_little_endian<float>(b, o::COV_VEVU),
      .cov_vedt = read_little_endian<float>(b, o::COV_VEDT),
      .cov_vudt = read_little_endian<float>(b, o::COV_VUDT),
  };
}

/*
 * AttEuler, block 5938, mosaic-X5 Reference Guide section 4.2.11.
 */

namespace att_euler_layout {
constexpr uint16_t BLOCK_NUMBER = 5938;
constexpr std::size_t TOW = 0;
constexpr std::size_t WNC = 4;
constexpr std::size_t NR_SV = 6;
constexpr std::size_t ERROR = 7;
constexpr std::size_t MODE = 8;
constexpr std::size_t RESERVED = 10;
constexpr std::size_t HEADING = 12;
constexpr std::size_t PITCH = 16;
constexpr std::size_t ROLL = 20;
constexpr std::size_t PITCH_DOT = 24;
constexpr std::size_t ROLL_DOT = 28;
constexpr std::size_t HEADING_DOT = 32;
constexpr std::size_t MIN_BODY = HEADING_DOT + sizeof(float);
} // namespace att_euler_layout

/**
 * Euler attitude (heading, pitch, roll) and their rates from the dual-
 * antenna baseline. Spec: section 4.2.11, block 5938. Angles in degrees.
 *
 * Note that on the mosaic-go-H, heading is the load-bearing output (dual
 * antenna baseline); roll is only available if a third antenna is fitted
 * and otherwise sits at DNU.
 */
struct AttEuler {
  uint32_t tow;  // 0.001 s, DNU = DNU_U4
  uint16_t wnc;  // weeks, DNU = DNU_U2
  uint8_t nr_sv; // satellites used (average over antennas), DNU = DNU_U1
  uint8_t error; // bit field, per-baseline error codes
  uint16_t mode; // attitude mode code (heading/pitch/roll availability)
  uint16_t reserved;
  float heading;     // degrees, DNU = DNU_F4
  float pitch;       // degrees, DNU = DNU_F4
  float roll;        // degrees, DNU = DNU_F4 (or always DNU if 2-antenna)
  float pitch_dot;   // degrees/s, DNU = DNU_F4
  float roll_dot;    // degrees/s, DNU = DNU_F4
  float heading_dot; // degrees/s, DNU = DNU_F4
};

/**
 * @brief Decode an AttEuler block from a parsed Packet.
 *
 * @param p The packet whose block_number must equal 5938.
 *
 * @return The decoded struct, or nullopt on block-number mismatch or
 *         short body.
 */
inline std::optional<AttEuler> parse_att_euler(const Packet &p) {
  namespace o = att_euler_layout;
  if (p.block_number() != o::BLOCK_NUMBER) {
    return std::nullopt;
  }
  if (p.body_length < o::MIN_BODY) {
    return std::nullopt;
  }
  const ByteSpan b = p.body();
  return AttEuler {
      .tow = read_little_endian<uint32_t>(b, o::TOW),
      .wnc = read_little_endian<uint16_t>(b, o::WNC),
      .nr_sv = b[o::NR_SV],
      .error = b[o::ERROR],
      .mode = read_little_endian<uint16_t>(b, o::MODE),
      .reserved = read_little_endian<uint16_t>(b, o::RESERVED),
      .heading = read_little_endian<float>(b, o::HEADING),
      .pitch = read_little_endian<float>(b, o::PITCH),
      .roll = read_little_endian<float>(b, o::ROLL),
      .pitch_dot = read_little_endian<float>(b, o::PITCH_DOT),
      .roll_dot = read_little_endian<float>(b, o::ROLL_DOT),
      .heading_dot = read_little_endian<float>(b, o::HEADING_DOT),
  };
}

/*
 * AttCovEuler, block 5939, mosaic-X5 Reference Guide section 4.2.11.
 */

namespace att_cov_euler_layout {
constexpr uint16_t BLOCK_NUMBER = 5939;
constexpr std::size_t TOW = 0;
constexpr std::size_t WNC = 4;
constexpr std::size_t RESERVED = 6;
constexpr std::size_t ERROR = 7;
constexpr std::size_t COV_HEADHEAD = 8;
constexpr std::size_t COV_PITCHPITCH = 12;
constexpr std::size_t COV_ROLLROLL = 16;
constexpr std::size_t COV_HEADPITCH = 20;
constexpr std::size_t COV_HEADROLL = 24;
constexpr std::size_t COV_PITCHROLL = 28;
constexpr std::size_t MIN_BODY = COV_PITCHROLL + sizeof(float);
} // namespace att_cov_euler_layout

/**
 * Variance-covariance matrix of the Euler attitude angles.
 * Spec: section 4.2.11, block 5939. All units are degrees^2.
 *
 * Per the spec, off-diagonal covariances are reserved future
 * functionality and are emitted as DNU_F4 by current firmware.
 */
struct AttCovEuler {
  uint32_t tow; // 0.001 s, DNU = DNU_U4
  uint16_t wnc; // weeks, DNU = DNU_U2
  uint8_t reserved;
  uint8_t error;        // bit field, per-baseline error codes
  float cov_headhead;   // degrees^2, heading variance, DNU = DNU_F4
  float cov_pitchpitch; // degrees^2, pitch variance, DNU = DNU_F4
  float cov_rollroll;   // degrees^2, roll variance, DNU = DNU_F4
  float cov_headpitch;  // degrees^2, reserved (currently DNU_F4)
  float cov_headroll;   // degrees^2, reserved (currently DNU_F4)
  float cov_pitchroll;  // degrees^2, reserved (currently DNU_F4)
};

/**
 * @brief Decode an AttCovEuler block from a parsed Packet.
 *
 * @param p The packet whose block_number must equal 5939.
 *
 * @return The decoded struct, or nullopt on block-number mismatch or
 *         short body.
 */
inline std::optional<AttCovEuler> parse_att_cov_euler(const Packet &p) {
  namespace o = att_cov_euler_layout;
  if (p.block_number() != o::BLOCK_NUMBER) {
    return std::nullopt;
  }
  if (p.body_length < o::MIN_BODY) {
    return std::nullopt;
  }
  const ByteSpan b = p.body();
  return AttCovEuler {
      .tow = read_little_endian<uint32_t>(b, o::TOW),
      .wnc = read_little_endian<uint16_t>(b, o::WNC),
      .reserved = b[o::RESERVED],
      .error = b[o::ERROR],
      .cov_headhead = read_little_endian<float>(b, o::COV_HEADHEAD),
      .cov_pitchpitch = read_little_endian<float>(b, o::COV_PITCHPITCH),
      .cov_rollroll = read_little_endian<float>(b, o::COV_ROLLROLL),
      .cov_headpitch = read_little_endian<float>(b, o::COV_HEADPITCH),
      .cov_headroll = read_little_endian<float>(b, o::COV_HEADROLL),
      .cov_pitchroll = read_little_endian<float>(b, o::COV_PITCHROLL),
  };
}

} // namespace sbf
