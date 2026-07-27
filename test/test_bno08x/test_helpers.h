/**
 * Test-only cargo/report builders. Not part of the driver.
 *
 * Built by hand (not via encode_packet / build_set_feature) so a bug in the
 * driver's encoders cannot mask a bug in its decoders.
 */

#pragma once
#include <cmath>
#include <cstdint>
#include <vector>

namespace btest {

// Two little-endian bytes of a signed 16-bit sample.
inline std::vector<uint8_t> s16_le(int16_t v) {
  auto u = static_cast<uint16_t>(v);
  return {static_cast<uint8_t>(u & 0xFF), static_cast<uint8_t>(u >> 8)};
}

// Quantize a float to a Q-format raw int16 (the inverse of q_to_float).
inline int16_t q_raw(float v, int qpoint) {
  return static_cast<int16_t>(std::lround(v * static_cast<float>(1 << qpoint)));
}

inline std::vector<uint8_t> q_le(float v, int qpoint) {
  return s16_le(q_raw(v, qpoint));
}

// One SHTP cargo: 4-byte header (length incl. header, LE) + payload.
inline std::vector<uint8_t>
make_cargo(uint8_t channel, uint8_t seq, const std::vector<uint8_t> &payload) {
  uint16_t total = static_cast<uint16_t>(4 + payload.size());
  std::vector<uint8_t> c {static_cast<uint8_t>(total & 0xFF),
                          static_cast<uint8_t>(total >> 8),
                          channel,
                          seq};
  c.insert(c.end(), payload.begin(), payload.end());
  return c;
}

// The 5-byte Base-Timestamp report that leads every batch.
inline std::vector<uint8_t> base_timestamp() {
  return {0xFB, 0x00, 0x00, 0x00, 0x00};
}

// One sensor report: 4-byte header (id, seq, status, delay) + data.
inline std::vector<uint8_t> make_report(uint8_t id,
                                        const std::vector<uint8_t> &data) {
  std::vector<uint8_t> r {id, 0x00, 0x00, 0x00};
  r.insert(r.end(), data.begin(), data.end());
  return r;
}

inline std::vector<uint8_t> vec3_report(uint8_t id,
                                        float x,
                                        float y,
                                        float z,
                                        int qpoint) {
  std::vector<uint8_t> data;
  for (float v : {x, y, z}) {
    auto b = q_le(v, qpoint);
    data.insert(data.end(), b.begin(), b.end());
  }
  return make_report(id, data);
}

// Rotation Vector report: i, j, k, real, accuracy (Q14 quat, Q12 accuracy).
inline std::vector<uint8_t>
rotation_report(float w, float x, float y, float z) {
  std::vector<uint8_t> data;
  for (float v : {x, y, z, w}) { // wire order i, j, k, real
    auto b = q_le(v, 14);
    data.insert(data.end(), b.begin(), b.end());
  }
  auto acc = q_le(0.0f, 12);
  data.insert(data.end(), acc.begin(), acc.end());
  return make_report(0x05, data);
}

inline void append(std::vector<uint8_t> &dst, const std::vector<uint8_t> &src) {
  dst.insert(dst.end(), src.begin(), src.end());
}

} // namespace btest
