/**
 * Shared primitives: geometry and little-endian byte access. No Arduino or
 * FreeRTOS dependency.
 */

#pragma once

#include <array>
#include <bit>
#include <cstdint>
#include <span>

namespace plrs {

using ByteSpan = std::span<const uint8_t>;

/*
 * Little-endian byte access.
 */

constexpr uint16_t read_u16_little_endian(ByteSpan b) {
  return static_cast<uint16_t>(b[0] | (b[1] << 8));
}

constexpr std::array<uint8_t, 2> write_u16_little_endian(uint16_t v) {
  return {static_cast<uint8_t>(v & 0xFF), static_cast<uint8_t>(v >> 8)};
}

constexpr float read_f32_little_endian(ByteSpan b) {
  uint32_t u = uint32_t(b[0]) | (uint32_t(b[1]) << 8) | (uint32_t(b[2]) << 16) |
               (uint32_t(b[3]) << 24);
  return std::bit_cast<float>(u);
}

constexpr std::array<uint8_t, 4> write_f32_little_endian(float f) {
  uint32_t u = std::bit_cast<uint32_t>(f);
  return {static_cast<uint8_t>(u & 0xFF),
          static_cast<uint8_t>((u >> 8) & 0xFF),
          static_cast<uint8_t>((u >> 16) & 0xFF),
          static_cast<uint8_t>(u >> 24)};
}

/**
 * Three-component vector.
 */
struct Vec3 {
  float x;
  float y;
  float z;
};

/**
 * Quaternion in {w, x, y, z} order (Xsens / Eigen convention).
 *
 * Not required to be unit-norm.
 */
struct Quaternion {
  float w;
  float x;
  float y;
  float z;
};

} // namespace plrs
