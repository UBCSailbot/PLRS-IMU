/**
 * Shared geometric primitives. No Arduino or FreeRTOS dependency.
 */

#pragma once

#include <array>
#include <bit>
#include <cstdint>
#include <span>

namespace plrs {

using ByteSpan = std::span<const uint8_t>;

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

/**
 * Helper functions for Big Endian byte operations.
 */
constexpr uint16_t read_u16_big_endian(ByteSpan b) {
  return static_cast<uint16_t>((b[0] << 8) | b[1]);
}

constexpr std::array<uint8_t, 2> write_u16_big_endian(uint16_t v) {
  return {static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v & 0xFF)};
}

constexpr float read_f32_big_endian(ByteSpan b) {
  uint32_t u = (uint32_t(b[0] << 24) | (uint32_t(b[1]) << 16) |
                uint32_t(b[2]) << 8 | uint32_t(b[3]));
  return std::bit_cast<float>(u);
}

} // namespace plrs