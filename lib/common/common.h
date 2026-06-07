/**
 * Shared geometric primitives. No Arduino or FreeRTOS dependency.
 */

#pragma once

namespace plrs {

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
