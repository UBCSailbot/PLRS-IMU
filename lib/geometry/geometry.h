/**
 * Shared geometric primitives.
 *
 * Plain PODs only. Layered libraries (xbus, fusion) define their own
 * invariant-bearing wrappers (e.g. UnitQuaternion) on top of these.
 *
 * No Arduino or FreeRTOS dependency. Host-compilable.
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
 * No unit-norm invariant; for rotation-bearing storage use the
 * UnitQuaternion wrapper in the consuming library.
 */
struct Quaternion {
  float w;
  float x;
  float y;
  float z;
};

} // namespace plrs
