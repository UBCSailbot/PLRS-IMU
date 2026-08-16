/**
 * Persist the GNSS-learned mag offset across reboots.
 *
 * The EKF's mag_offset (declination + boat iron + frame constant) is learned
 * from GNSS during a run; seeding it at boot lets heading anchor to the mag
 * immediately after a reset instead of coasting to the static tuning default
 * (see ekf_filter.h MtiYawConfig::offset_seed_deg). This is the Arduino-free
 * half: the blob format and the write policy. The flash I/O is firmware-side
 * (src/offset_store_eeprom.*).
 */

#pragma once

#include "common.h"
#include "fusion.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace offset_store {

using plrs::ByteSpan;

// Identifies our blob in the flash sector and versions its layout, so a stale
// or foreign sector (or a format change) is rejected rather than trusted.
// Bump VERSION whenever the meaning of the stored offset changes, not only its
// layout: a persisted offset is only valid for the frame it was learned in, so
// an IMU mount or antenna baseline change must invalidate it. Version 2 is the
// 2026-08-16 recalibration to boat-forward.
constexpr uint32_t MAGIC = 0x4d414730; // "MAG0"
constexpr uint16_t VERSION = 2;

// A mag offset outside this is not a physical heading offset; reject on load.
constexpr float OFFSET_LIMIT_DEG = 180.0f;

/**
 * On-flash record: magic, version, the offset, and a CRC over the preceding
 * bytes. Packed layout so the byte serialization is explicit and stable.
 */
struct StoredOffset {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved; // pad to a 4-byte boundary; keep zero
  float offset_deg;
  uint32_t crc;
};

constexpr std::size_t BLOB_BYTES = 16; // 4 + 2 + 2 + 4 + 4
constexpr std::size_t CRC_OFFSET = 12; // bytes covered by the CRC precede it

/**
 * @brief CRC-32 (IEEE 802.3, reflected) over a byte span.
 *
 * Bit-at-a-time so it stays constexpr and dependency-free; the blob is tiny.
 */
constexpr uint32_t crc32(ByteSpan bytes) {
  uint32_t crc = 0xffffffffu;
  for (const uint8_t byte : bytes) {
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
      crc = (crc >> 1) ^ (0xedb88320u & (~(crc & 1u) + 1u));
    }
  }
  return ~crc;
}

/**
 * @brief Serialize an offset into the 16-byte flash blob (little endian).
 */
inline std::array<uint8_t, BLOB_BYTES> encode(float offset_deg) {
  std::array<uint8_t, BLOB_BYTES> out {};
  out[0] = static_cast<uint8_t>(MAGIC & 0xff);
  out[1] = static_cast<uint8_t>((MAGIC >> 8) & 0xff);
  out[2] = static_cast<uint8_t>((MAGIC >> 16) & 0xff);
  out[3] = static_cast<uint8_t>((MAGIC >> 24) & 0xff);
  const auto ver = plrs::write_u16_little_endian(VERSION);
  out[4] = ver[0];
  out[5] = ver[1];
  out[6] = 0; // reserved
  out[7] = 0;
  const auto off = plrs::write_f32_little_endian(offset_deg);
  out[8] = off[0];
  out[9] = off[1];
  out[10] = off[2];
  out[11] = off[3];
  const uint32_t crc = crc32(ByteSpan(out.data(), CRC_OFFSET));
  out[12] = static_cast<uint8_t>(crc & 0xff);
  out[13] = static_cast<uint8_t>((crc >> 8) & 0xff);
  out[14] = static_cast<uint8_t>((crc >> 16) & 0xff);
  out[15] = static_cast<uint8_t>((crc >> 24) & 0xff);
  return out;
}

/**
 * @brief Validate a flash blob and recover the stored offset.
 *
 * @param bytes  Raw sector bytes (at least BLOB_BYTES).
 *
 * @return The offset, or nullopt if the blob is too short, has the wrong magic
 *   or version, fails its CRC, or holds a non-finite / out-of-range value (an
 *   erased or foreign sector, so the caller falls back to the static seed).
 */
inline std::optional<float> validate(ByteSpan bytes) {
  if (bytes.size() < BLOB_BYTES) {
    return std::nullopt;
  }
  const uint32_t magic = static_cast<uint32_t>(bytes[0]) |
                         (static_cast<uint32_t>(bytes[1]) << 8) |
                         (static_cast<uint32_t>(bytes[2]) << 16) |
                         (static_cast<uint32_t>(bytes[3]) << 24);
  if (magic != MAGIC) {
    return std::nullopt;
  }
  if (plrs::read_u16_little_endian(bytes.subspan(4, 2)) != VERSION) {
    return std::nullopt;
  }
  const uint32_t stored_crc = static_cast<uint32_t>(bytes[12]) |
                              (static_cast<uint32_t>(bytes[13]) << 8) |
                              (static_cast<uint32_t>(bytes[14]) << 16) |
                              (static_cast<uint32_t>(bytes[15]) << 24);
  if (crc32(bytes.subspan(0, CRC_OFFSET)) != stored_crc) {
    return std::nullopt;
  }
  const float offset = plrs::read_f32_little_endian(bytes.subspan(8, 4));
  if (!std::isfinite(offset) || std::fabs(offset) > OFFSET_LIMIT_DEG) {
    return std::nullopt;
  }
  return offset;
}

/**
 * @brief Decides when the learned offset is worth writing to flash.
 *
 * A write is due only when all three hold: the offset is GNSS-validated (its
 * variance has dropped below VALIDATED_MAX_DEG2 -- the mag alone can never
 * shrink it that far), it has moved past CHANGE_DEG since the last write, and
 * MIN_INTERVAL has elapsed. That keeps flash writes minutes-apart and
 * change-gated, far under the sector's erase budget, and off the hot path.
 * Time is injected; no clock is read here (host-testable).
 */
class OffsetPersistPolicy {
public:
  static constexpr float VALIDATED_MAX_DEG2 =
      4.0f; // sigma <= 2 deg: GNSS-fixed
  static constexpr float CHANGE_DEG = 1.0f;
  static constexpr fusion::Ms MIN_INTERVAL {60'000}; // 1 min

  /**
   * @brief Whether to persist @p offset_deg now.
   *
   * @param offset_deg  Current EKF mag offset.
   * @param variance_deg2  Its variance (debug().mag_offset_variance_deg2).
   * @param now  Current time.
   */
  bool due(float offset_deg, float variance_deg2, fusion::Ms now) const {
    if (variance_deg2 > VALIDATED_MAX_DEG2) {
      return false;
    }
    if (_written && (now - _last_write) < MIN_INTERVAL) {
      return false;
    }
    if (_written && std::fabs(offset_deg - _last_offset_deg) < CHANGE_DEG) {
      return false;
    }
    return true;
  }

  /**
   * @brief Record that @p offset_deg was persisted at @p now.
   */
  void mark_written(float offset_deg, fusion::Ms now) {
    _last_offset_deg = offset_deg;
    _last_write = now;
    _written = true;
  }

private:
  float _last_offset_deg = 0.0f;
  fusion::Ms _last_write {0};
  bool _written = false;
};

} // namespace offset_store
