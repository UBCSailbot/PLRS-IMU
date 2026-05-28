/**
 * Test-only frame-building helpers. Not part of the driver.
 *
 * Independent of encode() so that a bug in encode() cannot mask a bug in the
 * parser (and vice versa).
 */

#pragma once
#include "xbus_protocol.h"
#include <cstdint>
#include <vector>

namespace xtest {

inline std::vector<uint8_t> make_frame(xbus::MID mid,
                                       const std::vector<uint8_t> &payload,
                                       uint8_t bid = xbus::BID_MASTER) {
  std::vector<uint8_t> f;
  f.push_back(xbus::PREAMBLE);
  f.push_back(bid);
  f.push_back(static_cast<uint8_t>(mid));
  f.push_back(static_cast<uint8_t>(payload.size()));
  for (uint8_t b : payload)
    f.push_back(b);
  uint8_t sum =
      bid + static_cast<uint8_t>(mid) + static_cast<uint8_t>(payload.size());
  for (uint8_t b : payload)
    sum += b;
  f.push_back(static_cast<uint8_t>(0x100 - sum));
  return f;
}

inline std::vector<uint8_t> make_subpacket(xbus::DataId id,
                                           const std::vector<uint8_t> &bytes) {
  std::vector<uint8_t> sp;
  uint16_t raw = static_cast<uint16_t>(id);
  sp.push_back(raw >> 8);
  sp.push_back(raw & 0xFF);
  sp.push_back(static_cast<uint8_t>(bytes.size()));
  for (uint8_t b : bytes)
    sp.push_back(b);
  return sp;
}

inline std::vector<uint8_t> be_float(float v) {
  uint32_t u = std::bit_cast<uint32_t>(v);
  return {uint8_t(u >> 24), uint8_t(u >> 16), uint8_t(u >> 8), uint8_t(u)};
}

} // namespace xtest
