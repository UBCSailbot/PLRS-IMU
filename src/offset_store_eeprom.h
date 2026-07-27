/**
 * Firmware flash backend for the mag-offset persistence.
 *
 * Reads/writes the offset_store blob to the earlephilhower EEPROM emulation
 * (one flash sector). The Arduino-free policy and blob format live in
 * lib/fusion/offset_store.h; this is only the I/O. EEPROM.begin() is called
 * once in main.cpp setup(), not here (transport-wrapper rule).
 */

#pragma once

#ifdef ARDUINO
#include "offset_store.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <array>
#include <cstddef>
#include <optional>

namespace offset_store_eeprom {

// EEPROM emulation footprint: the blob rounded up to a tidy span. begin() must
// be called with this size before load()/save().
constexpr std::size_t EEPROM_SIZE = 64;

/**
 * @brief Read and validate the stored offset, or nullopt if none/invalid.
 */
inline std::optional<float> load() {
  std::array<uint8_t, offset_store::BLOB_BYTES> buf {};
  for (std::size_t i = 0; i < buf.size(); i++) {
    buf[i] = EEPROM.read(static_cast<int>(i));
  }
  return offset_store::validate(plrs::ByteSpan(buf.data(), buf.size()));
}

/**
 * @brief Serialize @p offset_deg and commit it to flash.
 *
 * commit() idles the other core and erases+programs the sector -- a few ms,
 * so callers must keep this off the 100 Hz path and rare (see the persist
 * task and OffsetPersistPolicy).
 */
inline void save(float offset_deg) {
  const auto blob = offset_store::encode(offset_deg);
  for (std::size_t i = 0; i < blob.size(); i++) {
    EEPROM.write(static_cast<int>(i), blob[i]);
  }
  EEPROM.commit();
}

} // namespace offset_store_eeprom

#endif
