#include "mti_transport.h"
#include "xbus_protocol.h"

#include <Arduino.h>
#include <pico/time.h>
#include <unity.h>

#include <array>
#include <bit>
#include <cstdint>
#include <optional>

static constexpr uint32_t LOOPBACK_BAUD = 115200;
static constexpr uint32_t LOOPBACK_TIMEOUT_MS = 500;

void setUp() {}
void tearDown() {}

static std::chrono::milliseconds now() {
  return std::chrono::milliseconds(time_us_64() / 1000);
}

/**
 * @brief Write a frame out the loopback UART and read the echoed bytes back
 *        through a parser.
 *
 * Requires a jumper between GP0 (TX) and GP1 (RX).
 *
 * @return The first packet the parser recovers, or nullopt on timeout.
 */
static std::optional<xbus::Packet> loopback_roundtrip(mti::Uart &uart,
                                                      xbus::ByteSpan frame) {
  xbus::Parser parser;
  uart.write(frame);
  auto deadline = now() + std::chrono::milliseconds(LOOPBACK_TIMEOUT_MS);
  while (now() < deadline) {
    auto byte = uart.read();
    if (!byte) {
      delay(1);
      continue;
    }
    if (auto packet = parser.feed(*byte, now())) {
      return packet;
    }
  }
  return std::nullopt;
}

/**
 * @brief A GoToConfig command survives the UART round-trip and parses back to
 *        the same MID.
 */
void test_xbus_loopback_goto_config() {
  auto packet = xbus::Packet::command(xbus::MID::GoToConfig, {});
  TEST_ASSERT_TRUE(packet.has_value());
  auto encoded = xbus::encode(*packet);
  TEST_ASSERT_TRUE(encoded.has_value());

  mti::Uart uart(Serial1);
  auto result = loopback_roundtrip(uart, encoded->view());
  TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "Timeout: no packet on loopback");
  TEST_ASSERT_EQUAL(xbus::MID::GoToConfig, result->mid);
}

/**
 * @brief A full MTData2 frame carrying a quaternion survives the UART
 *        round-trip and the quaternion is recovered intact. Exercises a larger,
 *        multi-byte frame than the bare command above.
 */
void test_xbus_loopback_mtdata2_quaternion() {
  const plrs::Quaternion sent {.w = 0.7071f, .x = 0.0f, .y = 0.7071f, .z = 0.0f};

  std::array<uint8_t, xbus::SUBPACKET_HEADER + xbus::QUATERNION_BYTES> sub {};
  auto id =
      xbus::write_u16_big_endian(static_cast<uint16_t>(xbus::DataId::Quaternion));
  sub[0] = id[0];
  sub[1] = id[1];
  sub[2] = xbus::QUATERNION_BYTES;

  const float comps[] = {sent.w, sent.x, sent.y, sent.z};
  for (std::size_t i = 0; i < 4; i++) {
    auto bits = std::bit_cast<uint32_t>(comps[i]);
    std::size_t off = xbus::SUBPACKET_HEADER + i * sizeof(float);
    sub[off + 0] = static_cast<uint8_t>(bits >> 24);
    sub[off + 1] = static_cast<uint8_t>(bits >> 16);
    sub[off + 2] = static_cast<uint8_t>(bits >> 8);
    sub[off + 3] = static_cast<uint8_t>(bits);
  }

  auto packet =
      xbus::Packet::command(xbus::MID::MTData2, {sub.data(), sub.size()});
  TEST_ASSERT_TRUE(packet.has_value());
  auto encoded = xbus::encode(*packet);
  TEST_ASSERT_TRUE(encoded.has_value());

  mti::Uart uart(Serial1);
  auto result = loopback_roundtrip(uart, encoded->view());
  TEST_ASSERT_TRUE_MESSAGE(result.has_value(), "Timeout: no packet on loopback");
  TEST_ASSERT_EQUAL(xbus::MID::MTData2, result->mid);

  auto got = xbus::read_quaternion(*result);
  TEST_ASSERT_TRUE(got.has_value());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, sent.w, got->w);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, sent.x, got->x);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, sent.y, got->y);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, sent.z, got->z);
}

void setup() {
  Serial.begin(115200);
  // Block until the host opens the CDC port. A fixed delay would race the
  // post-upload USB re-enumeration and lose the Unity output (PIO sees nothing,
  // reports [Errno 5]); waiting on DTR syncs the firmware to the test runner.
  while (!Serial) {
  }

  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(LOOPBACK_BAUD);

  UNITY_BEGIN();
  RUN_TEST(test_xbus_loopback_goto_config);
  RUN_TEST(test_xbus_loopback_mtdata2_quaternion);
  UNITY_END();
  Serial.flush();
}

void loop() {
  // Native USB drops the CDC port once tud_task() stops being pumped (which
  // happens with an empty loop after setup ends), cutting the read short before
  // the runner drains the results. Evaluating Serial pumps tud_task; keep it
  // alive so the full Unity output reaches the host.
  (void)static_cast<bool>(Serial);
  delay(10);
}
