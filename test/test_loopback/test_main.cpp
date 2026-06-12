#include "mti_transport.h"
#include "xbus_protocol.h"

#include <Arduino.h>
#include <pico/time.h>
#include <unity.h>

static constexpr uint32_t LOOPBACK_BAUD = 115200;
static constexpr uint32_t LOOPBACK_TIMEOUT_MS = 500;

void setUp() {}
void tearDown() {}

static std::chrono::milliseconds now() {
  return std::chrono::milliseconds(time_us_64() / 1000);
}

/**
 * @brief Encode a GoToConfig packet, send it over Serial1, and assert the
 *        parser recovers the same MID from the loopback bytes.
 *
 * Requires a jumper between GP0 (TX) and GP1 (RX).
 */
void test_xbus_loopback_goto_config() {
  auto packet = xbus::Packet::command(xbus::MID::GoToConfig, {});
  TEST_ASSERT_TRUE(packet.has_value());

  auto encoded = xbus::encode(*packet);
  TEST_ASSERT_TRUE(encoded.has_value());

  mti::Uart uart(Serial1);
  uart.write(encoded->view());

  xbus::Parser parser;
  auto deadline = now() + std::chrono::milliseconds(LOOPBACK_TIMEOUT_MS);

  while (now() < deadline) {
    auto byte = uart.read();
    if (!byte) {
      delay(1);
      continue;
    }
    auto result = parser.feed(*byte, now());
    if (result) {
      TEST_ASSERT_EQUAL(xbus::MID::GoToConfig, result->mid);
      return;
    }
  }

  TEST_FAIL_MESSAGE("Timeout: no packet received on loopback");
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(LOOPBACK_BAUD);

  UNITY_BEGIN();
  RUN_TEST(test_xbus_loopback_goto_config);
  UNITY_END();
}

void loop() {}
