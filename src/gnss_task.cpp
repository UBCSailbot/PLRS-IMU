#ifdef ARDUINO
#include "gnss_task.h"
#include "command.h"
#include "gnss_bridge.h"
#include "hardware_config.h"
#include "sbf_blocks.h"
#include "stack_check.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <pico/time.h>
#include <task.h>

namespace gnss_task {

static_assert(
    plrs::fits_on_task_stack<septentrio_gnss::Parser>(GNSS_TASK_STACK_SIZE),
    "GNSS_TASK_STACK_SIZE too small for septentrio_gnss::Parser");

static constexpr uint32_t REPLY_TIMEOUT_MS = 2000;
static constexpr uint32_t RETRY_DELAY_MS = 1000;

static std::chrono::milliseconds now() {
  return std::chrono::milliseconds(time_us_64() / 1000);
}

/**
 * @brief Drain the UART into the parser until a Reply arrives or timeout.
 *
 * @param uart        Transport to read from.
 * @param parser      Wire parser instance.
 * @param timeout_ms  Maximum wait in milliseconds.
 *
 * @return The reply, or nullopt on timeout.
 */
static std::optional<septentrio_gnss::Reply>
wait_for_reply(septentrio_gnss::Uart &uart,
               septentrio_gnss::Parser &parser,
               uint32_t timeout_ms) {
  auto deadline = now() + std::chrono::milliseconds(timeout_ms);
  while (now() < deadline) {
    auto byte = uart.read();
    if (!byte) {
      vTaskDelay(1);
      continue;
    }
    auto msg = parser.feed(*byte, now());
    if (!msg)
      continue;
    if (auto *reply = std::get_if<septentrio_gnss::Reply>(&*msg)) {
      return *reply;
    }
  }
  return std::nullopt;
}

/**
 * @brief Send one command and wait for a non-error acknowledgement.
 *
 * @param uart    Transport to the mosaic-go-H.
 * @param parser  Wire parser instance.
 * @param cmd     Built command, or the build error carried through.
 * @param label   Command name for the failure log.
 *
 * @return true if the receiver acknowledged; false on build failure, timeout,
 *   or an error reply. The caller retries the whole sequence on false.
 */
static bool send_verified(
    septentrio_gnss::Uart &uart,
    septentrio_gnss::Parser &parser,
    const std::expected<septentrio_gnss::Command, const char *> &cmd,
    const char *label) {
  const auto fail = [&](const char *why) {
    if (Serial) {
      Serial.print("# GNSS: ");
      Serial.print(label);
      Serial.print(' ');
      Serial.println(why);
    }
    return false;
  };

  if (!cmd) {
    return fail("build failed");
  }
  uart.write(cmd->view());

  auto reply = wait_for_reply(uart, parser, REPLY_TIMEOUT_MS);
  if (!reply) {
    return fail("timeout");
  }
  if (reply->kind == septentrio_gnss::ReplyKind::Err) {
    return fail("rejected");
  }
  return true;
}

/**
 * @brief Configure the receiver for dual-antenna heading and verify each step.
 *        Retries the whole sequence indefinitely on any failure.
 *
 * Enables multi-antenna attitude, then turns on the attitude blocks. The
 * attitude source is asserted here every boot rather than relying on the
 * receiver's saved config, so a factory-reset or reflashed unit still produces
 * headings. Without it the receiver emits AttEuler with mode NO_ATTITUDE and
 * the filter never gets a heading fix.
 *
 * @param uart    Transport to the mosaic-go-H.
 * @param parser  Wire parser instance.
 */
static void bring_up(septentrio_gnss::Uart &uart,
                     septentrio_gnss::Parser &parser) {
  constexpr std::array<septentrio_gnss::SbfBlock, 2> blocks {
      septentrio_gnss::SbfBlock::AttEuler,
      septentrio_gnss::SbfBlock::AttCovEuler,
  };

  while (true) {
    const bool ready =
        send_verified(uart,
                      parser,
                      septentrio_gnss::set_gnss_attitude(
                          septentrio_gnss::GnssAttitudeMode::MultiAntenna),
                      "setGNSSAttitude") &&
        send_verified(
            uart,
            parser,
            septentrio_gnss::set_sbf_output(septentrio_gnss::SbfStream::Stream1,
                                            septentrio_gnss::Connection::COM1,
                                            blocks,
                                            septentrio_gnss::SbfInterval::Msec100),
            "setSBFOutput");

    if (ready) {
      if (Serial)
        Serial.println("# GNSS: ready");
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
  }
}

void task(void *params) {
  auto &p = *static_cast<TaskParams *>(params);
  septentrio_gnss::Parser parser;

  bring_up(p.uart, parser);

  std::optional<sbf::AttEuler> pending_att;

  while (true) {
    auto byte = p.uart.read();
    if (!byte) {
      vTaskDelay(1);
      continue;
    }

    auto msg = parser.feed(*byte, now());
    if (!msg)
      continue;

    auto *packet = std::get_if<sbf::Packet>(&*msg);
    if (!packet)
      continue;

    if (auto att = sbf::parse_att_euler(*packet)) {
      pending_att = att;
      continue;
    }

    if (auto cov = sbf::parse_att_cov_euler(*packet)) {
      if (!pending_att || pending_att->tow != cov->tow) {
        pending_att = std::nullopt;
        continue;
      }

      auto sample =
          fusion::att_euler_to_gnss_sample(*pending_att, *cov, p.mount);
      pending_att = std::nullopt;

      xQueueSend(p.queue, &sample, 0);
    }
  }
}

} // namespace gnss_task
#endif
