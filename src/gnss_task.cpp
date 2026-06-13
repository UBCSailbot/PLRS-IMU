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
 * @brief Configure SBF output and verify the receiver accepted it.
 *        Retries indefinitely on timeout or error reply.
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
    auto cmd =
        septentrio_gnss::set_sbf_output(septentrio_gnss::SbfStream::Stream1,
                                        septentrio_gnss::Connection::COM1,
                                        blocks,
                                        septentrio_gnss::SbfInterval::Msec100);

    if (!cmd) {
      Serial.println("GNSS: failed to build command, retrying");
      vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
      continue;
    }

    uart.write(cmd->view());

    auto reply = wait_for_reply(uart, parser, REPLY_TIMEOUT_MS);
    if (!reply) {
      Serial.println("GNSS: setSBFOutput timeout, retrying");
      vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
      continue;
    }
    if (reply->kind == septentrio_gnss::ReplyKind::Err) {
      Serial.println("GNSS: setSBFOutput rejected, retrying");
      vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
      continue;
    }

    Serial.println("GNSS: ready");
    return;
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

      // TODO: thread mount calibration through TaskParams once tuning codegen
      // lands (#47)
      constexpr fusion::GnssAttitudeMount mount {};
      auto sample = fusion::att_euler_to_gnss_sample(*pending_att, *cov, mount);
      pending_att = std::nullopt;

      xQueueSend(p.queue, &sample, 0);
    }
  }
}

} // namespace gnss_task
#endif
