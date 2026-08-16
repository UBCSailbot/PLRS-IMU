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

// PVT diagnostic: throttle to ~1 Hz and isolate the fix-type nibble.
static constexpr uint32_t PVT_REPORT_INTERVAL_MS = 1000;
static constexpr uint8_t PVT_MODE_TYPE_MASK = 0x0F;

/**
 * @brief Print a throttled PVT status diagnostic: fix type, satellites, error.
 *
 * The telemetry path only carries attitude, so a stuck NO_ATTITUDE is opaque:
 * this surfaces whether the main antenna has a position fix (PVT mode nibble)
 * and how many satellites it uses, separating "no signal at all" from
 * "position fine, aux antenna missing".
 */
static void report_pvt(plrs::TelemetrySink &sink, const sbf::PVTGeodetic &pvt) {
  static std::chrono::milliseconds last {0};
  const auto t = now();
  if (t - last < std::chrono::milliseconds(PVT_REPORT_INTERVAL_MS)) {
    return;
  }
  last = t;
  sink.line().printf("# PVT: fix=%u sats=%u error=%u\n",
                     pvt.mode & PVT_MODE_TYPE_MASK,
                     pvt.nr_sv,
                     pvt.error);
}

/**
 * @brief Print a throttled aux-antenna tracking diagnostic: how many satellites
 *        the aux1 antenna is tracking for the attitude baseline, and its error.
 *
 * Isolates the aux side of a stuck heading: sats=0 means the aux antenna is
 * receiving nothing (RF path or receiver aux input), even when its port is
 * powered; n=0 means the aux antenna is not in the solution at all.
 */
static void report_aux(plrs::TelemetrySink &sink,
                       const sbf::AuxAntTracking &aux) {
  static std::chrono::milliseconds last {0};
  const auto t = now();
  if (t - last < std::chrono::milliseconds(PVT_REPORT_INTERVAL_MS)) {
    return;
  }
  last = t;
  sink.line().printf("# AUX: n=%u id=%u sats=%u error=%u\n",
                     aux.n,
                     aux.aux_ant_id,
                     aux.nr_sv,
                     aux.error);
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
// Diagnostic counters for what arrived on the GNSS link during a reply wait:
// distinguishes a silent link (baud/TX/receiver) from a receiver streaming SBF
// whose ASCII reply is getting buried.
struct LinkStats {
  uint32_t bytes = 0;
  uint32_t sbf = 0;
};

static std::optional<septentrio_gnss::Reply>
wait_for_reply(septentrio_gnss::Uart &uart,
               septentrio_gnss::Parser &parser,
               uint32_t timeout_ms,
               LinkStats &stats) {
  auto deadline = now() + std::chrono::milliseconds(timeout_ms);
  while (now() < deadline) {
    auto byte = uart.read();
    if (!byte) {
      vTaskDelay(1);
      continue;
    }
    stats.bytes++;
    auto msg = parser.feed(*byte, now());
    if (!msg)
      continue;
    if (std::get_if<sbf::Packet>(&*msg)) {
      stats.sbf++;
    }
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
static bool
send_verified(septentrio_gnss::Uart &uart,
              septentrio_gnss::Parser &parser,
              plrs::TelemetrySink &sink,
              const std::expected<septentrio_gnss::Command, const char *> &cmd,
              const char *label) {
  const auto fail = [&](const char *why) {
    auto line = sink.line();
    line.print("# GNSS: ");
    line.print(label);
    line.print(' ');
    line.println(why);
    return false;
  };

  if (!cmd) {
    return fail("build failed");
  }
  uart.write(cmd->view());

  LinkStats stats;
  auto reply = wait_for_reply(uart, parser, REPLY_TIMEOUT_MS, stats);
  if (!reply) {
    sink.line().printf("# GNSS: %s timeout (rx=%lu sbf=%lu)\n",
                       label,
                       static_cast<unsigned long>(stats.bytes),
                       static_cast<unsigned long>(stats.sbf));
    return false;
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
 * The resolution is Fixed because gnss_bridge accepts only fixed-ambiguity
 * modes (2 and 4). Under Float the receiver reports mode 1, so every sample
 * arrives valid=false and the filter never sees a heading at all.
 *
 * @param uart    Transport to the mosaic-go-H.
 * @param parser  Wire parser instance.
 */
static void bring_up(septentrio_gnss::Uart &uart,
                     septentrio_gnss::Parser &parser,
                     plrs::TelemetrySink &sink) {
  constexpr std::array<septentrio_gnss::SbfBlock, 4> blocks {
      septentrio_gnss::SbfBlock::AttEuler,
      septentrio_gnss::SbfBlock::AttCovEuler,
      septentrio_gnss::SbfBlock::PVTGeodetic,
      septentrio_gnss::SbfBlock::AuxAntPositions,
  };

  while (true) {
    const bool ready =
        send_verified(uart,
                      parser,
                      sink,
                      septentrio_gnss::set_gnss_attitude(
                          septentrio_gnss::GnssAttitudeMode::MultiAntenna,
                          septentrio_gnss::AttitudeResolution::Fixed),
                      "setGNSSAttitude") &&
        send_verified(uart,
                      parser,
                      sink,
                      septentrio_gnss::set_sbf_output(
                          septentrio_gnss::SbfStream::Stream1,
                          septentrio_gnss::Connection::COM1,
                          blocks,
                          septentrio_gnss::SbfInterval::Msec100),
                      "setSBFOutput");

    if (ready) {
      sink.line().println("# GNSS: ready");
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
  }
}

void task(void *params) {
  auto &p = *static_cast<TaskParams *>(params);
  septentrio_gnss::Parser parser;

  bring_up(p.uart, parser, p.telemetry);

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

    if (auto aux = sbf::parse_aux_ant_positions(*packet)) {
      report_aux(p.telemetry, *aux);
      continue;
    }

    if (auto pvt = sbf::parse_pvt_geodetic(*packet)) {
      report_pvt(p.telemetry, *pvt);
      continue;
    }

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
