#ifdef ARDUINO
#include "rudder_task.h"
#include "fusion.h"
#include "hardware_config.h"
#include "rudder_protocol.h"

#include <FreeRTOS.h>
#include <task.h>

namespace rudder_task {

// Heading is reported valid while its 1-sigma uncertainty stays within this
// bound. GNSS-anchored heading sits near the receiver's ~0.3 deg; once GNSS
// drops out the fused variance grows without bound, so any modest threshold
// cleanly separates anchored from free-drifting. Tune to taste.
static constexpr float HEADING_VALID_MAX_VARIANCE_DEG2 = 25.0f; // 5 deg sigma

void task(void *params) {
  auto &p = *static_cast<TaskParams *>(params);
  rudder::Sender sender;

  TickType_t next = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&next, pdMS_TO_TICKS(RUDDER_SEND_INTERVAL_MS));

    fusion::FusionOutput out;
    if (xQueuePeek(p.heading_mailbox, &out, 0) == pdTRUE) {
      const bool heading_valid =
          out.heading_variance_deg2 <= HEADING_VALID_MAX_VARIANCE_DEG2;
      p.uart.write(sender
                       .next(rudder::Attitude {.heading_deg = out.heading_deg,
                                               .roll_deg = out.roll_deg,
                                               .pitch_deg = out.pitch_deg,
                                               .yaw_rate_dps = out.yaw_rate_dps,
                                               .heading_valid = heading_valid})
                       .view());
      // Same tick, the pre-filter MTi values, so the rudder can switch to raw
      // heel and yaw rate while the EKF is being worked on.
      p.uart.write(
          sender
              .next(rudder::RawAttitude {.heel_deg = out.raw_roll_deg,
                                         .yaw_rate_dps = out.raw_yaw_rate_dps})
              .view());
    }
  }
}

} // namespace rudder_task
#endif
