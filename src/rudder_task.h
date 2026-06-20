/**
 * FreeRTOS task that streams the fused heading over the rudder link.
 */

#pragma once

#ifdef ARDUINO
#include "rudder_transport.h"
#include <FreeRTOS.h>
#include <queue.h>

namespace rudder_task {

struct TaskParams {
  rudder::Uart uart;
  // Length-1 latest-value mailbox of fusion::FusionOutput (xQueueOverwrite).
  QueueHandle_t heading_mailbox;
};

void task(void *params);

} // namespace rudder_task
#endif
