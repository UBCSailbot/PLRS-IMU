/**
 * FreeRTOS task for communicating with the GNSS kit.
 */

#pragma once

#ifdef ARDUINO
#include "septentrio_transport.h"
#include <FreeRTOS.h>
#include <queue.h>

namespace gnss_task {

struct TaskParams {
  septentrio_gnss::Uart uart;
  QueueHandle_t queue;
};

void task(void *params);

} // namespace gnss_task
#endif
