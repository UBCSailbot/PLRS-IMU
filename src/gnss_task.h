/**
 * FreeRTOS task for communicating with the GNSS kit.
 */

#pragma once

#ifdef ARDUINO
#include "gnss_bridge.h"
#include "septentrio_transport.h"
#include "telemetry.h"
#include <FreeRTOS.h>
#include <queue.h>

namespace gnss_task {

struct TaskParams {
  septentrio_gnss::Uart uart;
  QueueHandle_t queue;
  fusion::GnssAttitudeMount mount;
  plrs::TelemetrySink &telemetry;
};

void task(void *params);

} // namespace gnss_task
#endif
