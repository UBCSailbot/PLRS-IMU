/**
 * FreeRTOS task for communicating with the IMU.
 */

#pragma once

#ifdef ARDUINO
#include "bno08x_transport.h"
#include "telemetry.h"
#include <FreeRTOS.h>
#include <queue.h>

namespace imu_task {

struct TaskParams {
  bno08x::I2cTransport transport;
  QueueHandle_t queue;
  plrs::TelemetrySink &telemetry;
};

void task(void *params);

} // namespace imu_task
#endif
