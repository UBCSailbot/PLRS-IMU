/**
 * FreeRTOS task for communicating with the IMU.
 */

#pragma once

#ifdef ARDUINO
#include "bno08x_transport.h"
#include <FreeRTOS.h>
#include <queue.h>

namespace imu_task {

struct TaskParams {
  bno08x::I2cTransport transport;
  QueueHandle_t queue;
  Print &telemetry;
};

void task(void *params);

} // namespace imu_task
#endif
