/**
 * FreeRTOS task for communicating with the IMU.
 */

#pragma once

#ifdef ARDUINO
#include "mti_transport.h"
#include <FreeRTOS.h>
#include <queue.h>

namespace imu_task {

struct TaskParams {
  mti::Uart uart;
  QueueHandle_t queue;
};

void task(void *params);

} // namespace imu_task
#endif
