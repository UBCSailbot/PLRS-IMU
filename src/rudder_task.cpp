#ifdef ARDUINO
#include "rudder_task.h"
#include "fusion.h"
#include "hardware_config.h"
#include "rudder_protocol.h"

#include <FreeRTOS.h>
#include <task.h>

namespace rudder_task {

void task(void *params) {
  auto &p = *static_cast<TaskParams *>(params);
  rudder::Sender sender;

  TickType_t next = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&next, pdMS_TO_TICKS(RUDDER_SEND_INTERVAL_MS));

    fusion::FusionOutput out;
    if (xQueuePeek(p.heading_mailbox, &out, 0) == pdTRUE) {
      p.uart.write(
          sender.next(rudder::Heading {.deg = out.heading_deg}).view());
    }
  }
}

} // namespace rudder_task
#endif
