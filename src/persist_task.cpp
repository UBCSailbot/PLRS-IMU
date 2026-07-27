#ifdef ARDUINO
#include "persist_task.h"
#include "offset_store.h"
#include "offset_store_eeprom.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

namespace persist_task {

void task(void *params) {
  auto &p = *static_cast<TaskParams *>(params);
  offset_store::OffsetPersistPolicy policy;

  while (true) {
    OffsetSample sample;
    // Block until the fusion task publishes; the mailbox is a 1-slot overwrite,
    // so we always see the latest offset and never a backlog.
    if (xQueueReceive(p.offset_mailbox, &sample, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    if (!policy.due(
            sample.offset_deg, sample.variance_deg2, sample.timestamp)) {
      continue;
    }
    offset_store_eeprom::save(sample.offset_deg);
    policy.mark_written(sample.offset_deg, sample.timestamp);
    p.telemetry.print("# persist: saved ");
    p.telemetry.println(sample.offset_deg, 3);
  }
}

} // namespace persist_task
#endif
