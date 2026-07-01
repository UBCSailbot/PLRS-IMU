#include "_freertos.h"
#include "ekf_filter.h"
#include "fusion.h"
#include "fusion_task.h"
#include "gnss_task.h"
#include "hardware_config.h"
#include "imu_task.h"
#include "rudder_task.h"
#include "tuning.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <SerialPIO.h>
#include <task.h>

void heartbeat_task(void *params) {
  pinMode(HEARTBEAT_LED_PIN, OUTPUT);
  while (true) {
    for (uint32_t i = 0; i < 2; i++) {
      digitalWrite(HEARTBEAT_LED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(200));
      digitalWrite(HEARTBEAT_LED_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200); // required to bring up USB CDC (ttyACM0)

  static SerialPIO output_serial(OUTPUT_UART_TX_PIN, OUTPUT_UART_RX_PIN);

  Serial1.setTX(IMU_UART_TX_PIN);
  Serial1.setRX(IMU_UART_RX_PIN);
  Serial1.begin(IMU_UART_BAUD);

  Serial2.setTX(GNSS_UART_TX_PIN);
  Serial2.setRX(GNSS_UART_RX_PIN);
  Serial2.begin(GNSS_UART_BAUD);

  output_serial.begin(OUTPUT_UART_BAUD);

  QueueHandle_t imu_queue = xQueueCreate(8, sizeof(fusion::ImuSample));
  QueueHandle_t gnss_queue = xQueueCreate(4, sizeof(fusion::GnssSample));
  QueueHandle_t heading_mailbox = xQueueCreate(1, sizeof(fusion::FusionOutput));

  static imu_task::TaskParams imu_params {mti::Uart(Serial1), imu_queue};
  static gnss_task::TaskParams gnss_params {
      septentrio_gnss::Uart(Serial2), gnss_queue, tuning::kGnssMount};
  static fusion_task::TaskParams fusion_params {
      imu_queue, gnss_queue, heading_mailbox, tuning::kFilterConfig};
  static rudder_task::TaskParams rudder_params {rudder::Uart(output_serial),
                                                heading_mailbox};

  xTaskCreate(imu_task::task,
              "imu",
              IMU_TASK_STACK_SIZE,
              &imu_params,
              IMU_TASK_PRIORITY,
              nullptr);
  xTaskCreate(gnss_task::task,
              "gnss",
              GNSS_TASK_STACK_SIZE,
              &gnss_params,
              GNSS_TASK_PRIORITY,
              nullptr);
  xTaskCreate(fusion_task::task,
              "fusion",
              FUSION_TASK_STACK_SIZE,
              &fusion_params,
              FUSION_TASK_PRIORITY,
              nullptr);
  xTaskCreate(rudder_task::task,
              "rudder",
              RUDDER_TASK_STACK_SIZE,
              &rudder_params,
              RUDDER_TASK_PRIORITY,
              nullptr);
  // Temporary heartbeat task
  xTaskCreate(heartbeat_task, "heartbeat", 128, nullptr, 4, nullptr);
}

void loop() {
  // In FreeRTOS, the loop function is not strictly needed for task management.
  // The scheduler handles the tasks. You can leave it empty.
}
