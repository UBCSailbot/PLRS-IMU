#include "ekf_filter.h"
#include "fusion.h"
#include "fusion_task.h"
#include "gnss_task.h"
#include "hardware_config.h"
#include "imu_task.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <SerialPIO.h>
#include <task.h>

static SerialPIO output_serial(OUTPUT_UART_TX_PIN, OUTPUT_UART_RX_PIN);

void setup() {
  Serial1.setTX(GNSS_UART_TX_PIN);
  Serial1.setRX(GNSS_UART_RX_PIN);
  Serial1.begin(GNSS_UART_BAUD);

  Serial2.setTX(IMU_UART_TX_PIN);
  Serial2.setRX(IMU_UART_RX_PIN);
  Serial2.begin(IMU_UART_BAUD);

  output_serial.begin(OUTPUT_UART_BAUD);

  QueueHandle_t imu_queue = xQueueCreate(8, sizeof(fusion::ImuSample));
  QueueHandle_t gnss_queue = xQueueCreate(4, sizeof(fusion::GnssSample));

  static const fusion::TinyEkfFilter::Config filter_config {
      .q_heading_deg2 = 0.01f,
      .q_roll_deg2 = 0.01f,
      .q_pitch_deg2 = 0.01f,
      .q_bias_deg2_s2 = 0.0001f,
      .p0_heading_deg2 = 1000.0f,
      .p0_roll_deg2 = 1000.0f,
      .p0_pitch_deg2 = 1000.0f,
      .p0_bias_deg2_s2 = 1.0f,
      .mti_roll_variance_deg2 = 1.0f,
      .mti_pitch_variance_deg2 = 1.0f,
  };

  static imu_task::TaskParams imu_params {mti::Uart(Serial2), imu_queue};
  static gnss_task::TaskParams gnss_params {septentrio_gnss::Uart(Serial1),
                                            gnss_queue};
  static fusion_task::TaskParams fusion_params {
      imu_queue, gnss_queue, filter_config};

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
}

void loop() {
  // In FreeRTOS, the loop function is not strictly needed for task management.
  // The scheduler handles the tasks. You can leave it empty.
}
