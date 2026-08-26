#include "_freertos.h"
#include "ekf_filter.h"
#include "fusion.h"
#include "fusion_task.h"
#include "gnss_task.h"
#include "hardware_config.h"
#include "imu_task.h"
#include "offset_store_eeprom.h"
#include "persist_task.h"
#include "rudder_task.h"
#include "telemetry_task.h"
#include "telemetry.h"
#include "tuning.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <FreeRTOS.h>
#include <SerialPIO.h>
#include <Wire.h>
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

  Serial1.setTX(TELEMETRY_UART_TX_PIN);
  Serial1.setRX(TELEMETRY_UART_RX_PIN);
  Serial1.begin(TELEMETRY_UART_BAUD);
  static plrs::TelemetrySink telemetry(Serial1);

  static SerialPIO output_serial(OUTPUT_UART_TX_PIN, OUTPUT_UART_RX_PIN);

  // `Wire` is i2c0 on the Pico and i2c1 on the Feather; each accepts only its
  // own pins, so pick the pair that matches whichever it is here. Only variants
  // that remap Wire define __WIRE0_DEVICE; mirror Wire.cpp's i2c0 default so
  // the stock Pico (which leaves it undefined) still resolves.
#ifndef __WIRE0_DEVICE
#define __WIRE0_DEVICE i2c0
#endif
  const bool wire_is_i2c0 = (__WIRE0_DEVICE == i2c0);
  Wire.setSDA(wire_is_i2c0 ? BNO_I2C0_SDA_PIN : BNO_I2C1_SDA_PIN);
  Wire.setSCL(wire_is_i2c0 ? BNO_I2C0_SCL_PIN : BNO_I2C1_SCL_PIN);
  Wire.setClock(BNO_I2C_BAUD);
  Wire.begin();

  static SerialPIO gnss_serial(GNSS_UART_TX_PIN, GNSS_UART_RX_PIN);
  gnss_serial.begin(GNSS_UART_BAUD);

  output_serial.begin(OUTPUT_UART_BAUD);

  QueueHandle_t imu_queue = xQueueCreate(8, sizeof(fusion::ImuSample));
  QueueHandle_t gnss_queue = xQueueCreate(4, sizeof(fusion::GnssSample));
  QueueHandle_t heading_mailbox = xQueueCreate(1, sizeof(fusion::FusionOutput));
  QueueHandle_t offset_mailbox =
      xQueueCreate(1, sizeof(persist_task::OffsetSample));
  QueueHandle_t telemetry_mailbox =
      xQueueCreate(1, sizeof(telemetry_task::Snapshot));

  // Restore the mag offset a prior run learned from GNSS, so heading anchors to
  // it from boot instead of the static tuning seed. A missing/invalid blob
  // leaves the tuning.toml offset_seed_deg in place; GNSS refines either way.
  EEPROM.begin(offset_store_eeprom::EEPROM_SIZE);
  static auto filter_config = tuning::kFilterConfig;
  if (auto persisted = offset_store_eeprom::load();
      persisted && filter_config.mti_yaw) {
    filter_config.mti_yaw->offset_seed_deg = *persisted;
    auto line = telemetry.line();
    line.print("# persist: loaded ");
    line.println(*persisted, 3);
  } else {
    telemetry.line().println("# persist: no stored offset");
  }

  static imu_task::TaskParams imu_params {
      bno08x::I2cTransport(Wire, BNO_I2C_ADDR), imu_queue, telemetry};
  static gnss_task::TaskParams gnss_params {septentrio_gnss::Uart(gnss_serial),
                                            gnss_queue,
                                            tuning::kGnssMount,
                                            telemetry};
  static fusion_task::TaskParams fusion_params {imu_queue,
                                                gnss_queue,
                                                heading_mailbox,
                                                offset_mailbox,
                                                telemetry_mailbox,
                                                filter_config};
  static persist_task::TaskParams persist_params {offset_mailbox, telemetry};
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
  xTaskCreate(persist_task::task,
              "persist",
              PERSIST_TASK_STACK_SIZE,
              &persist_params,
              PERSIST_TASK_PRIORITY,
              nullptr);
  static telemetry_task::TaskParams telemetry_params {telemetry_mailbox,
                                                      telemetry};
  xTaskCreate(telemetry_task::task,
              "telemetry",
              TELEMETRY_TASK_STACK_SIZE,
              &telemetry_params,
              TELEMETRY_TASK_PRIORITY,
              nullptr);
  // Temporary heartbeat task
  xTaskCreate(heartbeat_task, "heartbeat", 128, nullptr, 4, nullptr);
}

void loop() {
  // In FreeRTOS, the loop function is not strictly needed for task management.
  // The scheduler handles the tasks. You can leave it empty.
}
