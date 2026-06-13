#ifdef ARDUINO
#include "imu_task.h"
#include "fusion.h"
#include "hardware_config.h"
#include "stack_check.h"
#include "xbus_protocol.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <pico/time.h>
#include <task.h>

namespace imu_task {

static_assert(plrs::fits_on_task_stack<xbus::Parser>(IMU_TASK_STACK_SIZE),
              "IMU_TASK_STACK_SIZE too small for xbus::Parser");

static constexpr uint32_t ACK_TIMEOUT_MS = 500;
static constexpr uint32_t RETRY_DELAY_MS = 1000;
static constexpr uint16_t IMU_RATE_HZ = 100;

static constexpr auto OUTPUT_CONFIG =
    xbus::build_output_config(std::array<xbus::OutputItem, 3> {{
        {xbus::DataId::Quaternion, IMU_RATE_HZ},
        {xbus::DataId::RateOfTurn, IMU_RATE_HZ},
        {xbus::DataId::Acceleration, IMU_RATE_HZ},
    }});

static std::chrono::milliseconds now() {
  return std::chrono::milliseconds(time_us_64() / 1000);
}

/**
 * @brief Drain the UART into the parser until a packet with @p mid arrives.
 *
 * @param uart Transport to read from.
 * @param parser Xbus parser instance.
 * @param mid Expected message ID.
 * @param timeout_ms Maximum wait in millisecons.
 *
 * @return The matched packet, or nullopt on timeout.
 */
static std::optional<xbus::Packet> wait_for(mti::Uart &uart,
                                            xbus::Parser &parser,
                                            xbus::MID mid,
                                            uint32_t timeout_ms) {
  auto deadline = now() + std::chrono::milliseconds(timeout_ms);
  while (now() < deadline) {
    auto byte = uart.read();
    if (!byte) {
      vTaskDelay(1);
      continue;
    }
    auto packet = parser.feed(*byte, now());
    if (packet && packet->mid == mid) {
      return packet;
    }
  }
  return std::nullopt;
}

static void send(mti::Uart &uart, xbus::MID mid, xbus::ByteSpan payload) {
  auto packet = xbus::Packet::command(mid, payload);
  if (!packet)
    return;
  auto encoded = xbus::encode(*packet);
  if (encoded)
    uart.write(encoded->view());
}

/**
 * @brief Configure output and enter measurement mode. Retries indefinitely.
 *
 * @param uart Transport to the MTi-3.
 * @param parser Xbus parser instance.
 */
static void bring_up(mti::Uart &uart, xbus::Parser &parser) {
  while (true) {
    send(uart, xbus::MID::GoToConfig, {});
    if (!wait_for(uart, parser, xbus::MID::GoToConfigAck, ACK_TIMEOUT_MS)) {
      Serial.println("IMU: GoToConfig timeout, retrying");
      vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
      continue;
    }

    send(uart, xbus::MID::SetOutputConfig, OUTPUT_CONFIG);
    if (!wait_for(uart, parser, xbus::MID::OutputConfigAck, ACK_TIMEOUT_MS)) {
      Serial.println("IMU: SetOutputConfig timeout, retrying");
      vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
      continue;
    }

    send(uart, xbus::MID::GoToMeasurement, {});
    if (!wait_for(uart, parser, xbus::MID::GoToMeasAck, ACK_TIMEOUT_MS)) {
      Serial.println("IMU: GoToMeasurement timeout, retrying");
      vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
      continue;
    }

    Serial.println("IMU: ready");
    return;
  }
}

void task(void *params) {
  auto &p = *static_cast<TaskParams *>(params);
  xbus::Parser parser;

  bring_up(p.uart, parser);

  while (true) {
    auto byte = p.uart.read();
    if (!byte) {
      vTaskDelay(1);
      continue;
    }

    auto packet = parser.feed(*byte, now());
    if (!packet)
      continue;
    if (packet->mid != xbus::MID::MTData2)
      continue;

    auto quat = xbus::read_quaternion(*packet);
    auto gyro_data = xbus::find_data(*packet, xbus::DataId::RateOfTurn);
    auto accel_data = xbus::find_data(*packet, xbus::DataId::Acceleration);

    if (!quat || !gyro_data || !accel_data)
      continue;

    auto read_vec3 = [](xbus::ByteSpan b) -> plrs::Vec3 {
      constexpr std::size_t stride = sizeof(float);
      return {
          xbus::read_f32_big_endian(b.subspan(0 * stride, stride)),
          xbus::read_f32_big_endian(b.subspan(1 * stride, stride)),
          xbus::read_f32_big_endian(b.subspan(2 * stride, stride)),
      };
    };

    auto orientation = fusion::UnitQuaternion::from_raw(*quat);
    if (!orientation)
      continue;

    fusion::ImuSample sample {
        .angular_velocity_rad_s = read_vec3(gyro_data->bytes),
        .accel_ms2 = read_vec3(accel_data->bytes),
        .orientation = *orientation,
        .timestamp = now(),
    };

    xQueueSend(p.queue, &sample, 0);
  }
}

} // namespace imu_task
#endif
