/**
 * For pin assignment and hardware configuration of the RP2040.
 */

#pragma once

#include <cstdint>

/**
 * FreeRTOS configuration.
 */

// The GNSS parser embeds full SBF/NMEA/reply frame buffers (~1.1 KB) and lives
// on its task stack, so this is sized well above the others.
static constexpr uint32_t GNSS_TASK_STACK_SIZE = 2048;
static constexpr uint32_t IMU_TASK_STACK_SIZE = 512;
static constexpr uint32_t FUSION_TASK_STACK_SIZE = 1024;

static constexpr uint32_t GNSS_TASK_PRIORITY = 3;
static constexpr uint32_t IMU_TASK_PRIORITY = 3;
static constexpr uint32_t FUSION_TASK_PRIORITY = 2;

/**
 * Hardware configuration.
 */

// UART 0
static constexpr uint32_t GNSS_UART_BAUD = 115200;
static constexpr uint32_t GNSS_UART_TX_PIN = 0;
static constexpr uint32_t GNSS_UART_RX_PIN = 1;

// UART 1
static constexpr uint32_t IMU_UART_BAUD = 115200;
static constexpr uint32_t IMU_UART_TX_PIN = 4;
static constexpr uint32_t IMU_UART_RX_PIN = 5;

// Emulated UART with PIO
static constexpr uint32_t OUTPUT_UART_BAUD = 115200;
static constexpr uint32_t OUTPUT_UART_TX_PIN = 20;
static constexpr uint32_t OUTPUT_UART_RX_PIN = 19;

// Heartbeat
static constexpr uint32_t HEARTBEAT_LED_PIN = 25;
