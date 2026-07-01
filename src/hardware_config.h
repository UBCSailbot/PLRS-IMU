/**
 * For pin assignment and hardware configuration of the RP2040 / RP2350 boards.
 */

#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * FreeRTOS configuration.
 */

// The GNSS parser embeds full SBF/NMEA/reply frame buffers (~1.1 KB) and lives
// on its task stack, so this is sized well above the others.
static constexpr uint32_t GNSS_TASK_STACK_SIZE = 2048;
static constexpr uint32_t IMU_TASK_STACK_SIZE = 512;
static constexpr uint32_t FUSION_TASK_STACK_SIZE = 1024;
static constexpr uint32_t RUDDER_TASK_STACK_SIZE = 512;

static constexpr uint32_t GNSS_TASK_PRIORITY = 3;
static constexpr uint32_t IMU_TASK_PRIORITY = 3;
static constexpr uint32_t FUSION_TASK_PRIORITY = 2;
static constexpr uint32_t RUDDER_TASK_PRIORITY = 2;

// Rudder link heading send rate.
static constexpr uint32_t RUDDER_SEND_INTERVAL_MS = 100;

/**
 * Hardware configuration.
 */

// UART 0
static constexpr uint32_t IMU_UART_BAUD = 115200;
static constexpr uint32_t IMU_UART_TX_PIN = 16;
static constexpr uint32_t IMU_UART_RX_PIN = 17;

// PIO UART on GP6/GP7; the hardware UART1 TX pin (GP8) is dead on the board.
static constexpr uint32_t GNSS_UART_BAUD = 115200;
static constexpr uint32_t GNSS_UART_TX_PIN = 7;
static constexpr uint32_t GNSS_UART_RX_PIN = 6;

// Emulated UART with PIO
static constexpr uint32_t OUTPUT_UART_BAUD = 115200;
static constexpr uint32_t OUTPUT_UART_TX_PIN = 4;
static constexpr uint32_t OUTPUT_UART_RX_PIN = 3;

// Heartbeat
static constexpr uint32_t HEARTBEAT_LED_PIN = LED_BUILTIN;
