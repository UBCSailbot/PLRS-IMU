/**
 * For pin assignment and hardware configuration of the RP2040 / RP2350 boards.
 */

#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * FreeRTOS configuration.
 */

// The GNSS parser embeds full SBF/NMEA/reply frame buffers (~1.1 KB)
static constexpr uint32_t GNSS_TASK_STACK_SIZE = 2048;
static constexpr uint32_t IMU_TASK_STACK_SIZE = 512;
static constexpr uint32_t FUSION_TASK_STACK_SIZE = 1024;
static constexpr uint32_t RUDDER_TASK_STACK_SIZE = 512;

// Persists the learned mag offset to flash; only the tiny policy + blob live on
// its stack (the EEPROM page buffer is heap), so a modest stack suffices.
static constexpr uint32_t PERSIST_TASK_STACK_SIZE = 1024;

static constexpr uint32_t GNSS_TASK_PRIORITY = 3;
static constexpr uint32_t IMU_TASK_PRIORITY = 3;
static constexpr uint32_t FUSION_TASK_PRIORITY = 2;
static constexpr uint32_t RUDDER_TASK_PRIORITY = 2;
// Lowest: a rare, latency-insensitive flash write must never preempt fusion.
static constexpr uint32_t PERSIST_TASK_PRIORITY = 1;

// Rudder link heading send rate.
static constexpr uint32_t RUDDER_SEND_INTERVAL_MS = 100;

/**
 * Hardware configuration.
 */

// I2C for the BNO085 IMU on `Wire`, which the core binds to a different
// peripheral per board: i2c0 on the Pico (boat), i2c1 on the Feather (bench).
// Each peripheral accepts only its own pin set, so we keep a pair for each and
// main.cpp picks by __WIRE0_DEVICE. i2c0 uses the former MTi UART pads GP16/17;
// i2c1 uses the Feather's free A0/A1 (its labelled SCL GP3 is the rudder TX).
// No INT/RST line: the driver polls and software-resets over SHTP.
static constexpr uint32_t BNO_I2C0_SDA_PIN = 16;
static constexpr uint32_t BNO_I2C0_SCL_PIN = 17;
static constexpr uint32_t BNO_I2C1_SDA_PIN = 26; // Feather A0
static constexpr uint32_t BNO_I2C1_SCL_PIN = 27; // Feather A1
static constexpr uint32_t BNO_I2C_BAUD = 400000;
static constexpr uint8_t BNO_I2C_ADDR = 0x4a; // 0x4b if the ADR jumper bridged

// UART 0 for telemetry out to the debug probe's UART bridge (bench readout on a
// single cable). Mirrors the USB CDC; see telemetry.h.
static constexpr uint32_t TELEMETRY_UART_BAUD = 115200;
static constexpr uint32_t TELEMETRY_UART_TX_PIN = 0;
static constexpr uint32_t TELEMETRY_UART_RX_PIN = 1;

// PIO UART for GNSS. GP6 = Pico TX -> module RXD1,
// GP7 = Pico RX <- module TXD1.
static constexpr uint32_t GNSS_UART_BAUD = 115200;
static constexpr uint32_t GNSS_UART_TX_PIN = 6;
static constexpr uint32_t GNSS_UART_RX_PIN = 7;

// PIO UART for communicating with the rudder module.
static constexpr uint32_t OUTPUT_UART_BAUD = 115200;
static constexpr uint32_t OUTPUT_UART_TX_PIN = 3;
static constexpr uint32_t OUTPUT_UART_RX_PIN = 4;

// Heartbeat
static constexpr uint32_t HEARTBEAT_LED_PIN = LED_BUILTIN;
