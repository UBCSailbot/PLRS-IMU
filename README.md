# Polaris IMU

This repository contains the source code for Polaris' IMU project.

## Setup

This project's toolchain is based on [PlatformIO](https://platformio.org/),
configured for the rp2040.

### Build and Flash

To build the project, run:
```bash
pio run
```

To flash the compiled firmware to the RP2040, run:
```bash
pio run -t upload
```

### Flakes

If you use a nix based package management, a dev shell is provided in
`flake.nix`. If you do not already use [direnv](https://direnv.net/), this can
automatically load and unload your development environment.

## Overview

The purpose of this project is to combine the measurements from a 9-DoF MEMS IMU
with a GNSS kit to get heading accuracy on Polaris with <= 2deg accuracy.

### Kalman Filtering

The sensor fusion core is built on a variant of the Kalman Filter called the
Unscented Kalman Filter (Due to tight turnaround the actual solution might be
the Extended Kalman Filter) (Due to tight turnaround the actual solution might
be the Extended Kalman Filter).

#### Reference Projects

[TinyEKF](https://github.com/simondlevy/TinyEKF): Lightweight C/C++ Extended
Kalman Filter.

## Hardware

MCU:
[Raspeberry Pi RP2040](https://pip-assets.raspberrypi.com/categories/814-rp2040/documents/RP-008371-DS-1-rp2040-datasheet.pdf?disposition=inline)

MEMS IMU (accelerometer, gyroscope, magnetometer):
[Xsens MTI-3-5A-T](https://mtidocs.xsens.com/mti-1-series)

GNSS kit (dual antenna):
[Septentrio MOSAIC-GO H](https://shop.septentrio.com/en/shop/mosaic-go-h-heading-gnss-module-evaluation-kit)

## Firmware

[FreeRTOS](https://www.freertos.org/)

## Project Creation

This project was created with:

```bash
pio project init -b pico
```

### FreeRTOS Setup Steps

To configure PlatformIO for FreeRTOS on the RP2040, the environment in `platformio.ini` is set up to use the Earle Philhower core:

1. **Set Custom Platform**: Point to the Max Gerhardt PlatformIO registry to access the required core.
   ```ini
   platform = https://github.com/maxgerhardt/platform-raspberrypi.git
   ```
2. **Set the Core**: Specify the Earle Philhower core instead of the default Mbed OS core.
   ```ini
   board_build.core = earlephilhower
   ```
3. **Enable FreeRTOS Flag**: Instruct the core to compile and link the built-in FreeRTOS kernel.
   ```ini
   build_flags = -D PIO_FRAMEWORK_ARDUINO_ENABLE_FREERTOS
   ```
