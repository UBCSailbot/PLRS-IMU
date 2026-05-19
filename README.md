# Polaris IMU

This repository contains the source code for Polaris' IMU project.

## Setup

This project's toolchain is based on [PlatformIO](https://platformio.org/),
configured for the rp2040.

### Flakes

If you use a nix based package management, a dev shell is provided in
`flake.nix`. If you do not already use [direnv](https://direnv.net/), this can
automatically load and unload your development environment.

## Hardware

MCU:
[Raspeberry Pi RP2040](https://pip-assets.raspberrypi.com/categories/814-rp2040/documents/RP-008371-DS-1-rp2040-datasheet.pdf?disposition=inline)
MEMS IMU (accelerometer, gyroscope, magnetometer):
[Xsens MTI-3-5A-T](https://mtidocs.xsens.com/mti-1-series) GNSS kit (dual
antenna):
[Septentrio MOSAIC-GO H](https://shop.septentrio.com/en/shop/mosaic-go-h-heading-gnss-module-evaluation-kit)

## Firmware

[FreeRTOS](https://www.freertos.org/)
