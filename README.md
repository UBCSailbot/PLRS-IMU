# Polaris IMU

[![CI](https://github.com/UBCSailbot/PLRS-IMU/actions/workflows/ci.yml/badge.svg)](https://github.com/UBCSailbot/PLRS-IMU/actions/workflows/ci.yml)

This repository contains the IMU firmware for Polaris.

## Setup

This project's toolchain is based on [PlatformIO](https://platformio.org/),
configured for the RP2040.

### Nix

A dev shell is provided in `flake.nix`. If you use
[direnv](https://direnv.net/), it will load automatically when you enter the
directory. Otherwise:

```bash
nix develop
```

### Linux

```bash
sudo apt-get install -y clang-format g++ python3-pip
pip install platformio
```

Ubuntu 22.04's default `g++` is version 11, which lacks `std::expected`. Install
`g++-12` or later.

### macOS

```bash
brew install llvm platformio
export PATH="$(brew --prefix llvm)/bin:$PATH"
```

### Windows

The recommended path is WSL2 with Ubuntu 24.04, then follow the Linux
instructions above. Install WSL2 from PowerShell:

```powershell
wsl --install -d Ubuntu-24.04
```

For flashing, the RP2040 needs to be forwarded from Windows into WSL using
[usbipd-win](https://github.com/dorssel/usbipd-win):

```powershell
usbipd list                  # find the RP2040 bus ID
usbipd bind --busid <id>
usbipd attach --wsl --busid <id>
```

### Build, test, and flash

```bash
pio test -e native          # host Unity tests
pio run  -e pico            # build firmware
pio run  -e pico -t upload  # flash the RP2040
```

### Git hooks

Pre-push hooks are tracked in `hooks/` and mirror the CI checks (clang-format
+ native tests). If you use direnv, they are installed automatically when you
enter the directory. Without direnv, install them once manually:

```bash
ln -sf ../../hooks/pre-push .git/hooks/pre-push
```

## Coding style

Formatting is enforced by CI. Format changed files with `clang-format -i <file>`
before pushing.

Naming conventions: types and enum members in `PascalCase`, free functions and
methods in `snake_case`, constants in `UPPER_SNAKE_CASE`, private members with a
leading underscore (`_state`, `_sum`).

A few design rules worth knowing before contributing:

- `ByteSpan` (`std::span<const uint8_t>`) is preferred to `(ptr, len)` pairs.
- Outputs are returned, not written through out-parameters.
- Use `std::expected<T, E>` when failure has a meaningful reason,
  `std::optional` when it doesn't.
- `std::bit_cast` for type punning — no pointer casts, no `memcpy`.
- Named constants for all magic numbers.
- Statically allocate all memory instead of using malloc.

## Overview

The goal is heading accuracy of ≤2° on Polaris by fusing IMU and GNSS
measurements through a Kalman filter (likely an Extended Kalman Filter given
time constraints).

### Reference Projects

[TinyEKF](https://github.com/simondlevy/TinyEKF): Lightweight C/C++ Extended
Kalman Filter.

## Hardware

MCU:
[Raspberry Pi RP2040](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)

MEMS IMU (accelerometer, gyroscope, magnetometer):
[Xsens MTi-3-5A-T](https://mtidocs.xsens.com/mti-1-series)

GNSS kit (dual antenna):
[Septentrio mosaic-go H](https://shop.septentrio.com/en/shop/mosaic-go-h-heading-gnss-module-evaluation-kit)

## Firmware

[FreeRTOS](https://www.freertos.org/)

## License

[GNU General Public License v3.0](LICENSE)
