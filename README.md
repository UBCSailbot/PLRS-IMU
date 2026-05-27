# Polaris IMU

[![CI](https://github.com/georgesleen/PLRS-IMU/actions/workflows/ci.yml/badge.svg)](https://github.com/georgesleen/PLRS-IMU/actions/workflows/ci.yml)

Driver and sensor fusion firmware for the Xsens MTi-3 IMU on an RP2040,
targeting Polaris. Phase one of a three-phase arc: IMU driver → dual-band GNSS
driver → Kalman filter fusion.

Runs on **FreeRTOS / RP2040** (Arduino framework via PlatformIO, earlephilhower
core). The protocol layer (`lib/mti_imu/xbus_protocol.h`) has no Arduino
dependency and compiles under plain `g++ -std=c++23` for host testing.

## Getting started

### Nix (Linux / macOS)

A dev shell is provided. With [direnv](https://direnv.net/) installed, entering
the directory loads it automatically:

```bash
echo "use flake" > .envrc && direnv allow
```

Without direnv:

```bash
nix develop
```

The shell hook regenerates `compile_commands.json` and `.clangd` on entry so
your editor's LSP is always in sync.

### Linux — without Nix (Ubuntu 24.04+)

```bash
sudo apt-get install -y clang-format g++ python3-pip
pip install platformio
```

> **Ubuntu 22.04:** the default `g++` is version 11, which lacks `std::expected`.
> Install `g++-12` or later and ensure it is first on your `PATH`.

### macOS — without Nix

```bash
brew install llvm platformio
export PATH="$(brew --prefix llvm)/bin:$PATH"   # add to your shell profile
```

Xcode's bundled `clang-format` is usually outdated; the Homebrew LLVM formula
provides a current version.

### Windows

**WSL2 is the recommended path.** The native toolchain setup is more involved
and the native test environment has not been validated on Windows.

#### WSL2 (recommended)

1. Install [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install) with
   Ubuntu 24.04:
   ```powershell
   wsl --install -d Ubuntu-24.04
   ```
2. Open the Ubuntu terminal and follow the **Linux** instructions above.
3. Clone the repo inside WSL (not on the Windows filesystem — performance on
   `/mnt/c/…` is poor):
   ```bash
   git clone <repo-url> ~/PLRS-IMU && cd ~/PLRS-IMU
   ```
4. For flashing firmware, the RP2040 USB device needs to be forwarded from
   Windows into WSL. Install
   [usbipd-win](https://github.com/dorssel/usbipd-win), then from PowerShell:
   ```powershell
   usbipd list                  # find the RP2040 bus ID, e.g. 2-3
   usbipd bind --busid 2-3
   usbipd attach --wsl --busid 2-3
   ```
   Then `pio run -e pico -t upload` from the WSL terminal works as normal.

#### Native Windows (advanced)

Install Python from [python.org](https://python.org), then:

```powershell
pip install platformio
```

Install `clang-format` from the
[LLVM releases page](https://github.com/llvm/llvm-project/releases) and add it
to your `PATH`.

For the native test environment you also need a C++ compiler with C++23
support. [MSYS2](https://www.msys2.org/) with MinGW-w64 (GCC 13+) is the path
of least resistance:

```bash
# inside the MSYS2 MinGW64 shell
pacman -S mingw-w64-x86_64-gcc
```

PlatformIO's native environment will pick up `gcc` / `g++` from `PATH`.

## Build, test, and flash

```bash
pio test -e native          # host Unity tests — run after every change
pio run  -e pico            # build firmware
pio run  -e pico -t upload  # flash the RP2040
```

A red test suite is a hard stop. CI enforces this on every pull request.

## Project layout

```
lib/mti_imu/xbus_protocol.h   Core Xbus protocol. No Arduino dependency.
src/main.cpp                   Firmware entry point (blinky placeholder).
test/test_xbus/test_main.cpp   Unity host tests.
platformio.ini                 pico (firmware) + native (host tests) envs.
flake.nix                      Nix dev shell (clang-tools, platformio, python).
.clang-format                  Style config: BasedOnStyle LLVM.
.github/workflows/ci.yml       Format check + native test jobs.
```

## Coding style

### Formatting

Style is `BasedOnStyle: LLVM` (2-space indent, 80-column limit), enforced by
CI. Before pushing, format all changed files:

```bash
# from helix
:fmt

# or from the shell
clang-format -i <file>
```

The CI job runs `clang-format --dry-run --Werror` on everything under `lib/`,
`src/`, and `test/`. A formatting violation fails the PR.

### Naming conventions

| Kind | Convention | Example |
|---|---|---|
| Types, structs, classes, enum classes | `PascalCase` | `ByteSpan`, `DataId`, `Parser` |
| Enum members | `PascalCase` | `MID::GoToConfig`, `State::Preamble` |
| Free functions and methods | `snake_case` | `find_data`, `mid_frame` |
| Constants (`constexpr`, `static constexpr`) | `UPPER_SNAKE_CASE` | `BID_MASTER`, `FRAME_TIMEOUT` |
| Private member variables | `_snake_case` | `_state`, `_last_advance` |
| Local compile-time test values | `kPascalCase` | `kGoToConfig`, `kOversize` |

### Design rules

These are derived directly from the shape of the protocol layer.

- **`ByteSpan` for all (pointer, length) pairs.** `std::span<const uint8_t>` is
  the only way a buffer travels through an API.

- **Return values, not out-parameters.** `encode` returns
  `std::optional<Encoded>`; `Packet::command` returns
  `std::expected<Packet, const char*>`. Nothing is written through a pointer
  parameter.

- **`std::expected` when failure has a reason; `std::optional` when it
  doesn't.** A malformed command has a diagnostic. A missing sub-packet in a
  data frame is just absent.

- **Classes only for real invariants or mutable state.** `Parser` earns a class
  because it owns a running checksum, a byte buffer, and a timeout. Stateless
  helpers are free functions.

- **`constexpr` where it buys compile-time verification.** `checksum`, `encode`,
  and `Packet::command` are all `constexpr` so frame correctness can be
  `static_assert`ed rather than only tested at runtime.

- **`std::bit_cast` for type punning; never pointer casts or `memcpy`.** See
  `read_f32_big_endian` — the cast is defined behaviour and `constexpr`-capable.

- **Named constants for every magic number.** `BID_MASTER`, `PREAMBLE`,
  `SUBPACKET_HEADER`, etc. A raw literal in a guard or signature is a hard no.

- **No Arduino dependency in the protocol layer.** `xbus_protocol.h` must
  compile under `g++ -std=c++23` with no Arduino headers. Anything
  hardware-specific belongs in a separate firmware file behind `#ifdef ARDUINO`.

## Hardware

| Component | Part |
|---|---|
| MCU | [Raspberry Pi RP2040](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) |
| IMU | [Xsens MTi-3-5A-T](https://mtidocs.xsens.com/mti-1-series) (9-DoF MEMS: accel, gyro, mag) |
| GNSS | [Septentrio mosaic-go H](https://shop.septentrio.com/en/shop/mosaic-go-h-heading-gnss-module-evaluation-kit) (dual-antenna heading) |
| RTOS | [FreeRTOS](https://www.freertos.org/) via earlephilhower Arduino core |

## License

[GNU General Public License v3.0](LICENSE)
