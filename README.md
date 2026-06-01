# Polaris IMU

[![CI](https://github.com/UBCSailbot/PLRS-IMU/actions/workflows/ci.yml/badge.svg)](https://github.com/UBCSailbot/PLRS-IMU/actions/workflows/ci.yml)
[![License](https://img.shields.io/github/license/UBCSailbot/PLRS-IMU)](LICENSE)
[![C++23](https://img.shields.io/badge/C%2B%2B-23-blue.svg)](https://en.cppreference.com/w/cpp/23)
[![RP2040](https://img.shields.io/badge/platform-RP2040-red.svg)](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)

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

Install Nix using the Determinate installer:

```bash
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
mkdir -p ~/.config/nix/
echo "experimental-features = nix-command flakes" >> ~/.config/nix/nix.conf
```

Then install direnv through Nix (the apt package is too old to support `use flake`):

```bash
nix profile add nixpkgs#direnv
echo '# Direnv shell hook'
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
exec bash
direnv allow
```

### macOS

Install Nix using the Determinate installer:

```bash
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
mkdir -p ~/.config/nix/
echo "experimental-features = nix-command flakes" >> ~/.config/nix/nix.conf
```

Then install direnv through Nix:

```bash
nix profile add nixpkgs#direnv
echo '# Direnv shell hook'
echo 'eval "$(direnv hook zsh)"' >> ~/.zshrc
exec zsh
direnv allow
```

### Windows

The recommended path is WSL2 with Ubuntu 24.04. Install WSL2 from PowerShell:

```powershell
wsl --install -d Ubuntu-24.04
```

Once in WSL, follow the [Linux](#linux) instructions above.

For flashing, the RP2040 needs to be forwarded from Windows into WSL using
[usbipd-win](https://github.com/dorssel/usbipd-win):

```powershell
usbipd list                  # find the RP2040 bus ID
usbipd bind --busid <id>
usbipd attach --wsl --busid <id>
```

### Set up Git and GitHub

Skip this section if you've already configured git and your GitHub credentials.

#### Configure Git:

```bash
git config --global user.email "you@example.com"
git config --global user.name "Your Name"
git config --global core.autocrlf false # Very important you don't miss this on WSL!
```

#### Configure the gh Helper (For Logging Into GitHub):

Linux:

```bash
sudo apt update && sudo apt install gh
```

macOS:

```bash
brew install gh
```

Then authenticate:

```bash
gh auth login
```

### Build, test, and flash

```bash
make test    # host Unity tests
make build   # build firmware
make upload  # flash the RP2040
make format  # clang-format all sources in-place
```

Run `make` with no arguments to list all targets.

### Python sim

A Python harness at `sim/` wraps the C++ EKF for offline visualization
and tuning. The same filter code that ships on the RP2040 runs in the
sim, so Q values picked offline transfer faithfully. Synthetic
trajectories, injectable IMU and GNSS noise, and an overlay plot of
truth / open-loop / GNSS / EKF estimate ship out of the box.

Quick start:

```bash
make sim                          # default: constant_turn scenario
make sim SCENARIO=sinusoidal      # or step_turns, static
make sim-example EXAMPLE=static   # canned examples with hand-tuned params
make sim-test                     # pytest suite
make sim-format                   # ruff format + check
```

Tune noise and EKF parameters from the CLI:

```bash
cd sim
uv run python -m plrs_sim sim step_turns \
    --duration 60 --seed 7 \
    --gyro-bias 0.02 --gnss-std 2.0 \
    --q-heading 0.05 \
    --save /tmp/step_turns.png
```

A `0` for any `--gyro-*` or `--gnss-std` flag disables that effect.
Pass `--no-show` for headless runs. See `python -m plrs_sim sim --help`
for the full flag list.

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

## Tuning

See [docs/tuning.md](docs/tuning.md) for theory, tradeoffs, and the
record-and-replay workflow. Summary:

The filter has four parameters in `TinyEkfFilter::Config`.

`q_heading_deg2` and `q_bias_deg2_s2` are the load-bearing knobs. Derive
starting values from the MTi-3 datasheet rather than guessing:

- `q_heading` — gyro noise density (deg/s/√Hz) squared, scaled by dt
- `q_bias` — in-run bias stability (deg/s, from the Allan deviation plot) squared

`p0_heading_deg2` and `p0_bias_deg2_s2` only affect convergence from startup;
set them large.

The efficient tuning workflow: capture a session with the serial logger
(milestone 1), replay it through the Python sim (milestone 2) with different
configs, then flash once.

To diagnose a live filter, log the innovation (`gnss_heading - filter_heading`).
Zero-mean innovations indicate a well-tuned filter; persistently large
innovations mean Q is too small.

## Hardware

MCU:
[Raspberry Pi RP2040](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)

MEMS IMU (accelerometer, gyroscope, magnetometer):
[Xsens MTi-3-5A-T](https://mtidocs.xsens.com/mti-1-series)

GNSS kit (dual antenna):
[Septentrio mosaic-go H](https://shop.septentrio.com/en/shop/mosaic-go-h-heading-gnss-module-evaluation-kit)

## Firmware

[FreeRTOS](https://www.freertos.org/)

## Roadmap

| Milestone | Issues |
|---|---|
| [Python Logger](https://github.com/UBCSailbot/PLRS-IMU/milestone/1) | Serial capture script, log format, replay utility |
| [Python EKF Sim](https://github.com/UBCSailbot/PLRS-IMU/milestone/2) | pybind11 bindings, synthetic data generator, sim runner, plotter |
| [HIL Testing](https://github.com/UBCSailbot/PLRS-IMU/milestone/3) | On-device test suite, PIO Remote agent, CI integration |

## License

[GNU General Public License v3.0](LICENSE)
