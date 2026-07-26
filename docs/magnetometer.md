# Magnetometer characterization and the no-GPS heading coast

When GNSS heading drops out, the filter holds heading on the gyro plus the MTi's
magnetometer-referenced yaw. With no GPS and no trustworthy mag, absolute heading
is unobservable and drifts with gyro bias; that is physics, not a tuning bug. So
"how long can we coast without GPS" comes down to one question: **how good is the
magnetometer on this boat?** This is how to answer it.

## The tools, in one command

Run the sim with no arguments for a guided menu:

```
cd sim && uv run python -m plrs_sim
```

It offers: run a simulation, watch or replay hardware telemetry, **analyze a
magnetometer capture**, and show the sensor mounting. Everything below is also a
plain subcommand (`sim`, `monitor`, `analyze`) if you prefer flags.

The offline pipeline:

```
capture (I/M/G telemetry)  --replay-->  the real C++ filter   (tune what ships)
                           --analyze->  hard/soft iron + a verdict  (decide the pin)
```

## Hard iron vs soft iron (1/rev vs 2/rev)

Plot the mag heading error against true heading over a full turn:

- **Hard iron** is a constant field (a magnet, or a steady DC current). It shifts
  the field circle off-center, so the error cycles **once per revolution
  (1/rev)**. Correction is a fixed offset.
- **Soft iron** is permeable metal (steel, iron) that bends the ambient field
  into an ellipse. An ellipse looks the same rotated 180 deg, so the error cycles
  **twice per revolution (2/rev)**. Correction is a matrix.

The analyzer separates these by frequency: 0/rev is declination plus a fixed
offset (the `mag_offset` state absorbs it), 1/rev is hard iron, 2/rev is soft
iron, and whatever is left is noise, wander, and dynamic iron.

## Getting a capture

A characterization needs the whole boat, with all its iron, to rotate through
most of the compass while GNSS logs true heading. It must be the mag's **final
mounted position** with operational iron present. You do not have to sail:

- **Trailer circles.** On a trailer, tow it in slow circles in a lot. Full 360,
  clean sky. The easiest full-coverage option.
- **Passive swing at a mooring or anchor.** Leave it logging; wind and tide
  weathervane it through a range of headings over hours. Zero effort.
- **Dockside warping / a dinghy nudging the bow.** Even a partial 120-180 deg
  swing is usable; the fit just carries more uncertainty, which the tool flags.

Rotating the IMU alone does **not** work: the boat's iron is fixed in the boat
frame, so only the whole assembly turning in the earth's field characterizes it.

A trailer or on-the-hard capture misses *dynamic* iron (engine running, switched
loads); recheck in operation later.

To capture, record the telemetry stream to a file:

```
uv run python -m plrs_sim monitor --record captures/swing.log
```

Faithful tuning needs the IMU logged at the full predict rate, not the throttled
10 Hz monitor telemetry. Flash the raw-log firmware first, which drops the
throttle:

```
pio run -e pico_rawlog -t upload   # full-rate capture build
# ... record the swing ...
pio run -e pico -t upload          # back to normal 10 Hz telemetry
```

## Reading the analysis

```
uv run python -m plrs_sim analyze captures/swing.log
```

```
mag vs GNSS over 240 fixes, 12/12 sectors
  constant (0/rev, offset absorbs): -0.12 deg
  hard iron (1/rev):                2.50 deg
  soft iron (2/rev):                7.00 deg
  residual after 0/1/2:  1.10 deg rms (uncorrectable floor)
  offset removed only:   5.37 deg rms (iron left in)

VERDICT: soft iron (2/rev) dominates ... it is correctable. Calibrate it out ...
```

- **sectors**: heading coverage out of 12. Below ~10 the fit is not trustworthy;
  cover more of the compass.
- **1/rev, 2/rev**: hard and soft iron amplitudes. Which dominates tells you which
  kind of iron you have.
- **residual**: the part no calibration removes (noise, wander, dynamic iron).
  Small means the iron is a steady, correctable pattern.
- **offset removed only**: the heading-dependent error left if the iron is *not*
  corrected, i.e. what the mag costs during an outage that turns.

The verdict reads these into a recommendation about the outage pin.

## The outage pin

`mti_yaw.q_offset_outage_deg2` in `tuning.toml` (unset by default) pins the mag
offset once GNSS has been gone a few seconds, turning the mag into a genuine
heading hold for the outage instead of coasting on the gyro. It is only safe with
a clean or calibrated mag:

- **Clean mag** (low uncorrected error): enable it for long GPS-gap heading hold.
- **Correctable iron**: calibrate it out first (MTi field map, or our own
  correction), then enable.
- **Large wander** (high residual): leave it unset; heading fails safe.

Confirm time-stability with a long stationary log before trusting the pin: a
swing measures heading-dependence (iron), a stationary hold measures drift over
time (wander). Both feed the decision. See `docs/tuning.md` for the pin mechanics.

## Replaying a capture to tune

`ReplaySource` feeds a recorded capture through the same runner and C++ filter as
the sim, so a candidate `tuning.toml` is judged on real data and the value you
pick is the value that ships:

```python
from plrs_sim import ReplaySource
from plrs_sim.runner import run
from plrs_sim.tuning import load_tuning
from plrs_sim.monitor import replay_file

trace = run(ReplaySource(lines=replay_file(Path("captures/swing.log"))), load_tuning())
```

`monitor --replay captures/swing.log` replays it into the live viewer instead.

## Before hardware

The whole pipeline runs on synthetic captures. The sim models hard iron (1/rev)
and soft iron (2/rev) via `MagNoiseModel`, so a made-up "trailer circles" run with
soft iron reproduces this boat's suspected problem and dry-runs the analyzer, the
verdict, and the pin, with no MCU attached.
