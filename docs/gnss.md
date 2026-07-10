# GNSS heading

The boat's heading comes from a dual-antenna GNSS receiver. With two antennas a
fixed distance apart, the receiver works out the compass direction of the line
between them, and that line is bolted to the boat, so its direction is the boat's
heading. This is the measurement that keeps the filter's heading honest (`ekf.md`
explains how it folds in).

## What the receiver gives us

The receiver reports its attitude in two blocks:

- **AttEuler** carries the heading (and pitch; roll too if a third antenna is
  fitted) as the true heading of the antenna baseline, `0..360` degrees.
- **AttCovEuler** carries how sure it is, as a variance per angle.

It also flags, in a mode field, whether it currently has a heading solution at
all. Both blocks and their decoders live in `lib/septentrio_gnss/sbf_blocks.h`.

## The bridge

`lib/fusion/gnss_bridge.h` turns one of those attitude readings into the heading
measurement the filter consumes. It does three small things:

- **Frame.** The receiver's heading runs `0..360` clockwise from north; the
  filter works in `-180..180`. The bridge converts, subtracting the mount offset
  below along the way.
- **Validity.** If the mode field says no heading solution, or the heading is the
  receiver's not-a-number sentinel, the sample is marked invalid and the filter
  skips it.
- **Uncertainty.** The heading variance comes straight from AttCovEuler, falling
  back to a configured value when the receiver reports none.

This is the one place the fusion code reaches into the GNSS decode layer; the
decode layer knows nothing about fusion.

## The baseline offset

The receiver reports the heading of the line between the two antennas. If that
line isn't parallel to the boat's centreline, every heading is wrong by the same
fixed angle. We measure that angle once and subtract it in the bridge
(`baseline_offset_deg` in `tuning.toml`).

To find it: hold the boat on a known bearing and compare it to the heading the
receiver reports; the difference is the offset. The receiver has its own offset
command, but we leave that at zero so the correction lives in exactly one place.

## Turning it on

The receiver streams nothing useful until told to. The typed builders in
`lib/septentrio_gnss/command.h` assemble the setup commands:

- `set_gnss_attitude(MultiAntenna)` — compute attitude from the two antennas on
  a single receiver. Without this the receiver reports `NO_ATTITUDE` and no
  heading is available.
- `set_sbf_output(...)` — stream `AttEuler` and `AttCovEuler` on a port at a rate.
- `set_attitude_offset(0, 0)` — leave the receiver's own offset at zero.

Each returns a ready-to-send command (an ASCII line ending in a carriage return);
the wire format and reply parsing are in the same directory. The GNSS task sends
these on every boot and retries until acknowledged, so a factory-reset or
reflashed receiver still comes up producing headings.

## Tuning

`baseline_offset_deg` and the fallback heading variance live in the `[gnss]`
section of `tuning.toml`; `docs/tuning.md` covers the knobs. Heading and roll/
pitch conventions are in `docs/attitude.md`.
