# Attitude

Heading, roll, and pitch are reported on `FusionOutput` as ZYX intrinsic Euler
angles in the ENU frame. This page covers the frame conventions and the mount
calibration; how the filter estimates them is in `ekf.md`.

## What the angles mean

- **Roll (heel)**: rotation about body X, positive = starboard down.
- **Pitch (trim)**: rotation about body Y, positive = bow up.
- **Heading (yaw)**: rotation about world Z.

## Frames

- **ENU world**: X east, Y north, Z up.
- **Boat body**: X bow, Y port, Z deck-up.
- **IMU body**: boat body rotated by the static mount.

`MountRotation::boat_to_imu` is the unit quaternion taking boat-body vectors to
IMU-body. The filter rotates the MTi quaternion by it to recover boat attitude.
It comes from the `[imu_mount]` section of `tuning.toml` (as ZYX Euler offsets)
and is identity until calibrated.

## Why heel couples into heading

A body-Z gyro only reads pure heading change when the boat is upright. Heeled by
`phi`, a world yaw rate `omega_z` arrives at the gyro split across two axes:

```
body_y = omega_z * sin(phi)
body_z = omega_z * cos(phi)
```

At 20 deg heel, body Z reads 94% of the turn and the rest leaks into body Y. The
filter untangles this from its own roll/pitch estimate (see `ekf.md`); the math
is in `lib/fusion/attitude.h`, mirrored in `sim/plrs_sim/attitude.py`.

## Calibrating the mount

1. Tie up on flat water. The boat is level by construction.
2. Log the MTi quaternion at 100 Hz for ~10 s, average, renormalize.
3. Read off the roll, pitch, and yaw of that average and store them as
   `mount_roll_deg`, `mount_pitch_deg`, `mount_yaw_deg` under `[imu_mount]` in
   `tuning.toml`.

If the boat moves during logging, the calibration absorbs the motion.

The sim exercises the mount: it tilts the synthesized MTi reading by the same
offset, so a run with a nonzero mount confirms the filter recovers boat
attitude (`python -m plrs_sim sim heeling_tack --mount-roll 10`).
