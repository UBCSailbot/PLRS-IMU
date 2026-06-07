# Attitude

## What this estimates

- **Roll (heel)**: rotation about body X, positive = starboard down.
- **Pitch (trim)**: rotation about body Y, positive = bow up.

Exposed as `FusionOutput.roll_deg` / `pitch_deg`. ZYX intrinsic Euler in
ENU. Variances are `FLT_MAX` until PR3 promotes attitude to filter state.

## Frames

- **ENU world**: X east, Y north, Z up.
- **Boat body**: X bow, Y port, Z deck-up.
- **IMU body**: boat body rotated by the static mount.

`MountRotation::boat_to_imu` is the unit quaternion taking boat-body
vectors to IMU-body. Identity until calibrated.

## The heel-coupling problem

Integrating raw body-Z gyro as heading only works when upright. Heeled by
`phi`, the world yaw rate arrives at the gyro as:

```
body_y = omega_z * sin(phi)
body_z = omega_z * cos(phi)
```

At 20 deg heel, body Z reads 94% of the truth; the rest leaks into
body Y. PR1 fixes this in `predict` by reading
`world_yaw_rate(orientation, gyro_xyz)` instead. Math in
`lib/fusion/attitude.h`, mirrored in `sim/plrs_sim/attitude.py`.

## Attitude source

| Approach | Verdict |
|---|---|
| MTi onboard quaternion as truth | Picked for PR1. Magnetometer-calibrated AHRS, no extra math. |
| Roll our own | Duplicates Xsens for no win. |
| MTi as a measurement, gyro propagates | Picked for PR3. Lets the filter reject outliers; needs attitude in state. |

## Calibrating the mount

1. Tie up on flat water. The boat is level by construction.
2. Log MTi quaternion at 100 Hz for ~10 s, average, renormalize.
3. Store the inverse as `MountRotation::boat_to_imu`.

If the boat moves during logging, the calibration absorbs the motion.
