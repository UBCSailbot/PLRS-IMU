# Sensor fusion EKF

The filter blends the MTi-3 IMU and the dual-antenna GNSS into one running
estimate of the boat's heading, roll, and pitch. It's a Kalman filter: each step
it predicts where the boat is pointing from the gyro, then nudges that prediction
toward the sensor readings, all while tracking how much it trusts each.

It carries four numbers:

    [heading, roll, pitch, gyro_z_bias]

the three angles in degrees, plus a slow-drift term for the vertical gyro in
deg/s.

## How it runs

The IMU gives us its orientation as a quaternion (a compact four-number way to
store a rotation) and the raw spin rates from its gyro. Every IMU sample, the
filter turns those spin rates into how fast heading, roll, and pitch are each
changing, and advances the three angles by that much.

Getting that conversion right is what keeps heading correct when the boat heels.
A level gyro spinning about its vertical axis reads pure heading change; a heeled
gyro mixes that turn into roll and pitch, and the conversion untangles it. The
drift term soaks up the gyro's slow bias so it doesn't quietly accumulate into
heading.

Two measurements pull the estimate back toward reality:

- the **MTi quaternion** corrects roll and pitch (it arrives with every IMU
  sample),
- a **GNSS fix** corrects heading.

Each correction touches one number at a time, which keeps the math cheap.
Headings are compared the short way around the compass, so a fix at -179 deg
against an estimate at 179 deg counts as 2 deg apart, not 358.

Until the first IMU sample arrives, roll and pitch report infinite uncertainty
(no estimate yet); heading does the same until the first GNSS fix.

## Why this shape

**Plain angles in the state, not a quaternion.** We could track orientation as a
quaternion inside the filter, but that needs extra bookkeeping TinyEKF (the
library we use) doesn't provide. Heading, roll, and pitch as plain angles fit it
directly and reuse the math in `attitude.h`. The one catch with plain angles is a
blind spot at 90 deg of pitch, which is harmless here since a boat never trims
near vertical.

**The MTi corrects attitude, not heading.** Its orientation arrives about 100
times a second. If it also corrected heading it would drown out the slower but
more accurate GNSS, so heading stays GNSS-owned.

**Heading is built from the filter's own roll/pitch, not the raw quaternion.**
That's why the MTi is treated as a measurement and not gospel: if magnetic
interference throws off its orientation, the filter has already smoothed it
before it can drag heading around.

**One drift term, not three.** It captures the gyro drift that matters for
heading. Roll and pitch don't need their own, because the MTi corrects them every
sample.

## Tuning

How much the filter trusts the gyro versus the measurements lives in
`tuning.toml`. `docs/tuning.md` explains the knobs.
