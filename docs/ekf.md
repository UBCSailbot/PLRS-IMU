# Sensor fusion EKF

The filter blends the MTi-3 IMU and the dual-antenna GNSS into one running
estimate of the boat's heading, roll, and pitch. It's a Kalman filter: each step
it predicts where the boat is pointing from the gyro, then nudges that prediction
toward the sensor readings, all while tracking how much it trusts each.

It carries seven numbers:

    [heading, roll, pitch, gyro_bias_x, gyro_bias_y, gyro_bias_z, mag_offset]

the three angles in degrees, a slow-drift term for each of the three gyro axes
in deg/s, and a mag-offset term in degrees (see below).

## How it runs

The IMU gives us its orientation as a quaternion (a compact four-number way to
store a rotation) and the raw spin rates from its gyro. Every IMU sample, the
filter turns those spin rates into how fast heading, roll, and pitch are each
changing, and advances the three angles by that much.

Getting that conversion right is what keeps heading correct when the boat heels.
A level gyro spinning about its vertical axis reads pure heading change; a heeled
gyro mixes that turn into roll and pitch, and the conversion untangles it. The
drift terms soak up the gyro's slow per-axis bias so it doesn't quietly
accumulate into the angles. The bias is subtracted in the body frame, before the
conversion, so each axis keeps correcting itself whatever the boat's attitude.

Three measurements pull the estimate back toward reality:

- the **MTi quaternion** corrects roll and pitch (it arrives with every IMU
  sample),
- a **GNSS fix** corrects heading,
- the **MTi magnetic yaw** nudges heading between fixes, but through the offset
  term (below) so it can never own absolute heading.

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
blind spot at 90 deg of pitch: heading is undefined pointing straight up, and the
kinematics that map the gyro into heading rate blow up there. A boat never trims
near vertical, so this never bites while sailing, but bench handling and a
knockdown do reach it. Two guards keep it safe: the pitch feeding the kinematics
is clamped (so the filter stays finite and re-anchors on the way back down
instead of latching to NaN), and heading is reported invalid past that pitch, so
the rudder ignores it rather than steering on a meaningless bearing.

**The MTi corrects attitude, not heading.** Its orientation arrives about 100
times a second. If it also corrected heading it would drown out the slower but
more accurate GNSS, so heading stays GNSS-owned.

**Heading is built from the filter's own roll/pitch, not the raw quaternion.**
That's why the MTi is treated as a measurement and not gospel: if magnetic
interference throws off its orientation, the filter has already smoothed it
before it can drag heading around.

**A drift term per gyro axis.** The vertical (Z) bias is the one that reads as
heading drift near level trim, but at heel a body-frame X or Y bias projects into
heading rate too, so an unmodelled one leaks out as a slow heading ramp whenever
the boat is heeled over. The X and Y biases cost almost nothing to observe: the
100 Hz roll/pitch measurements pin them directly, without waiting on GNSS. Only
the Z bias depends on a heading measurement to be seen.

**A mag-offset term, so the magnetometer can help without taking over.** The
MTi's magnetic yaw is not true heading: declination, boat iron, and the
ENU-to-compass frame constant all sit between them, and indoors that gap wanders.
The offset term absorbs that difference, so the mag stiffens heading between GNSS
fixes and keeps the Z bias observable through an outage, yet can never drag
absolute heading away from GNSS. `docs/tuning.md` covers how loosely it's tuned
and why, and the optional outage pin that lets a clean mag hold heading through
a long GNSS gap instead of coasting on the gyro.

## Tuning

How much the filter trusts the gyro versus the measurements lives in
`tuning.toml`. `docs/tuning.md` explains the knobs.
