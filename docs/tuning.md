# EKF Tuning

## Parameters

The values live in `tuning.toml`, the one place they are set. Each maps to a
field of `TinyEkfFilter::Config`:

| Parameter | Physical meaning |
|---|---|
| `q_heading_deg2` | Heading process noise per step (deg^2) |
| `q_roll_deg2` / `q_pitch_deg2` | Roll / pitch process noise per step (deg^2) |
| `q_bias_deg2_s2` | Gyro bias random walk per step ((deg/s)^2) |
| `p0_*` | Initial uncertainty for each state; only affects startup convergence |
| `mti_roll_variance_deg2` / `mti_pitch_variance_deg2` | Trust in the MTi's roll / pitch each sample (deg^2) |

Q controls how much the filter trusts the gyro integration relative to the
measurements. P0 only affects convergence from startup and can be set large.
GNSS heading noise is not tuned here -- the receiver reports it per fix as
`heading_variance_deg2`.

## Deriving Q from the MTi-3 datasheet

Rather than tuning Q blind, two datasheet specs map directly to the Q diagonal.

**Angle Random Walk (ARW)** is the gyro's white noise floor, read from the -1/2
slope of the Allan deviation plot at tau=1s. Reported in deg/s/sqrt(Hz) (or
equivalently deg/sqrt(hr)). Discretize to your predict step interval dt:

```
q_heading = (ARW_deg_per_s_per_sqrtHz)^2 * dt_s
```

**In-run bias instability** is the flat floor of the Allan deviation curve, in
deg/s. It represents how fast the bias drifts:

```
q_bias = (bias_instability_deg_per_s)^2 * dt_s
```

These are conservative starting points. In practice Q may need to be scaled up
slightly because it must cover not just sensor noise but also modeling error:
imperfect ENU alignment, vibration, flexible mast dynamics, etc. Think of Q as
"how wrong can my process model be per step" rather than "what is my sensor
noise."

## R: fixed vs adaptive

R is the measurement noise covariance -- how much the filter trusts each GNSS
heading fix.

**Fixed R**: use the mosaic-go-H's nominal heading accuracy (~0.3 deg RMS,
so R ~= 0.09 deg^2). Simple, but ignores variation in satellite geometry.

**Adaptive R**: pass `AttCovEuler.cov_headhead` directly as R on each update
step. The receiver computes this per-fix, so the filter automatically
down-weights measurements taken when DOP is high. This is what
`GnssSample.heading_variance_deg2` is designed to carry, and it is the
recommended approach.

The tradeoff is transparency: a fixed R is easy to reason about; adaptive R
means the filter's effective trust in GNSS changes with conditions, which can
be harder to debug if something goes wrong. Log `heading_variance_deg2`
alongside everything else.

## What goes wrong when Q is off

**Q too small** (filter over-trusts the gyro): GNSS corrections are
under-weighted. The output looks smooth and stable in open sky, but heading
drifts during a GNSS outage and snaps back with a visible step when GNSS
returns. The innovation sequence shows large, persistent values.

**Q too large** (filter over-trusts GNSS): the gyro integration is effectively
ignored. You lose the main benefit of IMU fusion -- smooth high-rate output
between GNSS fixes. During outages the heading freezes immediately rather than
coasting. The innovation sequence collapses toward zero.

The goal is the smallest Q at which the filter tracks real maneuvers (tacking,
gybing) without visible lag.

## Diagnosing with the innovation sequence

The innovation is the gap between what GNSS reports and what the filter
predicted:

```
v = gnss_heading - x[0]_predicted
```

Log this alongside filter output. A well-tuned filter has:

- **Zero mean** -- no systematic bias between prediction and measurement
- **Consistent magnitude** -- most values within +/- sqrt(P[0][0] + R)

A tighter diagnostic is the Normalized Innovation Squared (NIS):

```
NIS = v^2 / (P[0][0] + R)
```

For a scalar measurement (m=1), NIS is chi-squared with 1 degree of freedom.
Roughly 95% of samples should fall below 3.84. Consistently above means Q is
too small or R is too large (filter is overconfident in its prediction).
Consistently below means Q is too large or R is too small.

NIS is more reliable than RMSE alone because it accounts for the filter's own
uncertainty estimate. A filter that inflates P to cover its errors will have
low RMSE but consistently low NIS -- it is technically consistent but not well
calibrated.

## P0: initialization

P0 only matters during the first few GNSS cycles after startup. Set both values
large so the filter converges aggressively:

```cpp
p0_heading_deg2 = 1000.0; // very uncertain about initial heading
p0_bias_deg2_s2 = 1.0;    // uncertain about gyro bias
```

After a handful of updates P0 is effectively overwritten by Q and R, and the
initial value is forgotten. Do not spend time tuning it.

## Tuning with recorded data

The mosaic-go-H gives GNSS heading at ~0.3 deg RMS from the dual-antenna
baseline. This is accurate enough to treat as ground truth when tuning offline.

Workflow (once milestones 1 and 2 are in place):

1. Capture a session -- log both ImuSample and GnssSample streams with
   timestamps, along with `AttCovEuler.cov_headhead` per fix
2. Replay through the Python sim with a candidate Config
3. Compute RMSE between filter heading and GNSS heading over the session
4. Sweep Q values and select the minimum-RMSE configuration
5. Inspect the NIS sequence to confirm the filter is consistent, not just
   lucky on RMSE
6. Flash once with the final Config

Cover these scenarios in your recorded sessions:

- **Straight-line sailing** -- checks bias estimation; the filter should
  converge on a stable bias within a few minutes
- **Tacking and gybing** -- fast heading changes expose Q-too-small as lag
  between the filter output and GNSS
- **Simulated GNSS outage** -- briefly block or disconnect an antenna; the
  filter should coast on the gyro with slow, predictable drift rather than
  a sudden step

## Tradeoff summary

| Setting | Q too small | Q too large |
|---|---|---|
| GNSS outage | Drifts, then snaps on recovery | Freezes immediately |
| Fast maneuvers | Lags behind reality | Tracks but noisily |
| Innovation NIS | Persistently high | Persistently low |
| Steady-state RMSE | May appear good | Degrades |

The right Q is not the one that minimizes RMSE in calm conditions -- it is the
one that minimizes RMSE across the full range of conditions the boat will
encounter, with a well-behaved NIS sequence to prove the filter knows what it
does not know.
