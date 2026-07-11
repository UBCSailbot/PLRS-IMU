# Cleanup audit

Remaining items from the repo-wide terseness/composability audit, in priority
order. Each lands as its own test-green commit. Validate refactors that touch
the wire format or filter behavior the same way the telemetry refactor was
validated: golden-diff the output against the pre-change commit in a git
worktree (byte-identical telemetry, record-identical parses), plus the full
Python and native suites.

## 1. One home for angle wrapping (Python)

`wrap180` in live.py (`math.remainder`), `_wrap180` in plot.py (modulo), and
`_wrapped_residual` in tests/test_runner.py all wrap to +-180 independently.
Move to a small `angles.py` (scalar + ndarray `wrap180`, `seam_broken`) and
use it everywhere. Caution: the two implementations differ exactly at the
seam (`math.remainder(180, 360)` returns 180.0; the modulo form returns
-180.0). Pick one convention deliberately, match `fusion::wrap180` in
lib/fusion/fusion.h, and pin it with a boundary test.

## 2. Shared heading dead-reckoner

runner.py `run()` and live.py `MonitorState._update` each hand-roll the same
integrator (heading state, prev timestamp, `heading -= gyro_z * RAD_TO_DEG *
dt`). The copies have already diverged twice: sign (fixed in the convention
commits) and wrapping (live wraps to +-180, runner does not). Extract a
`HeadingDeadReckoner` with `anchor(heading_deg)` / `step(t_ms,
gyro_z_rad_s)`; the differing anchor policies stay in the callers.

## 3. Name the compass/ENU conversion seam

The "compass heading is negated ENU yaw" negation appears inline in four
places: lib/fusion/ekf_filter.h (predict and mag update), sim source.py,
runner.py, live.py. It cannot be shared across languages, but each side can
name it once (a helper pair in attitude.py; the C++ already comments it) and
point at docs/attitude.md as the single statement of the convention. The
regression tests that lock it: test_source.py
`test_no_noise_passes_clean_samples` / `test_mti_orientation_carries_enu_yaw`,
test_runner.py `test_openloop_tracks_truth_without_noise` /
`test_tracks_through_realistic_noise_with_mag_aiding`, and the C++
`test_predict_integrates_gyro`.

## 4. Split live.py

782 lines holding three concerns: wire records + codec (now self-contained
after the telemetry refactor), MonitorState/Series, and the Qt views. Split
into `telemetry.py`, monitor state, and views once items 1-2 have settled
where the shared helpers live. Mechanical; update test imports.

## 5. Decompose plot_animate

plot.py `plot_animate` handles the interactive loop, GIF writer, PNG final
frame, and FFMpeg path in one function with backend switching threaded
through. Extract the writers; keep the backend juggling at the edges.

## 6. Staleness sweep

CLAUDE.md "Current scope / deferred" still describes phase one (the xbus
host tests it lists as remaining have existed for a long time); sweep it and
docs/*.md against the current code.

## Considered and rejected

Unifying the three C++ protocol layers (xbus, SBF, rudder COBS): they share
a shape (checksum, framing, byte-fed parser) but differ in framing, escaping,
and error semantics; a shared framework would be speculative complexity. The
firmware task files and the EKF were audited clean.
