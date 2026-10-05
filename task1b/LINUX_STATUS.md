# Task 1B — Linux box status notes

## TL;DR
- `step_test.py` **works** on this Linux box. Run `20261005T120908Z_s1_step/summary.json` recommends `YAW_GAIN_K=0.0914, SPIN_SPEED=3.0, TURN_MIN_W=1.5`. That recommendation is applied in `task_1b_boilerplate.py`.
- `speed_test.py` **cannot produce `K_LIN` on this Linux box** (two issues below). The run folder `20261005T121001Z_s2_speed2` is included for the record; `k_lin: null` is expected, not a regression.
- `analyze_run.py` therefore has no valid Stage-2 speed-calibration run to compare against yet.

## Why step_test succeeded
Spin-in-place trials don't require clearance ahead, so surrounding walls don't matter. All 48 trials ran (8 wheel speeds × L/R × 3 repeats), ~192 s of sim time. `k = gyro/(2w)`, rise time, 90 deg time, coast angle and `flatten` flags normalize across L/R repeats and are usable.

## Why speed_test failed here (tried twice, same result)

1. **Spawn geometry defeats the validity filter.**
   The sim drops the robot at a maze entrance with `fl=fr=0.100` (front wall ~10 cm ahead). The acceptance stop for the back-up phase is `front_min >= BACK_TARGET (0.299)`. Backing up opens the front to exactly `0.300`, which the validity layer in `sensing.py` tags **`sat`** (saturation lower bound), and anything above `MAX_RANGE` becomes **`none`** — so the back-up phase never observes a *valid* sample `>= 0.299` and hits timeout every time. The forward sweep then has no valid `[0.15, 0.30]` samples to slope-fit → `n_win: 0, k_lin: null`.

2. **Phase timing is compressed.**
   `run_phase()` advances its local time by the payload `dt` (0.002 s) per loop iteration and only `time.sleep()`s if `dt` is wall-late — but the sim doesn't publish new sensor samples faster. Net effect: an entire 10 s "timeout" phase can complete in under a second of wall time, so the slope-fit has almost no real samples anyway. Each trial in `s2_speed2` completed in ~0.9 s of wall time.

## Log folders in this repo
- `task1b/logs/20261005T120908Z_s1_step/` — step test, **good data**, used for the constants above.
- `task1b/logs/20261005T120939Z_s2_speed/`, `20261005T121001Z_s2_speed2/` — speed test on this box, `k_lin: null` everywhere (see above).

## Next steps
- Run `speed_test.py --label s2_speed` on the MacBook (spawn/timing behave there) and commit the resulting `K_LIN`.
- Until then you can proceed to `--label s1_turns` against the controller with the mesh-guess `K_LIN = 0.017` (or the MacBook value once committed) and let `analyze_run.py` judge turn criteria (±5 deg, zero turn_timeout/no_progress, <=1 reversal, correct sign).
- If this box must be fixed rather than worked around: time-bound `run_phase` on wall clock (`time.monotonic()`) and drive the back-up acceptance through the raw samples (e.g. treat `sat` at MAX_RANGE as "clear enough" only when it represents an actual open direction), then rerun the speed test.
