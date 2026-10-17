# Lattice controller

The controller estimates only a cell, a cardinal heading, position within
that cell, lateral offset, and heading error. Each estimate has an explicit
unknown state where applicable. Side-wall edges and a symmetric front wall
correct commanded-wheel distance; the gyro controls each turn. There is no
continuous x/y pose estimator.

The six states are `GATE`, `CRUISE`, `DECIDE`, `DOCK`, `PIVOT`, and `RECOVER`.
At a cell center, a 0.15 s sensor sample selects left, straight, right, or
U-turn. The side-opening decision never depends on a single splayed front
ray. `DOCK` stops at a projected front range of 0.06 m by default. A closer
0.045 m dock is available only for a supervised validation run. The
normal pivot gate requires fresh readings from both side sensors and a
lateral offset within 8 mm (5 mm for a U-turn). Recovery backs away by
front-range change or bounded commanded distance, then tries an alternate
turn. Recovery's pivot is an escape maneuver and does not claim to be
cell-centered.

The ToF reading `0.300` is a lower bound, not a point distance. It can
indicate open space, but it never enters range differences or an odometry
anchor. The first entrance probe accepts asymmetric front rays because
one ray can still see a gate post after the other sees through the gap.
Both range and commanded travel bound the probe; a close front ray stops it.

The front-route label requires a consistent close pair before calling a wall;
an asymmetric pair with one open ray and no close hazard may be cautiously
followed. A planned pivot requires a recent position anchor and a predicted
cell-center error no larger than 1 cm. Logs distinguish commanded travel from
observed front-range progress and include a heuristic position-uncertainty
budget (not a guaranteed error bound). Unchanged
ToF values alone cannot prove a stall in a uniform straight corridor, so GATE
reports unverified travel rather than treating every flat sensor trace as a
collision. Live simulator pose or wheel feedback is needed to verify actual
translation in that case.

## Run

Install the MQTT client dependency with
`python3 -m pip install -r task1b/requirements.txt`, start `mosquitto`, and
start the official `task_1b_launch` simulator. From this directory run:

```sh
python3 task_1b_boilerplate.py --label lattice_trial_1
```

The runner subscribes to `pacbot/sensors` and `pacbot/result` and publishes
`{"left": ..., "right": ...}` to `pacbot/wheel_vel`. Each run creates
`logs/<UTC>_<label>/` with `meta.json`, sampled `ticks.jsonl`, every state
transition in `events.jsonl`, and `result.json` if the simulator publishes
one. Also capture the simulator's own stdout, since an MQTT result may not
be emitted until completion.

Analyze a completed run with
`python3 analyze_run.py logs/<UTC>_<label>/`.
Run the unit tests with `python3 -m unittest discover -s task1b -v` from
the repository root.

## Validation gates and current limits

1. Record a straight run with at least two distinct side-wall edges.
   Check each edge spacing against a multiple of the 0.22 m cell pitch
   (within roughly 2 cm), and calibrate the commanded-speed estimate if
   real spacing differs. Existing logs do not prove this calibration.
2. Check that corner pivots start within 1.5 cm of cell center at least
   90% of the time, and that median post-turn lateral offset is at most
   2 cm. The default 0.06 m dock is deliberately conservative and sits
   about 1.9 cm short of geometric center; lower it toward 0.045 m only
   after confirming front-ToF accuracy and collision clearance.
3. Check that the robot does not turn in a straight segment, that
   recovery exits, and that a full run produces a simulator verdict.

There is no local simulator in this branch. Unit tests use synthetic
measurements, and a brief entrance check against an older sensor log is
only a classification check: recorded sensors cannot react to this
controller's different commands. The geometry, front range at 4 cm,
slip-adjusted linear gain, and doorway behavior still need a live run.
