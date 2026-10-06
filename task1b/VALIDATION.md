# Task 1B validation status

The controller now treats 0.300 m as a measured range, classifies fresh side
readings with persistence, follows the left wall, advances into a confirmed
left opening before turning, and records aborted turns after emergency backout.
The 15-second unchanged-ToF route trigger and heading-error pivot are removed.
`commanded_travel_est_m` in `ticks.csv` is an estimate, not measured motion.

## Local checks

```sh
python3 -m unittest discover -s task1b -p 'test_*.py'
python3 task1b/decision_replay.py task1b/logs/*maze17/ticks.csv \
  task1b/logs/*maze18/ticks.csv task1b/logs/*maze19/ticks.csv
```

Replay stops at the first changed decision. The subsequent old sensor stream is
not a valid trajectory for the new controller. The grid/ray test uses the
sensor positions and ±20° front rays in `SIM_NOTES.md`; it verifies corridor
tracking and a completed left turn under a small plant model, not a maze solve.

## Linux simulator gate

Run a short straight corridor and single junction trial first. Then run ten
independent random 6×6 mazes, each to a simulator verdict or ten minutes.
Start each run from a committed controller, preserve its `meta.json`,
`ticks.csv`, `events.csv`, `result.json`, simulator stdout, and controller
stdout. Analyze each folder with `analyze_run.py` and review collisions.
A `pending` verdict or timeout fails the set. Accept only ten solved verdicts,
no repeated backout loop, and no completed turn after emergency interruption.
If any run fails, preserve its logs, fix the observed failure, and restart the
whole ten-run set.

The bundled `task_1b_launch` is a Linux x86-64 ELF executable. This workspace
is macOS and its Docker daemon is unavailable, so no live simulator verdict is
claimed here.
