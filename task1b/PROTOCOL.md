# PacBot Task 1B — Agent Handoff Protocol

Two agents, two machines, one repo. The Mac agent (this file's author)
writes code, analysis, and task cards. The Linux agent (owns the only
box that runs the sim) runs procedures and reports back data.
**This file is the communication channel.** Both sides append to the
Handoff Log at the bottom; never rewrite history, commit + push after
every edit so the other side sees it.

## Roles

- **Mac**: controller code, tooling (`analyze_run.py`, `replay.py`,
  `tune_local.py`, `step_test.py`, `speed_test.py`), offline
  verification, verdicts. Cannot run the sim.
- **Linux**: runs sim procedures exactly as specified, commits raw
  logs + sim output, reports observations. Does not need to debug;
  just record faithfully (unexpected behavior is data, not failure).

## Conventions

- One run = one label = one folder `task1b/logs/<UTC>_<label>/`.
- One commit per change group; **never bundle code changes into log
  commits** (keeps later bisection possible).
- Every run records through `RunLogger` (automatic). Always also
  capture the sim's own stdout — it prints solve state, collisions,
  time, and score, none of which reaches MQTT.
- Constants live in `task_1b_boilerplate.py` and are snapshotted into
  every run's `meta.json`. Two runs are comparable only via meta.

## Current state (2026-10-05, controller @ `f6d2a63` + brake work)

Validated: 3/3 turns within ±5° (TURN→BRAKE→FOLLOW), stuck clears
~1.2 s, 1.4 m wall-follow tracked, yaw gain 0.0914 measured,
`K_LIN=0.017` geometric (binary MJCF; sim slope fits are
rotation-contaminated — see `SIM_NOTES.md`).
Provisional: timeout 4.43 s (matches `1.5×t90+0.5` ≈ 4.7 s, keep),
`SAT_MODE=ceiling`, cruise 0.04 m/s, `KP_HEADING=1.0` (start low).

## ACTIVE TASK — maze1: first full maze run

**Goal:** traverse a full random maze; exercise wedge/stuck paths
that short routes never reach.

**Terminals (Linux):**
```
# 1. broker
mosquitto
# 2. sim, capturing its score line (its stdout never reaches MQTT)
./task_1b_launch 2>&1 | tee sim_maze1.log
# 3. controller (pull first: git pull mine main)
python3 task_1b_boilerplate.py --label maze1
```

**End condition:** sim prints `MAZE SOLVED` (note time/score/
collisions from its terminal), or abort after ~10 min wall.
Then Ctrl+C the controller.

**Commit:** `task1b/logs/<run>/` + `sim_maze1.log` in ONE commit
(`Add maze1 run logs`), push. No code changes in that commit.

**Report back** (append to Handoff Log below):
1. Sim verdict line (solved? time? score? collisions?)
2. Anything surprising observed (pins, spins, wall contact, weird loops)
3. `python3 analyze_run.py task1b/logs/<run>/` output pasted verbatim

**Pass criteria (all printed by `analyze_run.py`, none judged by eye):**
- Every turn final err within ±5° (gyro integral, 0.3 s after exit)
- Zero `turn_timeout` / `no_progress` / `overshoot`, zero TURN|BRAKE→BACKUP
- ≤1 reversal per turn, within 8° of target only
- Zero stuck episodes over 1 s; any firing clears within 4 s
- No wedge cycle shorter than 0.5 s
- No wall-contact signature (sl+sr both pinned <0.06 while driving)

## Backlog queue (ordered, one per run)

1. `maze1` (this task).
2. `maze2`+ — whatever `maze1` breaks writes the next work item.
3. `s2_speed_v5` — gyro-gated BACK fit rerun (expect `k_lin≈0.017`
   r²>0.9 or honest nulls). Only if a straight translation exists;
   spawn slot looks wedged, do not force it.
4. s2c cruise steps (0.08, 0.12) — BLOCKED on scoring info. Coast
   gate already passes on paper (0.15 ≥ 0.019+0.05). Do not run
   until scoring is known to reward speed.
5. `KP_HEADING` tuning up from 1.0 — needs long-run heading data first.

## Open questions (need human/spec, not runs)

- ToF spec: is 0.300 a saturation ceiling? (`SAT_MODE` stays a switch.)
- Official scoring: what is rewarded (time? coverage? collisions?)?
- Task 1A maze planning: untouched; separate project after 1B is solid.

## Handoff Log (append-only, newest at bottom)

- 2026-10-05 Mac→Linux: PROTOCOL.md created. `maze1` tasked as
  above (controller `f6d2a63`). s1 (3/3 turns) and s2 analyses are
  in prior commit messages; `SIM_NOTES.md` holds binary-extracted
  sim facts. Waiting on maze1 logs + sim verdict line.
