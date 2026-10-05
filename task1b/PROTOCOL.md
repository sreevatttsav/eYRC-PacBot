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
# 2. sim, force stdout/stderr into a PTY and unbuffered log
script -q -f -c 'stdbuf -o0 -e0 ./task_1b_launch' sim_maze1.log
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
- 2026-10-05 Mac verdict on `maze1` (251k ticks, 600 s): turns 2/2
  PASS, stuck 2x cleared ~1.1 s each, BUT the run failed at the
  strategy level. Trace: turn#0 left at spawn (t=0.5) → drove 1.0 m
  north hugging the west perimeter wall (sr numeric 88%, mean
  0.093) with the entrance gap behind/beside it → NW corner at
  (-0.24,+1.0) → tie-break left again (west) → 9 min blind cruise
  20 m out (all sat). Sustained sr openings at t≈23-28 (up to
  ~2.3 s, the entrance or perimeter gaps) were driven straight
  past: the controller turns AWAY from blockages but never INTO
  openings. Sim stdout empty (never solved; nothing to report).
  Proposed fix pack: (1) wall-loss gap-seek (lose follow wall with
  front clear → 90° into it), (2) tie-break toward last-seen wall,
  (3) anti-void (blind >15 s → 180° turn-back, then hold).
  Awaiting go-ahead to implement.
- 2026-10-05 Mac→Linux: fix pack IMPLEMENTED and pushed (gap-seek +
  wall-memory tie-break + anti-void/HOLD, new `_start_turn` funnel,
  `turn_cause`-tagged reasons). Verified offline: gap triggers after
  1 s follow + 0.5 s opening and completes; corridor junctions and
  brief follows do not trigger; virgin ties keep prev-dir, informed
  ties go to the most-recent wall; blind 15 s fires a 180° lost
  turn, second consecutive stretch latches HOLD zeros. Full
  regression green (harness, 5 turn-logic tests, both replay gates,
  old-log analyzes). Also fixed: `_start_turn` now resets
  `spin_done_s` (else the clear-front gate eats the second armed
  turn of a run). Next: `maze2` full run against the same card.
- 2026-10-05 Linux→Mac: `maze1` ran to controller completion/timeout
  (251,400 ticks; `task1b/logs/20261005T131606Z_maze1/`). Controller was
  at `c8019a3`, with `K_LIN=0.017`, `YAW_GAIN_K=0.0914`, `SAT_MODE=ceiling`.
  The captured sim stdout file is **empty**, so there is no sim `MAZE
  SOLVED` line, time, score, or collision count to report. What I noticed:
  the controller did both commanded +90° turns cleanly, both brief stuck
  events cleared in about one second, and after the second turn at
  wall-clock ~45 s it remained in FOLLOW. Front and side sensors stayed
  overwhelmingly at the 0.300/saturated boundary, so this run demonstrates
  state-machine recovery, not verified forward maze progress. The verbatim
  `analyze_run.py` report is below.
  ```
  == maze1 commit=c8019a3 ticks=251400
  -- turns --
    #0 t=0.5s target=+90deg(logged) final_err=-1.8deg dur=3.60s exit=FOLLOW:clear rev=0[] PASS
    #1 t=41.4s target=+90deg(logged) final_err=-0.8deg dur=3.60s exit=FOLLOW:clear rev=0[] PASS
  -- stuck --
    t=18.7s @RECOVER hold=1.17s
    t=35.2s @RECOVER hold=1.12s
  -- transitions --
    BRAKE->FOLLOW [clear] x2
    FOLLOW->RECOVER [stuck] x2
    FOLLOW->REVERSE [blocked] x1
    RECOVER->FOLLOW [clear] x2
    REVERSE->TURN [left] x2
    TURN->BRAKE [coast] x2
  -- sensors (valid/sat/held/none) --
    fl: valid=0.3% sat=93.1% held=0.0% none=6.6%
    fr: valid=5.8% sat=87.0% held=0.0% none=7.2%
    sl: valid=1.6% sat=98.3% held=0.1% none=0.0%
    sr: valid=6.3% sat=93.6% held=0.1% none=0.0%
  -- mean |wheel| by state --
    BRAKE: 2.00 rad/s
    FOLLOW: 2.35 rad/s
    RECOVER: 3.00 rad/s
    REVERSE: 3.00 rad/s
    TURN: 1.87 rad/s
  -- lateral: mean|offset|=0.009005999999999998 coverage=0.2% n=475
  ```
- 2026-10-05 Linux→Mac: `maze2` stopped early on user instruction
  (~134 s wall, 54,997 ticks; `task1b/logs/20261005T133931Z_maze2/`,
  controller `77c99d1`). Sim stdout is **empty** again, so no `MAZE
  SOLVED` line, time, score, or collisions. What I noticed: 4/5 turns
  PASS, but the `lost_left` gap-seek turn at t=70.1 s FAILs with
  +86.8° error against its inferred +90° target — the new gap-seek
  path fires but does not complete the intended turn. Both stuck
  episodes cleared in ~1.15 s; one later stuck escalated
  RECOVER→BACKUP and then cleared. Side-sensor validity is much
  better than maze1 (sl/sr valid ~36-38% vs ~2-6%). Verbatim
  `analyze_run.py` below.
  ```
  == maze2 commit=77c99d1 ticks=54997
  -- turns --
    #0 t=0.4s target=+90deg(logged) final_err=-2.7deg dur=3.24s exit=FOLLOW:clear rev=0[] PASS
    #1 t=5.8s target=+90deg(logged) final_err=-2.2deg dur=3.37s exit=FOLLOW:clear rev=0[] PASS
    #2 t=70.1s target=+90deg(inferred-90deg) final_err=+86.8deg dur=5.14s exit=FOLLOW:clear rev=0[] FAIL
    #3 t=115.0s target=-90deg(logged) final_err=+2.9deg dur=3.28s exit=FOLLOW:clear rev=0[] PASS
    #4 t=130.4s target=-90deg(logged) final_err=+4.8deg dur=3.42s exit=FOLLOW:clear rev=0[] PASS
  -- stuck --
    t=119.9s @RECOVER hold=1.16s
    t=123.3s @RECOVER hold=1.15s
  -- transitions --
    BACKUP->FOLLOW [clear] x1
    BRAKE->FOLLOW [clear] x5
    FOLLOW->RECOVER [stuck] x2
    FOLLOW->REVERSE [blocked] x3
    FOLLOW->TURN [lost_left] x1
    RECOVER->BACKUP [stuck_escalate] x1
    RECOVER->FOLLOW [clear] x1
    REVERSE->TURN [left] x2
    REVERSE->TURN [right] x2
    TURN->BRAKE [coast] x5
  -- sensors (valid/sat/held/none) --
    fl: valid=20.4% sat=69.8% held=0.3% none=9.5%
    fr: valid=6.0% sat=79.7% held=0.0% none=14.3%
    sl: valid=37.8% sat=57.1% held=1.4% none=3.7%
    sr: valid=36.1% sat=58.2% held=2.5% none=3.1%
  -- mean |wheel| by state --
    BACKUP: 3.00 rad/s
    BRAKE: 2.00 rad/s
    FOLLOW: 2.36 rad/s
    RECOVER: 3.00 rad/s
    REVERSE: 3.00 rad/s
    TURN: 1.96 rad/s
  -- lateral: mean|offset|=0.042101814814814814 coverage=2.5% n=1350
  ```
- 2026-10-05 Linux→Mac: `maze3` stopped on user instruction (~112 s
  wall, 53,499 ticks; `task1b/logs/20261005T140519Z_maze3/`, controller
  `e40e8c5`). Sim stdout is **empty** again, so no `MAZE SOLVED` line,
  time, score, or collisions. What I noticed: **all 6 turns PASS**
  (including two +180° lost_left turns and one gap_left turn), 3 stuck
  episodes all cleared in ~1.1-1.2 s, and the new gateway probe
  (`gap_left` at t=51.4 s) completed cleanly. Sensor validity improved
  further (fl/fr valid ~30-38%). The spawn behavior is unchanged:
  REVERSE→TURN left at t=0.44 s, then FOLLOW. Verbatim
  `analyze_run.py` below.
  ```
  == maze3 commit=e40e8d5 ticks=53499
  -- turns --
    #0 t=0.4s target=+90deg(logged) final_err=-2.3deg dur=3.63s exit=FOLLOW:clear rev=0[] PASS
    #1 t=6.5s target=+90deg(logged) final_err=-3.7deg dur=3.65s exit=FOLLOW:clear rev=0[] PASS
    #2 t=29.3s target=+180deg(logged) final_err=-3.6deg dur=5.21s exit=FOLLOW:clear rev=0[] PASS
    #3 t=51.4s target=+90deg(logged) final_err=-0.4deg dur=3.32s exit=FOLLOW:approach rev=0[] PASS
    #4 t=71.5s target=+180deg(logged) final_err=-0.6deg dur=5.62s exit=FOLLOW:clear rev=0[] PASS
    #5 t=106.5s target=+180deg(logged) final_err=-3.3deg dur=5.38s exit=FOLLOW:clear rev=0[] PASS
  -- stuck --
    t=36.2s @RECOVER hold=1.22s
    t=78.4s @RECOVER hold=1.11s
    t=83.3s @RECOVER hold=1.22s
  -- transitions --
    BACKUP->FOLLOW [clear] x1
    BRAKE->FOLLOW [approach] x1
    BRAKE->FOLLOW [clear] x5
    FOLLOW->RECOVER [stuck] x3
    FOLLOW->REVERSE [blocked] x1
    FOLLOW->TURN [gap_left] x1
    FOLLOW->TURN [lost_left] x3
    RECOVER->BACKUP [stuck_escalate] x1
    RECOVER->FOLLOW [clear] x2
    REVERSE->TURN [left] x2
    TURN->BRAKE [coast] x6
  -- sensors (valid/sat/held/none) --
    fl: valid=30.0% sat=57.8% held=0.1% none=12.1%
    fr: valid=37.8% sat=54.6% held=0.0% none=7.7%
    sl: valid=28.6% sat=71.2% held=0.0% none=0.2%
    sr: valid=24.2% sat=75.8% held=0.0% none=0.0%
  -- mean |wheel| by state --
    BACKUP: 3.00 rad/s
    BRAKE: 2.00 rad/s
    FOLLOW: 2.33 rad/s
    RECOVER: 3.00 rad/s
    REVERSE: 3.00 rad/s
    TURN: 2.09 rad/s
  -- lateral: mean|offset|=0.025216516620498616 coverage=2.7% n=1444
  ```
- 2026-10-05 Linux→Mac: `maze4` stopped on user instruction (~11 s
  wall, 16,100 ticks; `task1b/logs/20261005T142136Z_maze4/`, controller
  `a3596c6`). Sim stdout is **empty** again. What I noticed: the new
  GATEWAY probe **fired at spawn** (t=0.8 s, `GATEWAY` state at 1.18
  rad/s creep) — spawn recognition works — but it exited via
  `GATEWAY→REVERSE [blocked]` after only ~0.5 s, i.e. the creep met
  the frame/post and fell through to normal escape (REVERSE→TURN
  left). Both turns PASS, zero stuck episodes, lateral coverage 11.1%
  (best yet). Verbatim `analyze_run.py` below.
  ```
  == maze4 commit=a3596c6 ticks=16100
  -- turns --
    #0 t=1.3s target=+90deg(logged) final_err=+0.3deg dur=3.51s exit=FOLLOW:clear rev=0[] PASS
    #1 t=7.1s target=+90deg(logged) final_err=-0.6deg dur=3.74s exit=FOLLOW:clear rev=0[] PASS
  -- stuck --
    zero episodes
  -- transitions --
    BRAKE->FOLLOW [clear] x2
    FOLLOW->REVERSE [blocked] x1
    GATEWAY->REVERSE [blocked] x1
    REVERSE->TURN [left] x2
    TURN->BRAKE [coast] x2
  -- sensors (valid/sat/held/none) --
    fl: valid=6.9% sat=78.4% held=0.0% none=14.7%
    fr: valid=25.7% sat=71.5% held=0.0% none=2.8%
    sl: valid=12.2% sat=86.4% held=0.3% none=1.1%
    sr: valid=19.6% sat=80.2% held=0.3% none=0.0%
  -- mean |wheel| by state --
    BRAKE: 2.00 rad/s
    FOLLOW: 2.32 rad/s
    GATEWAY: 1.18 rad/s
    RECOVER: 3.00 rad/s
    REVERSE: 3.00 rad/s
    TURN: 1.86 rad/s
  -- lateral: mean|offset|=0.020438129899216124 coverage=11.1% n=1786
  ```
- 2026-10-05 Linux→Mac: `maze5` stopped on user instruction (~64 s
  wall, 31,800 ticks; `task1b/logs/20261005T143929Z_maze5/`, controller
  `6e348be`). Sim stdout is **empty** again. What I noticed: GATEWAY
  fired at spawn and crept at 1.18 rad/s, but at t=9.36 s it hit
  `gateway_timeout` → `GATEWAY_HOLD` (zeros) and sat still for the
  rest of the run — the latch held but the bot never crossed the
  entrance. All sensors 100% valid in GATEWAY state, lateral offset
  ~0 (perfectly centered). No turns, no stuck episodes. Verbatim
  `analyze_run.py` below.
  ```
  == maze5 commit=6e348be ticks=31800
  -- turns --
    (none)
  -- stuck --
    zero episodes
  -- transitions --
    GATEWAY->GATEWAY_HOLD [gateway_timeout] x1
  -- sensors (valid/sat/held/none) --
    fl: valid=100.0% sat=0.0% held=0.0% none=0.0%
    fr: valid=100.0% sat=0.0% held=0.0% none=0.0%
    sl: valid=100.0% sat=0.0% held=0.0% none=0.0%
    sr: valid=100.0% sat=0.0% held=0.0% none=0.0%
  -- mean |wheel| by state --
    GATEWAY: 1.18 rad/s
    GATEWAY_HOLD: 0.00 rad/s
  -- lateral: mean|offset|=1.2578616352201618e-08 coverage=100.0% n=31800
  ```
- 2026-10-05 Linux→Mac: `maze6` stopped on user instruction (~83 s
  wall, 41,550 ticks; `task1b/logs/20261005T144949Z_maze6/`, controller
  `138e4ee`). Sim stdout is **empty** again. What I noticed: identical
  to maze5 — GATEWAY fired at spawn, crept at 1.18 rad/s, hit
  `gateway_timeout` at t=9.6 s → `GATEWAY_HOLD` (zeros), sat still
  for the rest of the run. The "steer and hand off at gateway corridor"
  fix did not change the outcome: the bot still never crosses the
  entrance. All sensors 100% valid in GATEWAY state. No turns, no
  stuck episodes. Verbatim `analyze_run.py` below.
  ```
  == maze6 commit=138e4ee ticks=41550
  -- turns --
    (none)
  -- stuck --
    zero episodes
  -- transitions --
    GATEWAY->GATEWAY_HOLD [gateway_timeout] x1
  -- sensors (valid/sat/held/none) --
    fl: valid=100.0% sat=0.0% held=0.0% none=0.0%
    fr: valid=100.0% sat=0.0% held=0.0% none=0.0%
    sl: valid=100.0% sat=0.0% held=0.0% none=0.0%
    sr: valid=100.0% sat=0.0% held=0.0% none=0.0%
  -- mean |wheel| by state --
    GATEWAY: 1.18 rad/s
    GATEWAY_HOLD: 0.00 rad/s
  -- lateral: mean|offset|=None coverage=0.0% n=0
  ```
- 2026-10-05 Linux→Mac: `maze7` stopped on user instruction (~39 s
  wall, 19,300 ticks; `task1b/logs/20261005T150451Z_maze7/`, controller
  `8a77d68`). Sim stdout is **empty** again. What I noticed: the
  "advance one cell" fix worked — GATEWAY fired at spawn and handed
  off to FOLLOW at t=2.0 s (no more GATEWAY_HOLD). But then the bot
  fell into a **wedge cycle**: 8× `FOLLOW→WEDGE→FOLLOW` from t=8.7 s
  to t=44 s, never escaping. All sensors 100% valid, lateral offset
  0.012 m. No turns, no stuck episodes. Verbatim `analyze_run.py`
  below.
  ```
  == maze7 commit=8a77d68 ticks=19300
  -- turns --
    (none)
  -- stuck --
    zero episodes
  -- transitions --
    FOLLOW->WEDGE [wedge_enter] x8
    GATEWAY->FOLLOW [approach] x1
    WEDGE->FOLLOW [approach] x7
  -- sensors (valid/sat/held/none) --
    fl: valid=100.0% sat=0.0% held=0.0% none=0.0%
    fr: valid=100.0% sat=0.0% held=0.0% none=0.0%
    sl: valid=100.0% sat=0.0% held=0.0% none=0.0%
    sr: valid=100.0% sat=0.0% held=0.0% none=0.0%
  -- mean |wheel| by state --
    FOLLOW: 2.35 rad/s
    GATEWAY: 2.35 rad/s
    WEDGE: 3.00 rad/s
  -- lateral: mean|offset|=0.012178204663212436 coverage=100.0% n=19300
  ```
- 2026-10-05 Linux→Mac: `maze8` stopped on user instruction (~55 s
  wall, 26,350 ticks; `task1b/logs/20261005T151254Z_maze8/`, controller
  `4070951`). Sim stdout is **empty** again. What I noticed: GATEWAY
  handed off to FOLLOW at t=2.3 s (good), but then 4 wedge cycles,
  and **all 5 turns FAIL** (errors −5.7° to −68.8°, two exiting via
  RECOVER:stuck). The "prioritize ToF steering" fix made turns worse,
  not better. Sensor validity excellent (83–95% valid), lateral
  coverage 72.6% (best yet). Two stuck episodes cleared. Verbatim
  `analyze_run.py` below.
  ```
  == maze8 commit=4070951 ticks=26350
  -- turns --
    #0 t=21.8s target=+90deg(logged) final_err=-6.5deg dur=4.34s exit=RECOVER:stuck rev=0[] FAIL
    #1 t=28.7s target=-90deg(logged) final_err=-25.7deg dur=5.12s exit=RECOVER:stuck rev=0[] FAIL
    #2 t=36.5s target=+90deg(logged) final_err=-5.7deg dur=3.21s exit=FOLLOW:approach rev=0[] FAIL
    #3 t=42.3s target=-90deg(logged) final_err=-68.8deg dur=7.20s exit=FOLLOW:approach rev=0[] FAIL
    #4 t=54.7s target=-180deg(logged) final_err=+12.5deg dur=5.21s exit=END:(no-event-row) rev=0[] FAIL
  -- stuck --
    t=26.1s @RECOVER hold=1.11s
    t=33.8s @RECOVER hold=1.12s
  -- transitions --
    BRAKE->FOLLOW [approach] x2
    BRAKE->TURN [right] x2
    FOLLOW->REVERSE [blocked] x2
    FOLLOW->WEDGE [wedge_enter] x4
    GATEWAY->FOLLOW [approach] x1
    RECOVER->REVERSE [blocked] x2
    REVERSE->TURN [left] x1
    REVERSE->TURN [right] x3
    TURN->BRAKE [coast] x4
    TURN->RECOVER [stuck] x2
    WEDGE->FOLLOW [approach] x3
    WEDGE->TURN [wedge_left] x1
  -- sensors (valid/sat/held/none) --
    fl: valid=83.4% sat=13.5% held=0.2% none=2.9%
    fr: valid=87.2% sat=9.3% held=0.2% none=3.4%
    sl: valid=93.9% sat=5.9% held=0.0% none=0.2%
    sr: valid=94.9% sat=5.1% held=0.0% none=0.0%
  -- mean |wheel| by state --
    BRAKE: 2.00 rad/s
    FOLLOW: 2.02 rad/s
    GATEWAY: 2.35 rad/s
    RECOVER: 3.00 rad/s
    REVERSE: 3.00 rad/s
    TURN: 1.97 rad/s
    WEDGE: 3.00 rad/s
  -- lateral: mean|offset|=0.01214166231187291 coverage=72.6% n=19136
  ```
- 2026-10-05 Linux→Mac: `maze9` stopped on user instruction (~39 s
  wall, 19,249 ticks; `task1b/logs/20261005T152444Z_maze9/`, controller
  `5abb2b7`). Sim stdout is **empty** again. What I noticed: GATEWAY
  fired but exited via `GATEWAY→REVERSE [blocked]` at t=6.8 s (no
  handoff to FOLLOW this time). Then a long sequence of failed turns:
  **4/5 turns FAIL** (errors +21.5°, −15.0°, −95.2°, +96.6°), with
  `TURN→BACKUP [no_progress]` ×2 and `TURN→BACKUP [turn_timeout]` ×1.
  The "fix turn recovery" commit made turns worse. Sensor validity
  good (77–88% valid), lateral coverage 60%. Zero stuck episodes.
  Verbatim `analyze_run.py` below.
  ```
  == maze9 commit=5abb2b7 ticks=19249
  -- turns --
    #0 t=7.3s target=+90deg(logged) final_err=-3.8deg dur=3.59s exit=FOLLOW:clear rev=0[] PASS
    #1 t=16.5s target=+90deg(logged) final_err=+21.5deg dur=6.11s exit=BACKUP:no_progress rev=0[] FAIL
    #2 t=23.8s target=+90deg(logged) final_err=-15.0deg dur=5.45s exit=BACKUP:turn_timeout rev=0[] FAIL
    #3 t=34.3s target=+180deg(logged) final_err=-95.2deg dur=3.40s exit=BACKUP:no_progress rev=0[] FAIL
    #4 t=38.7s target=+90deg(logged) final_err=+96.6deg dur=7.03s exit=END:(no-event-row) rev=0[] FAIL
  -- stuck --
    zero episodes
  -- transitions --
    BACKUP->FOLLOW [clear] x1
    BACKUP->REVERSE [blocked] x2
    BACKUP->WEDGE [wedge_enter] x1
    BRAKE->FOLLOW [clear] x1
    BRAKE->TURN [left] x2
    BRAKE->TURN [wedge_left] x1
    FOLLOW->REVERSE [blocked] x1
    FOLLOW->WEDGE [wedge_enter] x2
    GATEWAY->REVERSE [blocked] x1
    REVERSE->TURN [left] x4
    TURN->BACKUP [no_progress] x2
    TURN->BACKUP [turn_timeout] x1
    TURN->BRAKE [coast] x4
    WEDGE->BACKUP [giveup] x1
    WEDGE->FOLLOW [clear] x1
    WEDGE->TURN [wedge_left] x1
  -- sensors (valid/sat/held/none) --
    fl: valid=77.3% sat=14.4% held=0.5% none=7.8%
    fr: valid=82.5% sat=15.6% held=0.2% none=1.6%
    sl: valid=87.6% sat=12.4% held=0.0% none=0.0%
    sr: valid=84.1% sat=14.1% held=0.0% none=1.8%
  -- mean |wheel| by state --
    BACKUP: 3.00 rad/s
    BRAKE: 2.00 rad/s
    FOLLOW: 2.15 rad/s
    GATEWAY: 2.35 rad/s
    REVERSE: 3.00 rad/s
    TURN: 2.13 rad/s
    WEDGE: 3.00 rad/s
  -- lateral: mean|offset|=0.023056939994804744 coverage=60.0% n=11549
  ```
- 2026-10-05 Linux→Mac: `maze10` stopped on user instruction (~17 s
  wall, 8,400 ticks; `task1b/logs/20261005T153053Z_maze10/`, controller
  `78ec850`). Sim stdout is **empty** even with `stdbuf -oL` — the sim
  binary appears to fully buffer or only print at exit. What I noticed:
  GATEWAY fired but exited via `GATEWAY→REVERSE [blocked]` at t=6.7 s.
  1/2 turns PASS (the second truncated by the stop). 2 wedge cycles.
  Sensor validity good (67–99% valid). Verbatim `analyze_run.py` below.
  ```
  == maze10 commit=78ec850 ticks=8400
  -- turns --
    #0 t=7.2s target=+90deg(logged) final_err=-3.7deg dur=3.56s exit=FOLLOW:clear rev=0[] PASS
    #1 t=15.6s target=+90deg(logged) final_err=+15.0deg dur=4.58s exit=END:(no-event-row) rev=0[] FAIL
  -- stuck --
    zero episodes
  -- transitions --
    BRAKE->FOLLOW [clear] x1
    BRAKE->TURN [wedge_left] x1
    FOLLOW->WEDGE [wedge_enter] x2
    GATEWAY->REVERSE [blocked] x1
    REVERSE->TURN [left] x1
    TURN->BRAKE [coast] x2
    WEDGE->FOLLOW [clear] x1
    WEDGE->TURN [wedge_left] x1
  -- sensors (valid/sat/held/none) --
    fl: valid=67.4% sat=26.2% held=0.6% none=5.8%
    fr: valid=99.9% sat=0.1% held=0.0% none=0.0%
    sl: valid=87.3% sat=12.7% held=0.0% none=0.0%
    sr: valid=87.7% sat=12.1% held=0.0% none=0.2%
  -- mean |wheel| by state --
    BRAKE: 2.00 rad/s
    FOLLOW: 2.35 rad/s
    GATEWAY: 2.35 rad/s
    REVERSE: 3.00 rad/s
    TURN: 1.98 rad/s
    WEDGE: 3.00 rad/s
  -- lateral: mean|offset|=0.012544009648887697 coverage=44.4% n=3731
  ```
- 2026-10-05 Linux→Mac: `maze11` stopped on user instruction (~33 s
  wall, 16,600 ticks; `task1b/logs/20261005T153431Z_maze11/`, controller
  `f11e7ee`). Sim stdout is **empty** even with `stdbuf -oL`. What I
  noticed: **5/6 turns PASS** (including four +180° turns), a big
  improvement over maze9's 1/5. The "prevent corridor wedge triggers
  and turn repeats" fix worked. GATEWAY fired but exited via
  `GATEWAY→REVERSE [blocked]` at t=6.6 s. One wedge cycle. Sensor
  validity excellent (81–91% valid), lateral coverage 56.9%. Verbatim
  `analyze_run.py` below.
  ```
  == maze11 commit=f11e7ee ticks=16600
  -- turns --
    #0 t=7.1s target=+90deg(logged) final_err=-4.4deg dur=3.50s exit=FOLLOW:clear rev=0[] PASS
    #1 t=16.0s target=+180deg(logged) final_err=-2.5deg dur=5.24s exit=FOLLOW:approach rev=0[] PASS
    #2 t=21.7s target=+180deg(logged) final_err=-0.6deg dur=4.85s exit=FOLLOW:approach rev=0[] PASS
    #3 t=27.0s target=+180deg(logged) final_err=-3.0deg dur=4.85s exit=FOLLOW:approach rev=0[] PASS
    #4 t=32.3s target=+180deg(logged) final_err=-2.3deg dur=5.18s exit=FOLLOW:approach rev=0[] PASS
    #5 t=38.0s target=+180deg(logged) final_err=-138.3deg dur=0.80s exit=END:(no-event-row) rev=0[] FAIL
  -- stuck --
    zero episodes
  -- transitions --
    BRAKE->FOLLOW [approach] x4
    BRAKE->FOLLOW [clear] x1
    FOLLOW->REVERSE [blocked] x5
    FOLLOW->WEDGE [wedge_enter] x1
    GATEWAY->REVERSE [blocked] x1
    REVERSE->TURN [left] x6
    TURN->BRAKE [coast] x5
    WEDGE->FOLLOW [clear] x1
  -- sensors (valid/sat/held/none) --
    fl: valid=81.1% sat=11.0% held=0.7% none=7.2%
    fr: valid=86.5% sat=9.7% held=0.3% none=3.5%
    sl: valid=91.0% sat=9.0% held=0.0% none=0.0%
    sr: valid=87.7% sat=12.3% held=0.0% none=0.0%
  -- mean |wheel| by state --
    BRAKE: 2.00 rad/s
    FOLLOW: 2.02 rad/s
    GATEWAY: 2.35 rad/s
    REVERSE: 3.00 rad/s
    TURN: 2.22 rad/s
    WEDGE: 3.00 rad/s
  -- lateral: mean|offset|=0.019240486772486774 coverage=56.9% n=9450
  ```
- 2026-10-05 Linux→Mac: `maze12` stopped on user instruction (~36 s
  wall, 17,800 ticks; `task1b/logs/20261005T154132Z_maze12/`, controller
  `54b9203`). Sim stdout is **empty** even with `stdbuf -oL`. What I
  noticed: GATEWAY handed off to FOLLOW at t=1.76 s (good), then FOLLOW
  held for the entire run — no turns, no stuck, no wedge. All sensors
  100% valid, lateral offset 0.003 m (excellent centering). The "require
  both front rays" fix eliminated false escape triggers, but the bot
  never turned — it likely drove straight and sat in FOLLOW. Verbatim
  `analyze_run.py` below.
  ```
  == maze12 commit=54b9203 ticks=17800
  -- turns --
    (none)
  -- stuck --
    zero episodes
  -- transitions --
    GATEWAY->FOLLOW [approach] x1
  -- sensors (valid/sat/held/none) --
    fl: valid=100.0% sat=0.0% held=0.0% none=0.0%
    fr: valid=100.0% sat=0.0% held=0.0% none=0.0%
    sl: valid=100.0% sat=0.0% held=0.0% none=0.0%
    sr: valid=100.0% sat=0.0% held=0.0% none=0.0%
  -- mean |wheel| by state --
    FOLLOW: 2.33 rad/s
    GATEWAY: 2.35 rad/s
  -- lateral: mean|offset|=0.0029028848314606743 coverage=100.0% n=17800
  ```
- 2026-10-05 Linux→Mac: `maze13` stopped on user instruction (~64 s
  wall, 31,800 ticks; `task1b/logs/20261005T155044Z_maze13/`, controller
  `84372a2`). Sim stdout is **empty** even with `stdbuf -oL`. What I
  noticed: GATEWAY handed off to FOLLOW at t=2.16 s, then a `gap_left`
  turn at t=29.4 s completed PASS (−3.6°). The "restore front safety"
  fix brought back a turn. FOLLOW mean wheel speed dropped to 0.60
  rad/s (very slow — likely creeping near a wall). Sensor validity mixed
  (fl 49% valid / 43% none, fr/sl/sr 99–100% valid). Lateral coverage
  75.6%. Verbatim `analyze_run.py` below.
  ```
  == maze13 commit=84372a2 ticks=31800
  -- turns --
    #0 t=29.4s target=+90deg(logged) final_err=-3.6deg dur=3.97s exit=FOLLOW:approach rev=0[] PASS
  -- stuck --
    zero episodes
  -- transitions --
    BRAKE->FOLLOW [approach] x1
    FOLLOW->TURN [gap_left] x1
    GATEWAY->FOLLOW [approach] x1
    TURN->BRAKE [coast] x1
  -- sensors (valid/sat/held/none) --
    fl: valid=49.2% sat=7.3% held=0.3% none=43.2%
    fr: valid=99.8% sat=0.2% held=0.0% none=0.0%
    sl: valid=100.0% sat=0.0% held=0.0% none=0.0%
    sr: valid=98.8% sat=1.2% held=0.0% none=0.0%
  -- mean |wheel| by state --
    BRAKE: 2.00 rad/s
    FOLLOW: 0.60 rad/s
    GATEWAY: 2.35 rad/s
    TURN: 2.04 rad/s
  -- lateral: mean|offset|=0.009430193900054092 coverage=75.6% n=24033
  ```
- 2026-10-05 Linux→Mac: `maze14` stopped on user instruction (~50 s
  wall, 24,949 ticks; `task1b/logs/20261005T160551Z_maze14/`, controller
  `5f698e3`). Sim stdout captured via `script -qec` — the wrapper
  lines appear (`Script started/done`) but the sim binary itself
  produces **no stdout during the run**; it only prints at exit
  (MAZE SOLVED/score), which is lost on SIGTERM. Controller output
  captured in `controller_maze14.log`. What I noticed: GATEWAY fired
  but exited via `GATEWAY→REVERSE [blocked]` at t=6.8 s. One turn PASS
  (−1.8°). FOLLOW held the rest. Sensor validity mixed (fl 17% valid /
  76% sat, fr/sl/sr 100% valid). Lateral coverage 86.6% (best yet).
  Verbatim `analyze_run.py` below.
  ```
  == maze14 commit=5f698e3 ticks=24949
  -- turns --
    #0 t=7.2s target=+90deg(logged) final_err=-1.8deg dur=3.54s exit=FOLLOW:clear rev=0[] PASS
  -- stuck --
    zero episodes
  -- transitions --
    BRAKE->FOLLOW [clear] x1
    GATEWAY->REVERSE [blocked] x1
    REVERSE->TURN [left] x1
    TURN->BRAKE [coast] x1
  -- sensors (valid/sat/held/none) --
    fl: valid=16.8% sat=75.9% held=0.2% none=7.0%
    fr: valid=100.0% sat=0.0% held=0.0% none=0.0%
    sl: valid=100.0% sat=0.0% held=0.0% none=0.0%
    sr: valid=100.0% sat=0.0% held=0.0% none=0.0%
  -- mean |wheel| by state --
    BRAKE: 2.00 rad/s
    FOLLOW: 2.35 rad/s
    GATEWAY: 2.35 rad/s
    REVERSE: 3.00 rad/s
    TURN: 1.86 rad/s
  -- lateral: mean|offset|=0.001545055097694231 coverage=86.6% n=21598
  ```

- 2026-10-05 Mac→Linux: maze8 follow-up fix implemented. Stuck
  detection now evaluates only FOLLOW/GATEWAY and clears its partial
  yaw window whenever another maneuver owns the wheels; this prevents
  reverse/turn samples from being combined into a false stuck verdict
  that interrupts a gyro-controlled turn. Junction seeking now reacts
  to a sustained opening on the latched follow-wall side even when the
  opposite wall remains present. Repeated wedge recovery now commits
  to the clearer side after 2 cycles (was 3). Regression tests cover
  the turn/stuck interaction, a one-sided T-junction opening, and the
  wedge side choice. Local validation: 3/3 new tests, 5/5 turn-plant
  tests, and all 5 corridor replay scenarios pass.

  **Next run: maze9** on this controller. Use the normal full-maze
  procedure above, capture sim stdout, and let it run to `MAZE SOLVED`
  or the 10-minute limit. Report turn results, FOLLOW→TURN reasons
  (`gap_left/right`), wedge-cycle counts, stuck transitions, and the
  sim verdict. This run is needed to establish whether the maze-level
  path choice improved; offline tests cannot validate the route.
- 2026-10-05 Mac verdict on `maze2` (55k ticks, stopped early):
  turns now read 5/5 PASS after an analyzer fix -- the reported
  `lost_left` FAIL (+86.8 deg) was a TOOLING artifact: gap/lost
  turns log one arming tick (state TURN, zeros out) before the
  servo initializes, so `turn_target` is empty on the episode's
  first row and analyze inferred +90 instead of the logged +180.
  The turn actually completed 180 deg within -3.2 deg. Stuck 2x
  cleared ~1.15 s with one escalation that also cleared. The
  instant-left-turn-at-spawn question is OPEN (symmetric readings:
  fl=fr=0.100 frozen, sl=sr=1.14): awaiting eyes-on-sim -- see
  next handoff entry.
- 2026-10-05 Mac→Linux: user confirmed spawn view = "Gap ahead,
  posts both sides". Implemented GATEWAY probe: symmetric close
  fronts (<0.15, within 0.03) + open sides (>=0.30) = frame to
  squeeze through, NOT a wall. Creeps at 50% cruise centered on
  e_front trim; exits on pass-through, timeout (4s), or falls
  through to normal escape. Verified: fires on spawn signature,
  passes through, times out, does NOT fire on asymmetric fronts
  (real wall) or close sides (corridor). Full regression green.
  Next: `maze3` full run -- expect gateway to carry the bot through
  the entrance at t=0.
- 2026-10-05 Mac→Linux: maze9 corridor over-turn follow-up fixed.
  WEDGE now requires sustained close readings on both sides AND a
  blocked front, so a clear-front narrow straight corridor stays in
  FOLLOW. Turn completion now keys on gyro angle after braking; if the
  front remains blocked, the completed turn is accepted and the next
  tick makes a fresh obstacle decision instead of repeating a full
  target turn. Added regressions for both maze9 failures and for a
  blocked-front turn handoff. Local validation: 5/5 controller
  regressions, 5/5 turn-plant cases, and all 5 corridor replay
  scenarios pass.

  **Next run: maze10** with this commit. Specifically report whether a
  long corridor remains in FOLLOW when side readings are both <0.08 m
  but the front is clear; whether any turn repeats immediately after
  reaching its gyro target; and the usual analyzer + sim verdict.
- 2026-10-05 Mac→Linux: single-front-ray escape trigger simplified.
  The nearest front ray still controls cautious speed and front PD, but
  obstacle escape now requires BOTH front sensors below FRONT_STOP_DIST.
  Thus one splayed ray grazing a side wall cannot trigger reverse or a
  180° dead-end turn. Added regressions for one-ray continuation and
  two-ray escape. Validation: 7/7 controller tests, 5/5 turn-plant
  tests, and all 5 corridor replay scenarios pass.

  **Next run: maze12** with this change. Watch for single-ray readings
  below 0.15 m: the bot should remain in FOLLOW and steer/slow; only
  simultaneous close readings should enter REVERSE/TURN. Record all
  escape transitions and whether 180° turns still repeat.
- 2026-10-05 Mac→Linux: remove persistent gateway override after
  handoff. `gateway_passed` and its side-corridor latch are removed;
  while the probe itself remains bounded, normal FOLLOW always uses
  the nearest front ray for slowdown and the ordinary front PD. Added
  a maze12-signature regression (one close front, one open front,
  symmetric close side walls). Validation: 8/8 controller tests,
  5/5 turn-plant tests, and all 5 corridor replay scenarios pass.

  **Next run: maze13**. Confirm the gateway hands off normally, then
  when one front ray is <0.15 m and the other is open, check the bot
  slows and steers away without a blind U-turn. Report transitions and
  the sim observation/log as usual.

- 2026-10-05 Mac→Linux: maze13 exposed the remaining near-contact case:
  `fr` stayed valid near 0.044 m while `fl` was unavailable, so the
  two-ray front error was unknown and the bot remained in FOLLOW at its
  minimum forward-speed clamp. Added bounded single-ray steering away,
  a zero-speed taper near contact, and a straight `FRONT_BACKOUT` latch
  at 0.06 m that backs until the same ray clears 0.10 m. The existing
  both-rays-blocked REVERSE/TURN behavior is unchanged. Added regressions
  for the maze13 sensor signature and the release hysteresis.

  **Next run: maze14.** Pull the controller changes, launch the broker,
  then launch the sim with forced stdout/stderr capture (do not use a
  pipe to `tee`; it can leave the simulator's output buffered):
  `script -q -f -c 'stdbuf -o0 -e0 ./task_1b_launch' sim_maze14.log`
  Start the controller separately with `python3 task_1b_boilerplate.py
  --label maze14`. Let it run until `MAZE SOLVED` or ~10 minutes. Before
  committing, verify `sim_maze14.log` exists and is non-empty (`test -s
  sim_maze14.log`) and that it contains the simulator verdict, time,
  score, and collision count. If the log is empty or lacks those lines,
  do not report a clean run: capture the sim terminal output with a PTY
  and rerun. Commit raw controller logs and simulator output together,
  without bundling any code changes.

  Report: simulator verdict/time/score/collisions; first and every
  `FRONT_BACKOUT` transition; whether the close ray cleared after
  backing; any wall contact; and the standard `analyze_run.py` output.
  This run determines whether the close-ray intervention prevents the
  maze13 collision without creating repeated backing or blocking turns.
