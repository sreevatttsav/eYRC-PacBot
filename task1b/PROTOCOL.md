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
