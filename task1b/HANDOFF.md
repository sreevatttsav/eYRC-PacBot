# PacBot Controller Handoff

## Current objective

Make the PacBot traverse the maze reliably. The gyro turn execution is accurate; the remaining problems are route authorization, gateway handling, post-turn acquisition, and recovery from forward wedges.

## Current controller policy

Main file:

```text
task1b/task_1b_boilerplate.py
```

Current state flow:

```text
GATEWAY -> FOLLOW -> ROUTE_BUFFER -> REVERSE
         -> TURN_ENTRY_STOP -> TURN -> BRAKE
         -> POST_TURN_ADVANCE -> FOLLOW
```

### Gateway

- Normal FOLLOW logic is not enabled until a real side wall is observed.
- The previous distance-only gateway handoff was removed because it handed the symmetric spawn posts to FOLLOW and caused an invalid first turn.

### Front sensing

- Front ToF rays are splayed by approximately 20 degrees.
- A normal front route requires both forward rays to be close.
- A single front ray is treated as ambiguous/grazing evidence and should not independently authorize a route.
- A single ray at emergency distance is handled as safety contact, not as left/right route evidence.

### Side opening sensing

- A side opening must follow a previously observed side wall.
- The opening must persist for `SIDE_OPEN_CONFIRM_S` before becoming a candidate.
- The selected side is latched through route buffering and turn entry.

### Turns

- Gyro targets remain exact 90-degree or 180-degree targets.
- ToF readings do not rewrite the gyro target.
- TURN and BRAKE own the wheels; corridor centering is not active during them.
- Current turn PID/local tests have historically passed with final errors normally within about 5 degrees.

### Turn entry

- The robot enters `REVERSE` before a normal turn.
- Current backoff is bounded and short (`TURN_ENTRY_BACKOFF_M` plus `TURN_ENTRY_BACKOFF_MAX_S`).
- `TURN_ENTRY_STOP` holds the wheels stopped before starting the gyro turn.

### Post-turn acquisition

- The robot holds briefly, then creeps through the outgoing corridor.
- It does not hand off to FOLLOW while the outgoing path is classified as blocked/emergency unless the wide-mouth recovery logic allows controlled creep.
- Persistent blocked/emergency post-turn states have bounded recovery paths into REVERSE.

### Forward recovery

- A frozen ToF signature while commanding forward motion is monitored by the forward-progress watchdog.
- First suspected wedge: bounded reverse, then return to FOLLOW.
- Repeated same-signature wedge: bounded recovery remains reverse-only; it must not select a route or create a 180-degree turn.

## Important historical findings

### Best historical navigation baseline

`lean28` was the strongest overall traversal run:

- approximately 25 completed turns;
- reliable FOLLOW handoffs;
- route selection functioned better than later conservative versions;
- accumulated heading/position error remained the main issue.

Its weakness was not basic gyro turning. It sometimes entered turns from poor geometry and had long post-turn acquisition periods.

### Recent failures and their causes

#### Invalid gateway turn

The gateway handed off with readings similar to:

```text
front: 0.10 / 0.10 m
sides: 0.556 / 0.556 m
```

This was the spawn-post signature, not a junction. FOLLOW interpreted it as a blocked route. The current gateway handoff fix requires an observed side wall.

#### Premature splayed-ray turn

Examples such as:

```text
front: 0.115 / 0.098 m
sides: 0.245 / 0.235 m
```

were previously interpreted as front blockage because the center average crossed the threshold. The two-ray blocked-front rule was added to prevent this.

#### Watchdog-generated turns

Older runs contained:

```text
FOLLOW -> REVERSE [forward_stalled]
```

followed by the normal route chooser and sometimes a default 180-degree turn. This was wrong: a motion watchdog does not know whether a U-turn is appropriate. Watchdog recovery is now separated from route selection.

#### Post-turn deadlock

Some runs stayed in `POST_TURN_ADVANCE` for tens or hundreds of seconds. The robot was stopped by persistent `blocked_pair` or `emergency` readings. Bounded post-turn recovery was added, but this remains an area to validate carefully.

#### Conservative route suppression

When single-ray turns were completely disabled, runs such as `lean31` failed to turn at all. This showed that the simulator's valid branch geometry is not always represented by a clean side-opening label. Any future relaxation must be based on a specific geometry signature, not a global threshold change.

## Current known risks

1. The current conservative side-opening policy may fail to recognize valid branches when side-wall loss is not clean.
2. The route chooser still has a dead-end fallback to 180 degrees when a confirmed front blockage has no opening. This is valid only if the blockage classification is trustworthy.
3. Post-turn acquisition can still be slow or unsafe when the front rays see a rotated-away corner.
4. The simulator does not provide rear rangefinders, encoders, or pose feedback, so reverse distance and physical wedging are inferred from front/side ToF only.
5. The simulator verdict commonly remains `pending`; a long run does not prove the maze was solved.

## Recommended next investigation

Do not tune the gyro PID first. Use short, labeled simulations and classify each transition.

For each turn, record:

```text
route evidence:
  front rays, side rays, hazard, side-opening latch

turn entry:
  side clearance at ROUTE_BUFFER -> REVERSE -> TURN

turn execution:
  target, final gyro error, duration

exit:
  POST_TURN duration, front hazard, side acquisition, handoff state
```

The preferred next design is:

1. Keep gateway protection.
2. Keep exact gyro turning.
3. Keep watchdog recovery unable to choose routes.
4. Add explicit logging for route evidence and latched direction.
5. Compare valid branch signatures from the old `lean28` logs against rejected candidates.
6. Add only a narrowly targeted route exception for a confirmed valid signature.

## Useful commands

From `task1b/`:

```bash
python3 -m py_compile task_1b_boilerplate.py
python3 tune_local.py --turn-tests
python3 -m unittest test_controller_regressions

LD_LIBRARY_PATH="$PWD/lib" QT_QPA_PLATFORM=offscreen \
  script -qec "./task_1b_launch" sim_label.log

python3 task_1b_boilerplate.py --label label
python3 analyze_run.py logs/<run-directory>
```

Stop a run before changing code:

```bash
pgrep -af "[t]ask_1b_launch|[t]ask_1b_boilerplate"
kill <sim-pid> <controller-pid>
```

## Recent run references

```text
lean28: strongest historical navigation baseline
lean31: overly strict route gate; did not turn
lean37: good short run; 5 accurate turns and FOLLOW handoffs
lean38: watchdog-induced false turn/recovery regression
lean42: conservative post-turn containment; stalled safely
lean43: 8 accurate turns; exposed front-ray early commitment
lean44: 12 accurate turns; eventually stuck in POST_TURN emergency
lean49: restored route behavior; repeated forward-stall/recovery loop
lean50: escalated recovery; controlled 90-degree escape test
lean51: front-entry gates caused route suppression/recovery behavior
lean52: strict two-ray/side-opening policy made the initial turn invalid at the gateway
```
