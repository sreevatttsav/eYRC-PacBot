# PacBot Controller Handoff

## Repository state

- Repository: `eYRC-PacBot`
- Active branch: `post-turn-acquisition`
- Remote: `origin/post-turn-acquisition`
- Latest code commit: `ed9d113`
- Simulator: stopped
- Local simulator/controller logs are untracked and should not be committed as code changes.

The branch is based on `main` and contains the hybrid flood-fill experiment,
the turn-controller work, and the post-turn acquisition changes described
below.

## Validation

From `task1b/`:

```bash
python3 -m pytest -q
python3 tune_local.py --turn-tests
```

Current validation result before this document was added:

- `59 passed`
- All local turn tests pass:
  - left 90 degrees
  - right 90 degrees
  - 180-degree dead end
  - slow turn
  - stalled turn/no-progress recovery

## Controller architecture

`task1b/task_1b_boilerplate.py` uses a priority pipeline:

1. Sensor filtering and wall/front classification
2. Post-turn settle/verify/acquisition
3. Stuck recovery
4. Wedge handling
5. Gateway/entrance handling
6. Front safety and emergency handling
7. Route selection and gyro turn execution
8. Wall-following

The turn controller uses:

- Integrated gyro angle
- Proportional turn control
- Wheel-speed ramping
- Bounded lateral-PD assist
- Brake phase with low-yaw dwell
- TOF opening cue for late 90-degree braking
- Post-turn sensor verification

## Current route hierarchy

When the front is blocked, route selection currently checks:

1. Persistent side opening classification
2. Fresh numeric side clearance (`SIDE_ROUTE_MIN_M = 0.20 m`)
3. Side-clearance comparison when both sides are plausible
4. Flood-fill tie-break when both sides are equivalent
5. Left exploratory probe only when neither side provides usable evidence

Side opening classification uses a `0.50 s` dwell. The flood-fill planner is
in `task1b/flood_fill.py`; it maintains an optimistic 6x6 map and only ranks
locally observed route choices.

## Sensor geometry

From `task1b/SIM_NOTES.md`:

- Wheel radius: `0.017 m`
- Track width: `0.078 m`
- Chassis half-width: `0.043 m`
- Front sensors: `(0.036, +/-0.005)`, 20-degree splay
- Side sensors: `(-0.005, +/-0.034)`, lateral-facing
- Maze cell pitch: `0.22 m`
- Documented corridor walls: approximately `+/-0.085 m`
- Centered side readings: approximately `0.051 m`

Front-path classification now uses the average of both splayed front rays as
a centerline estimate, with a near-contact guard. A single close splayed ray
should not automatically imply that the centerline is blocked.

## Current acquisition behavior

The `post-turn-acquisition` branch contains a route latch and a bounded
forward acquisition phase. The selected direction remains fixed through the
turn and initial acquisition. Lateral PD plus heading hold are used while
advancing into the selected corridor.

There is also a route-decision buffer:

- Buffer distance: `0.05 m`
- Buffer speed: `0.018 m/s`

It creeps forward when the centerline is safe but side openings are not yet
visible, then makes the route decision.

## Most recent simulator evidence

The latest route-aware run (`routeaware1`) showed:

- Gateway entry succeeded.
- A visible opposite-side opening was correctly allowed to route selection
  instead of causing immediate `FRONT_BACKOUT`.
- Several left and right turns completed accurately.
- The remaining repeated failure was:

```text
POST_TURN_VERIFY -> POST_TURN_ADVANCE -> REVERSE
```

This means the main remaining blocker is reliable entry into the selected
outgoing corridor, not the basic gyro angle controller.

## Known risks and next work

1. **Post-turn acquisition still rejects or loses corridors.**
   The bot can complete a turn accurately but may reverse before it has
   physically entered the new corridor. The acquisition distance, front
   centerline clearance, and selected-side confirmation need coordinated
   tuning.

2. **Exploratory probes can be ambiguous.**
   A left probe is only confirmed after fresh front centerline clearance.
   Do not let wide side readings alone prove that a route exists.

3. **Speed changes affect gateway behavior.**
   Long-corridor cruise was increased to `0.06 m/s`, but gateway speed is
   held separately at `0.04 m/s`.

4. **Flood fill should remain secondary.**
   Do not rely on the map until physical corridor acquisition is stable.
   Flood fill should rank confirmed/local routes, not override live sensor
   evidence.

5. **Recommended next experiment.**
   Instrument and tune post-turn acquisition using a fixed left/right corner:

   - Record front centerline clearance
   - Record selected-side clearance
   - Record lateral error
   - Record acquisition distance
   - Count `POST_TURN_ADVANCE -> REVERSE`

   Change one acquisition parameter at a time.

## Running a simulator trial

From `task1b/`:

```bash
LD_LIBRARY_PATH="$PWD/lib" QT_QPA_PLATFORM=offscreen \
  script -qec "./task_1b_launch" sim_label.log

python3 task_1b_boilerplate.py --label label
python3 analyze_run.py logs/<run-directory>
```

Do not add the generated `controller_*.log`, `sim_*.log`, or temporary swap
files to a code commit.
