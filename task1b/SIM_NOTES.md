# Task 1B sim facts, extracted from the `task_1b_launch` binary
(`strings task_1b_launch` — the MJCF model XML is embedded in it).
No behavior was changed to obtain these; they ground the constants
the controller previously had to guess.

## Robot (model `micromouse_maze`, body `base`)
- Wheels: hinge joints directly on wheel bodies (direct drive, no gear).
  - `left_wheel` at `pos="-0.033 0.039 0"`, `right_wheel` at `-0.033 -0.039 0`
  - **track = 0.078 m** (was 0.08 guess)
  - wheel geom `size="0.017 0.0045"` (cylinder r/h) → **R = 0.017 m**
  - **K_LIN = R = 0.017 m/s per wheel rad/s** (straight rolling; slip ~1-5%)
- Chassis collision box `size="0.046 0.043 0.011"` (half extents) =
  92 x 86 mm footprint (matches `chassis_sim.stl`).
- Caster at rear (`pos="0.038 0 -0.011"`); forward = +X
  (wheels at x=-0.033 = rear, caster +0.038 = front).
- Mass 0.20 kg (`inertial`).
- Wheel friction `1.0 0.005 0.0001`.

## ToF rangefinders (4x, MuJoCo `rangefinder` sensors)
- `tof_fl` at `(0.036, +0.005)`, `tof_fr` at `(0.036, -0.005)`:
  **20 deg off +X, splayed** (per the embedded comment: "mostly
  forward-looking, their job is spotting the wall AHEAD").
- `tof_sl` at `(-0.005, +0.034)`, `tof_sr` at `(-0.005, -0.034)`:
  lateral (side-looking).
- Consequence for speed work: front range understates travel by
  cos(splay) ~= cos20 deg ~= 0.94 (~6%), and only while facing the
  wall squarely. A rotating robot's rays sweep across walls, so
  range-rate during rotation is NOT translation speed.

## IMU
- MPU6050 (accel + gyro, no magnetometer) at chassis center.
- Payload also carries `accel` (the boilerplate ignores it).

## Solver / timing
- `timestep="0.002"`, `integrator="implicitfast"` → payload `dt`
  is honest; sim-time sums are real durations.

## Maze + task
- 6x6 grid, **cell pitch 0.22 m**, regenerated randomly every run
  (`GenerateMaze`); entrance gap in the perimeter; goal site on the
  east border at the exit gap (solved = robot x passes goal x).
- Spawn: **just outside the entrance, facing +X into the maze**,
  nose ~0.10 m from the entrance wall, open space behind.
- Topics: `pacbot/sensors` (sim -> controller), `pacbot/wheel_vel`
  (controller -> sim), `pacbot/result`.

## Implications for calibration
- `YAW_GAIN_K = 0.0914` (step_test) vs kinematic `R/track = 0.218`:
  the sim loses ~58% in yaw (contact slip while spinning). Use the
  measured value for turns; do NOT derive it from geometry.
- `K_LIN = 0.017` (geometry). The s2 maze-slot slope fits (0.05-0.17)
  are rotation-contaminated and must not overwrite it.
- speed_test needs a straight unobstructed translation to *confirm*
  0.017, not discover it: back out of the entrance (open behind),
  fit the opening slope there.
