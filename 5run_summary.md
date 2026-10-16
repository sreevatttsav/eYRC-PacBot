# Lattice controller: five-run batch

Date: 2026-10-16
Branch: `lattice-controller-from-scratch`
Controller: `lattice-from-scratch`

Each run was given a 120-second wall-clock allowance. The simulator stopped
emitting sensor frames before the allowance in four runs, so the durations
below are the actual simulated durations recorded by the controller.

| Run | Simulated duration | Turns completed | Recoveries | Final state | Final cell | Simulator result |
|---|---:|---:|---:|---|---|---|
| 1 | 98.5 s | 8 | 9 | PIVOT | [1, 0] | none emitted |
| 2 | 21.5 s | 3 | 4 | RECOVER | [0, 0] | none emitted |
| 3 | 20.5 s | 2 | 3 | PIVOT | [0, 0] | none emitted |
| 4 | 21.0 s | 2 | 3 | PIVOT | [0, 0] | none emitted |
| 5 | 22.5 s | 2 | 3 | PIVOT | [0, 0] | none emitted |

The raw controller traces are in the corresponding `logs/*lattice_2min_*`
directories. The launcher and controller stdout captures are included beside
this summary. No simulator `result.json` was produced in these runs.
