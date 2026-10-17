"""Summarize the scratch controller's own run folder.

This does not turn an offline sensor replay into a simulator verdict.
"""

from collections import Counter
import json
from pathlib import Path
import statistics
import sys

from lattice import PITCH


def load_lines(path: Path):
    if not path.exists():
        return []
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def analyze(directory: Path) -> dict:
    ticks = load_lines(directory / "ticks.jsonl")
    events = load_lines(directory / "events.jsonl")
    turns = [e for e in events if e.get("event") == "turn_complete"]
    reasons = Counter(e.get("reason") for e in events if "reason" in e)
    spacings = []
    last = None
    for tick in ticks:
        spacing = tick.get("last_edge_spacing")
        if spacing is not None and spacing != last:
            spacings.append(spacing)
            last = spacing
    spacing_errors = [abs(v - round(v / PITCH) * PITCH) for v in spacings]
    verdict_path = directory / "result.json"
    verdict = json.loads(verdict_path.read_text()) if verdict_path.exists() else None
    return {
        "verdict": verdict,
        "seconds_logged": ticks[-1]["t"] if ticks else 0.0,
        "last_state": ticks[-1]["state"] if ticks else None,
        "last_cell": ticks[-1]["cell"] if ticks else None,
        "last_channel_width_m": (ticks[-1].get("channel_width_m")
                                 if ticks else None),
        "last_channel_width_slope": (ticks[-1].get("channel_width_slope")
                                     if ticks else None),
        "channel_acquired": (ticks[-1].get("channel_acquired")
                             if ticks else None),
        "suspected_stalls": sum(e.get("event") == "tof_stall_suspected"
                                for e in events),
        "turns": len(turns),
        "turn_errors_deg": [round(e["error_deg"], 2) for e in turns],
        "turns_within_5deg": sum(abs(e["error_deg"]) <= 5 for e in turns),
        "pivot_center_errors_m": [e["center_error_m"] for e in turns
                                  if e.get("center_error_m") is not None],
        "edge_spacings_m": [round(v, 4) for v in spacings],
        "median_edge_pitch_error_m": (statistics.median(spacing_errors)
                                      if spacing_errors else None),
        "duplicate_edges": ticks[-1].get("duplicate_edges", 0) if ticks else 0,
        "rejected_anchors": ticks[-1].get("rejected_anchors", 0) if ticks else 0,
        "repeated_cell_heading": sum(e.get("event") == "repeated_cell_heading"
                                     for e in events),
        "state_reasons": dict(reasons),
    }


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: python3 analyze_run.py logs/<run-folder>")
    print(json.dumps(analyze(Path(sys.argv[1])), indent=2))


if __name__ == "__main__":
    main()
