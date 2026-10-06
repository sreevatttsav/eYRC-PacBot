"""Regression tests for newly surfaced run failure modes."""
import unittest

from analyze_run import (
    analyze_heading_drift, analyze_repeated_state_signatures,
    analyze_steer_cancellation,
)


class AnalyzeRunRegressionTests(unittest.TestCase):
    def test_heading_error_over_twenty_degrees_for_five_seconds_flags(self):
        ticks = [{"e_heading": "0.4", "dt_rep": "0.1",
                  "t_wall": f"{i * 0.1:.1f}"} for i in range(60)]
        result = analyze_heading_drift({"ticks": ticks})
        self.assertEqual(len(result["flagged"]), 1)
        self.assertGreater(result["flagged"][0]["duration_s"], 5.0)

    def test_large_opposing_steer_components_with_zero_total_flag(self):
        row = {
            "steer_lat": "1.2", "steer_front": "-1.15",
            "steer_heading": "0.0", "steer": "0.05",
            "t_wall": "1.0", "state": "FOLLOW",
        }
        result = analyze_steer_cancellation({"ticks": [row]})
        self.assertEqual(result["count"], 1)

    def test_three_similar_front_backouts_are_reported_as_cycle(self):
        rows = []
        for i in range(3):
            for state in ("FOLLOW", "FRONT_BACKOUT", "FOLLOW"):
                rows.append({
                    "state": state, "t_wall": str(len(rows) * 0.1),
                    "fl_f": "0.075", "fr_f": "0.240",
                    "sl_f": "0.080", "sr_f": "0.130",
                })
        result = analyze_repeated_state_signatures({"ticks": rows})
        self.assertEqual(len(result["repeated"]), 1)
        self.assertEqual(result["repeated"][0]["count"], 3)


if __name__ == "__main__":
    unittest.main()
