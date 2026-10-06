"""Result-topic persistence regression tests."""
import json
import os
import tempfile
import unittest

from runlog import RunLogger


class RunLogResultTests(unittest.TestCase):
    def test_result_payload_is_persisted_in_run_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = RunLogger(tmp, "result-test", constants={})
            payload = {
                "solved": True, "collisions": 1,
                "time_sec": 12.5, "score": 42.0,
            }
            logger.result(payload)
            run_dir = logger.run_dir
            logger.close()

            with open(os.path.join(run_dir, "result.json")) as f:
                saved = json.load(f)
            self.assertEqual(saved["status"], "received")
            self.assertEqual(saved["messages"][-1]["payload"], payload)

    def test_missing_result_is_explicit_at_shutdown(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = RunLogger(tmp, "no-result-test", constants={})
            run_dir = logger.run_dir
            logger.close()

            with open(os.path.join(run_dir, "result.json")) as f:
                saved = json.load(f)
            self.assertEqual(saved["status"], "not_received_before_shutdown")
            self.assertEqual(saved["messages"], [])


if __name__ == "__main__":
    unittest.main()
