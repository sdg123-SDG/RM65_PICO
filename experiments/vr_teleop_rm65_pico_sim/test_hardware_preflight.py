"""Recorded-input checks only; never connects to the controller."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from hardware_preflight import check_inventory


class RecordedPreflightTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original = json.loads((HERE / "real_readonly_inventory.json").read_text(encoding="utf-8"))

    def evaluate(self, mutate=None):
        report = copy.deepcopy(self.original)
        if mutate:
            mutate(report)
        with tempfile.TemporaryDirectory(prefix="rm65-preflight-") as tmp:
            path = Path(tmp) / "inventory.json"
            path.write_text(json.dumps(report), encoding="utf-8")
            return check_inventory(path)

    def test_saved_snapshot_passes_only_offline(self):
        result = self.evaluate()
        self.assertTrue(result["offline_geometry_and_joint_checks_passed"])
        self.assertFalse(result["live_feedback_freshness_checked"])
        self.assertFalse(result["offline_validator_opened_robot_connection"])
        self.assertFalse(result["real_motion_tested"])

    def test_controller_errors_and_disabled_axis_rejected(self):
        for mutate in (
            lambda r: r["responses"]["get_current_arm_state"]["arm_state"].update(sys_err=1),
            lambda r: r["responses"]["get_joint_en_state"]["en_state"].__setitem__(1, 0),
        ):
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                self.evaluate(mutate)

    def test_nonzero_work_frame_and_inconsistent_fk_rejected(self):
        for mutate in (
            lambda r: r["responses"]["get_current_work_frame"]["pose"].__setitem__(0, 1000),
            lambda r: r["responses"]["get_current_arm_state"]["arm_state"]["joint"].__setitem__(1, -20000),
        ):
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                self.evaluate(mutate)

    def test_invalid_joint_and_non_read_only_report_rejected(self):
        for mutate in (
            lambda r: r["responses"]["get_current_arm_state"]["arm_state"]["joint"].__setitem__(0, 200000),
            lambda r: r.update(read_only=False),
        ):
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                self.evaluate(mutate)


if __name__ == "__main__":
    unittest.main()
