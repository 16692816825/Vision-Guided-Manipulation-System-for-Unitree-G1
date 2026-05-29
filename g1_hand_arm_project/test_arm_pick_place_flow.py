from pathlib import Path
import re
import unittest


SCRIPT = Path(__file__).with_name("arm_pick_place.py")


class ArmPickPlaceFlowTest(unittest.TestCase):
    def test_safety_return_runs_before_pick_flow(self):
        text = SCRIPT.read_text(encoding="utf-8")

        self.assertIn("SAFE_START_Q", text)
        for joint in (15, 16, 17, 18, 19, 22, 23, 24, 25, 26):
            self.assertRegex(text, rf"{joint}:\s*-?\d+\.\d+")

        self.assertIn("SAFE_START_TOLERANCE_RAD", text)
        self.assertIn("SAFE_RETURN_SECONDS", text)
        self.assertIn("def is_near_safe_start", text)
        self.assertIn("def ensure_safe_start", text)
        self.assertIn("safety: slowly return arms to safe start", text)

        run_tail = text[text.index("def run") :]
        enable_index = run_tail.index("enable and hold both arms")
        safety_index = run_tail.index("self.ensure_safe_start()")
        pick_index = run_tail.index("pick: fold forearm near upper arm")
        self.assertLess(enable_index, safety_index)
        self.assertLess(safety_index, pick_index)

    def test_pick_place_lifts_then_returns_to_release_pose(self):
        text = SCRIPT.read_text(encoding="utf-8")

        self.assertIn("LIFT_BOTTLE_DELTA", text)
        self.assertRegex(text, r"LIFT_BOTTLE_DELTA\s*=\s*{[^}]*18:\s*-1\.05")
        self.assertIn("self.q_lift_bottle", text)

        bottle_index = text.index('run_left_hand("bottle")')
        tail = text[bottle_index:]
        expected_order = [
            "pick: hold after grasp",
            "pick: lift forearm with bottle",
            "pick: hover with bottle lifted",
            "pick: lower bottle back to release position",
            "place: hold before release",
            'run_left_hand("open")',
            "place: fold forearm back after release",
            "place: final empty-hand back to initial",
            "release arm_sdk",
        ]

        pos = -1
        for marker in expected_order:
            next_pos = tail.find(marker, pos + 1)
            self.assertNotEqual(next_pos, -1, marker)
            pos = next_pos

        removed_markers = [
            "pick: upper arm outward after grasp",
            "pick: fold forearm back with bottle",
            "pick: retract outward with bottle",
            "hold bottle at outward home",
            "place: start from outward home",
            "place: lift and move folded arm",
            "place: unfold forearm to release position",
        ]
        for marker in removed_markers:
            self.assertNotIn(marker, tail)


if __name__ == "__main__":
    unittest.main()
