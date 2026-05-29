from pathlib import Path
import re
import unittest


SCRIPT = Path(__file__).with_name("arm_pick_place.py")


class ArmPickPlaceFlowTest(unittest.TestCase):
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
