from pathlib import Path
import re
import unittest


SCRIPT = Path(__file__).with_name("arm_pick_place.py")
STANDALONE_SCRIPT = Path(__file__).with_name("arm_pick_place_standalone.py")
RIGHT_HAND_SCRIPT = Path(__file__).with_name("right_hand_thumb_test.py")


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

    def test_standalone_flow_embeds_hand_control(self):
        self.assertTrue(STANDALONE_SCRIPT.exists(), "arm_pick_place_standalone.py is missing")
        text = STANDALONE_SCRIPT.read_text(encoding="utf-8")

        self.assertIn("HAND_POSES", text)
        self.assertIn('"thumb_ready": [0, 1000, 0, 0, 0, 0]', text)
        self.assertIn('"bottle": [180, 850, 480, 560, 540, 420]', text)
        self.assertIn("async def send_hand_pose", text)
        self.assertIn("get_finger_positions", text)
        self.assertIn("REQUIRE_THUMB_AUX_READY = True", text)
        self.assertIn("def validate_thumb_aux_ready", text)
        self.assertIn("abort_return", text)
        self.assertNotIn("sixth channel is currently used", text)

        self.assertNotIn("left_hand_safe_once.py", text)
        self.assertNotIn("HAND_SCRIPT", text)
        self.assertNotIn("subprocess", text)

    def test_standalone_forearm_targets_are_raised(self):
        self.assertTrue(STANDALONE_SCRIPT.exists(), "arm_pick_place_standalone.py is missing")
        text = STANDALONE_SCRIPT.read_text(encoding="utf-8")

        self.assertRegex(text, r"UNFOLD_PREGRASP_DELTA\s*=\s*{[^}]*18:\s*-0\.19")
        self.assertRegex(text, r"LIFT_BOTTLE_DELTA\s*=\s*{[^}]*18:\s*-1\.0")

    def test_standalone_hand_and_safety_order(self):
        self.assertTrue(STANDALONE_SCRIPT.exists(), "arm_pick_place_standalone.py is missing")
        text = STANDALONE_SCRIPT.read_text(encoding="utf-8")

        run_tail = text[text.index("async def run") :]
        enable_index = run_tail.index("enable and hold both arms")
        safety_index = run_tail.index("self.ensure_safe_start()")
        pick_index = run_tail.index("pick: fold forearm near upper arm")
        self.assertLess(enable_index, safety_index)
        self.assertLess(safety_index, pick_index)

        thumb_open_index = run_tail.index('await self.send_hand_pose("thumb_open_max"')
        thumb_ready_index = run_tail.index('await self.send_hand_pose("thumb_ready"')
        bottle_index = run_tail.index('await self.send_hand_pose("bottle"')
        self.assertLess(thumb_open_index, thumb_ready_index)
        self.assertLess(thumb_ready_index, bottle_index)

        self.assertIn("hand_only_test", text)
        self.assertIn("release arm_sdk", run_tail)

    def test_right_hand_thumb_test_is_hand_only(self):
        self.assertTrue(RIGHT_HAND_SCRIPT.exists(), "right_hand_thumb_test.py is missing")
        text = RIGHT_HAND_SCRIPT.read_text(encoding="utf-8")

        self.assertIn('HAND_PORT = "/dev/ttyUSB2"', text)
        self.assertIn("HAND_SLAVE_ID = 0x7F", text)
        self.assertIn('"thumb_ready": [0, 1000, 0, 0, 0, 0]', text)
        self.assertIn('"bottle": [180, 850, 480, 560, 540, 420]', text)
        self.assertIn("open -> thumb_ready -> bottle -> open", text)
        self.assertIn("CONNECT_RETRIES = 3", text)
        self.assertIn("for attempt in range(1, self.connect_retries + 1):", text)
        self.assertIn("connect attempt", text)
        self.assertIn("--connect-retries", text)

        self.assertNotIn("unitree_sdk2py", text)
        self.assertNotIn("rt/arm_sdk", text)
        self.assertNotIn("LowCmd", text)
        self.assertNotIn("subprocess", text)


if __name__ == "__main__":
    unittest.main()
