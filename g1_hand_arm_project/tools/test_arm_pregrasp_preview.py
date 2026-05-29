import unittest

from arm_pregrasp_preview import (
    ALL_ARM,
    LEFT_ARM,
    apply_limited_offsets,
    build_targets,
    parse_joint_offsets,
)


class ArmPregraspPreviewTests(unittest.TestCase):
    def test_parse_joint_offsets_accepts_repeated_and_comma_forms(self):
        self.assertEqual(
            parse_joint_offsets(["16=0.03,18=-0.02", "19=0.01"]),
            {16: 0.03, 18: -0.02, 19: 0.01},
        )

    def test_apply_limited_offsets_rejects_unknown_joint(self):
        base = {joint: 0.0 for joint in ALL_ARM}

        with self.assertRaises(ValueError):
            apply_limited_offsets(base, {14: 0.01}, allowed_joints=LEFT_ARM, max_abs_delta=0.05)

    def test_apply_limited_offsets_rejects_large_delta(self):
        base = {joint: 0.0 for joint in ALL_ARM}

        with self.assertRaises(ValueError):
            apply_limited_offsets(base, {16: 0.08}, allowed_joints=LEFT_ARM, max_abs_delta=0.05)

    def test_build_targets_applies_offsets_only_to_pregrasp_target(self):
        q0 = {joint: float(joint) for joint in ALL_ARM}

        targets = build_targets(q0, pregrasp_offsets={16: 0.03, 18: -0.02}, max_abs_offset=0.05)

        self.assertEqual(targets["q0"], q0)
        self.assertEqual(targets["q_fold"][16], q0[16] + 0.20)
        self.assertEqual(targets["q_lift_folded"][16], q0[16] + 0.20)
        self.assertAlmostEqual(targets["q_unfold_pregrasp"][16], q0[16] + 0.16 + 0.03)
        self.assertAlmostEqual(targets["q_unfold_pregrasp"][18], q0[18] - 0.60 - 0.02)


if __name__ == "__main__":
    unittest.main()
