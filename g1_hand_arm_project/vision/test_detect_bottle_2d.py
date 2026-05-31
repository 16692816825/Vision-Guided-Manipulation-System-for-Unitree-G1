import unittest

from record_bottle_positions import csv_header, flatten_payload

from compare_bottle_to_target import build_offset_report

from evaluate_grasp_window import build_record_report, summarize_rows

from auto_grasp_decision import build_decision

from detect_bottle_depth import (
    CameraIntrinsics,
    build_output_payload,
    deproject_pixel,
    scale_point,
    sample_depth_median_mm,
    should_retry_reconnect,
)
from detect_bottle_2d import (
    Detection,
    TargetTracker,
    backend_names_for_candidate,
    box_center,
    camera_candidates,
    collect_target_detections,
    normalize_target_names,
    parse_source,
    read_latest_frame,
    select_best_detection,
)


class FakeBox:
    def __init__(self, xyxy, conf, cls_id):
        self.xyxy = [xyxy]
        self.conf = [conf]
        self.cls = [cls_id]


class DetectBottle2DTests(unittest.TestCase):
    def test_box_center_rounds_to_image_pixel(self):
        self.assertEqual(box_center((10.0, 20.0, 30.0, 60.0)), (20, 40))
        self.assertEqual(box_center((10.2, 20.2, 31.8, 61.8)), (21, 41))

    def test_normalize_target_names_accepts_comma_list(self):
        self.assertEqual(
            normalize_target_names("bottle, water bottle, Cup"),
            {"bottle", "water bottle", "cup"},
        )

    def test_parse_source_accepts_auto_index_and_video_path(self):
        self.assertEqual(parse_source("auto"), "auto")
        self.assertEqual(parse_source("2"), 2)
        self.assertEqual(parse_source("/dev/video4"), "/dev/video4")
        self.assertEqual(parse_source("/tmp/input.mp4"), "/tmp/input.mp4")

    def test_camera_prefers_v4l2_backend_for_realsense_capture(self):
        self.assertEqual(backend_names_for_candidate(1), ["v4l2", "default"])
        self.assertEqual(backend_names_for_candidate("/dev/video4"), ["v4l2", "default"])
        self.assertEqual(backend_names_for_candidate("/tmp/input.mp4"), ["default"])

    def test_camera_candidates_prioritizes_realsense_color_paths(self):
        self.assertEqual(camera_candidates("auto")[:2], ["/dev/video4", "/dev/video2"])
        self.assertEqual(camera_candidates("/dev/video4"), ["/dev/video4", 4])

    def test_read_latest_frame_grabs_a_newer_buffered_frame(self):
        class FakeCapture:
            def __init__(self):
                self.frames = ["old", "newer", "latest"]
                self.grabbed = None

            def grab(self):
                if not self.frames:
                    return False
                self.grabbed = self.frames.pop(0)
                return True

            def retrieve(self):
                return self.grabbed is not None, self.grabbed

            def read(self):
                if not self.frames:
                    return False, None
                return True, self.frames.pop(0)

        ok, frame = read_latest_frame(FakeCapture(), flush_frames=3)

        self.assertTrue(ok)
        self.assertEqual(frame, "latest")

    def test_select_best_detection_picks_highest_confidence_bottle(self):
        boxes = [
            FakeBox([0, 0, 10, 10], 0.92, 1),
            FakeBox([20, 30, 80, 110], 0.61, 0),
            FakeBox([40, 50, 140, 170], 0.83, 0),
        ]
        names = {0: "bottle", 1: "person"}

        detection = select_best_detection(
            boxes=boxes,
            names=names,
            target_names={"bottle"},
            min_confidence=0.3,
        )

        self.assertEqual(
            detection,
            Detection(
                label="bottle",
                confidence=0.83,
                xyxy=(40.0, 50.0, 140.0, 170.0),
                center=(90, 110),
            ),
        )

    def test_select_best_detection_returns_none_without_target(self):
        boxes = [
            FakeBox([0, 0, 10, 10], 0.95, 1),
            FakeBox([20, 30, 80, 110], 0.20, 0),
        ]
        names = {0: "bottle", 1: "person"}

        detection = select_best_detection(
            boxes=boxes,
            names=names,
            target_names={"bottle"},
            min_confidence=0.3,
        )

        self.assertIsNone(detection)

    def test_collect_target_detections_returns_all_valid_target_boxes(self):
        boxes = [
            FakeBox([0, 0, 10, 10], 0.92, 1),
            FakeBox([20, 30, 80, 110], 0.61, 0),
            FakeBox([40, 50, 140, 170], 0.83, 0),
        ]

        detections = collect_target_detections(
            boxes=boxes,
            names={0: "bottle", 1: "person"},
            target_names={"bottle"},
            min_confidence=0.3,
        )

        self.assertEqual([item.center for item in detections], [(50, 70), (90, 110)])

    def test_target_tracker_rejects_single_far_jump(self):
        tracker = TargetTracker(
            max_jump_px=40,
            smooth_alpha=1.0,
            lost_frames=3,
            switch_frames=3,
        )
        first = Detection("bottle", 0.8, (80.0, 80.0, 120.0, 160.0), (100, 120))
        far = Detection("bottle", 0.95, (300.0, 80.0, 340.0, 160.0), (320, 120))

        self.assertEqual(tracker.update([first]), first)
        self.assertEqual(tracker.update([far]), first)

    def test_target_tracker_accepts_near_slow_motion_and_smooths(self):
        tracker = TargetTracker(
            max_jump_px=80,
            smooth_alpha=0.5,
            lost_frames=3,
            switch_frames=3,
        )
        first = Detection("bottle", 0.8, (80.0, 80.0, 120.0, 160.0), (100, 120))
        moved = Detection("bottle", 0.7, (100.0, 90.0, 140.0, 170.0), (120, 130))

        tracker.update([first])
        tracked = tracker.update([moved])

        self.assertEqual(tracked.center, (110, 125))
        self.assertEqual(tracked.xyxy, (90.0, 85.0, 130.0, 165.0))

    def test_target_tracker_switches_after_far_target_persists(self):
        tracker = TargetTracker(
            max_jump_px=40,
            smooth_alpha=1.0,
            lost_frames=5,
            switch_frames=2,
        )
        first = Detection("bottle", 0.8, (80.0, 80.0, 120.0, 160.0), (100, 120))
        far = Detection("bottle", 0.95, (300.0, 80.0, 340.0, 160.0), (320, 120))

        tracker.update([first])
        self.assertEqual(tracker.update([far]), first)
        self.assertEqual(tracker.update([far]), far)

    def test_target_tracker_requires_confidence_to_lock_initial_target(self):
        tracker = TargetTracker(
            max_jump_px=40,
            smooth_alpha=1.0,
            lost_frames=3,
            switch_frames=3,
            lock_confidence=0.25,
        )
        weak = Detection("bottle", 0.19, (80.0, 80.0, 120.0, 160.0), (100, 120))
        strong = Detection("bottle", 0.45, (80.0, 80.0, 120.0, 160.0), (100, 120))

        self.assertIsNone(tracker.update([weak]))
        self.assertEqual(tracker.update([strong]), strong)


class DetectBottleDepthTests(unittest.TestCase):
    def test_sample_depth_median_ignores_zero_and_far_values(self):
        import numpy as np

        depth = np.array(
            [
                [0, 990, 1000],
                [1010, 65000, 1020],
                [980, 970, 0],
            ],
            dtype=np.uint16,
        )

        self.assertEqual(
            sample_depth_median_mm(depth, center=(1, 1), radius=1, min_mm=100, max_mm=5000),
            995.0,
        )

    def test_sample_depth_median_returns_none_without_valid_depth(self):
        import numpy as np

        depth = np.zeros((5, 5), dtype=np.uint16)

        self.assertIsNone(sample_depth_median_mm(depth, center=(2, 2), radius=1))

    def test_scale_point_maps_color_pixel_to_depth_size(self):
        self.assertEqual(
            scale_point((320, 240), source_size=(640, 480), target_size=(320, 240)),
            (160, 120),
        )

    def test_deproject_pixel_uses_pinhole_model(self):
        intr = CameraIntrinsics(fx=600.0, fy=600.0, ppx=320.0, ppy=240.0)

        x, y, z = deproject_pixel(380, 300, 1.2, intr)

        self.assertAlmostEqual(x, 0.12)
        self.assertAlmostEqual(y, 0.12)
        self.assertAlmostEqual(z, 1.2)

    def test_build_output_payload_marks_valid_3d_detection(self):
        detection = Detection(
            label="bottle",
            confidence=0.91,
            xyxy=(300.0, 10.0, 420.0, 230.0),
            center=(360, 120),
        )
        intr = CameraIntrinsics(fx=600.0, fy=600.0, ppx=320.0, ppy=240.0)

        payload = build_output_payload(
            frame_index=12,
            timestamp_s=100.5,
            fps=3.2,
            detection=detection,
            depth_mm=1100.0,
            xyz_m=(0.073, -0.22, 1.1),
            intrinsics=intr,
            color_source="1",
            depth_source="/dev/video0",
        )

        self.assertTrue(payload["valid"])
        self.assertEqual(payload["frame_index"], 12)
        self.assertEqual(payload["detection"]["center_px"], {"cx": 360, "cy": 120})
        self.assertEqual(payload["depth"]["median_mm"], 1100.0)
        self.assertEqual(payload["camera_xyz_m"], {"x": 0.073, "y": -0.22, "z": 1.1})
        self.assertEqual(payload["fps"], 3.2)

    def test_build_output_payload_marks_missing_detection_invalid(self):
        payload = build_output_payload(
            frame_index=2,
            timestamp_s=101.0,
            fps=None,
            detection=None,
            depth_mm=None,
            xyz_m=None,
            intrinsics=CameraIntrinsics(fx=1.0, fy=1.0, ppx=0.0, ppy=0.0),
            color_source="1",
            depth_source="/dev/video0",
        )

        self.assertFalse(payload["valid"])
        self.assertIsNone(payload["detection"])
        self.assertIsNone(payload["depth"])
        self.assertIsNone(payload["camera_xyz_m"])

    def test_should_retry_reconnect_supports_unlimited_and_limited_modes(self):
        self.assertTrue(should_retry_reconnect(attempts=100, max_attempts=0))
        self.assertTrue(should_retry_reconnect(attempts=2, max_attempts=3))
        self.assertFalse(should_retry_reconnect(attempts=3, max_attempts=3))


class RecordBottlePositionsTests(unittest.TestCase):
    def test_csv_header_matches_flattened_payload_keys(self):
        payload = {
            "valid": True,
            "timestamp_s": 100.5,
            "frame_index": 12,
            "fps": 6.2,
            "color_source": "auto",
            "depth_source": "/dev/video0",
            "detection": {
                "label": "bottle",
                "confidence": 0.91,
                "center_px": {"cx": 360, "cy": 120},
                "xyxy": {"x1": 300.0, "y1": 10.0, "x2": 420.0, "y2": 230.0},
            },
            "depth": {"median_mm": 1100.0},
            "camera_xyz_m": {"x": 0.073, "y": -0.22, "z": 1.1},
        }

        row = flatten_payload(payload, read_time_s=101.0)

        self.assertEqual(list(row.keys()), csv_header())
        self.assertEqual(row["valid"], True)
        self.assertEqual(row["age_s"], 0.5)
        self.assertEqual(row["cx"], 360)
        self.assertEqual(row["camera_z_m"], 1.1)

    def test_flatten_payload_handles_invalid_detection(self):
        row = flatten_payload(
            {
                "valid": False,
                "timestamp_s": 100.0,
                "frame_index": 3,
                "fps": None,
                "color_source": "auto",
                "depth_source": "/dev/video0",
                "detection": None,
                "depth": None,
                "camera_xyz_m": None,
            },
            read_time_s=101.25,
        )

        self.assertFalse(row["valid"])
        self.assertEqual(row["frame_index"], 3)
        self.assertEqual(row["age_s"], 1.25)
        self.assertIsNone(row["cx"])
        self.assertIsNone(row["depth_mm"])
        self.assertIsNone(row["camera_x_m"])


class CompareBottleToTargetTests(unittest.TestCase):
    def test_build_offset_report_computes_camera_and_pixel_delta(self):
        reference = {
            "reference": {
                "center_px": {"cx": 168.0, "cy": 59.0},
                "depth_mm": 532.3,
                "camera_xyz_m": {"x": -0.14257, "y": -0.163469, "z": 0.5323},
            }
        }
        current = {
            "valid": True,
            "timestamp_s": 200.0,
            "frame_index": 33,
            "detection": {
                "label": "bottle",
                "confidence": 0.88,
                "center_px": {"cx": 178, "cy": 55},
            },
            "depth": {"median_mm": 542.3},
            "camera_xyz_m": {"x": -0.13257, "y": -0.166469, "z": 0.5423},
        }

        report = build_offset_report(
            current,
            reference,
            tolerance_m={"x": 0.03, "y": 0.03, "z": 0.04},
            tolerance_px={"cx": 30, "cy": 30},
        )

        self.assertTrue(report["valid"])
        self.assertTrue(report["within_tolerance"])
        self.assertEqual(report["status"], "ready")
        self.assertEqual(report["delta_px"], {"cx": 10.0, "cy": -4.0})
        self.assertAlmostEqual(report["delta_camera_m"]["x"], 0.01)
        self.assertAlmostEqual(report["delta_camera_m"]["y"], -0.003)
        self.assertAlmostEqual(report["delta_depth_mm"], 10.0)

    def test_build_offset_report_marks_invalid_current_payload_not_ready(self):
        reference = {
            "reference": {
                "center_px": {"cx": 168.0, "cy": 59.0},
                "depth_mm": 532.3,
                "camera_xyz_m": {"x": -0.14257, "y": -0.163469, "z": 0.5323},
            }
        }

        report = build_offset_report({"valid": False, "detection": None}, reference)

        self.assertFalse(report["valid"])
        self.assertFalse(report["within_tolerance"])
        self.assertEqual(report["status"], "invalid_current")

    def test_build_offset_report_marks_out_of_range_delta_not_ready(self):
        reference = {
            "reference": {
                "center_px": {"cx": 168.0, "cy": 59.0},
                "depth_mm": 532.3,
                "camera_xyz_m": {"x": -0.14257, "y": -0.163469, "z": 0.5323},
            }
        }
        current = {
            "valid": True,
            "detection": {
                "label": "bottle",
                "confidence": 0.91,
                "center_px": {"cx": 220, "cy": 59},
            },
            "depth": {"median_mm": 532.3},
            "camera_xyz_m": {"x": -0.08257, "y": -0.163469, "z": 0.5323},
        }

        report = build_offset_report(
            current,
            reference,
            tolerance_m={"x": 0.03, "y": 0.03, "z": 0.04},
            tolerance_px={"cx": 30, "cy": 30},
        )

        self.assertTrue(report["valid"])
        self.assertFalse(report["within_tolerance"])
        self.assertEqual(report["status"], "out_of_range")
        self.assertEqual(report["failed_axes"], ["x", "cx"])


class EvaluateGraspWindowTests(unittest.TestCase):
    def test_summarize_rows_averages_valid_rows_and_tracks_ranges(self):
        rows = [
            {
                "valid": "True",
                "confidence": "0.8",
                "cx": "168",
                "cy": "59",
                "depth_mm": "532",
                "camera_x_m": "-0.142",
                "camera_y_m": "-0.163",
                "camera_z_m": "0.532",
            },
            {
                "valid": "False",
                "confidence": "",
                "cx": "",
                "cy": "",
                "depth_mm": "",
                "camera_x_m": "",
                "camera_y_m": "",
                "camera_z_m": "",
            },
            {
                "valid": "True",
                "confidence": "0.9",
                "cx": "170",
                "cy": "57",
                "depth_mm": "536",
                "camera_x_m": "-0.140",
                "camera_y_m": "-0.165",
                "camera_z_m": "0.536",
            },
        ]

        summary = summarize_rows(rows, label="sample")

        self.assertEqual(summary["label"], "sample")
        self.assertEqual(summary["sample_count"], 3)
        self.assertEqual(summary["valid_count"], 2)
        self.assertEqual(summary["invalid_count"], 1)
        self.assertEqual(summary["mean"]["cx"], 169.0)
        self.assertEqual(summary["mean"]["depth_mm"], 534.0)
        self.assertEqual(summary["range"]["cy"], 2.0)

    def test_build_record_report_classifies_average_point_against_reference(self):
        reference = {
            "reference": {
                "center_px": {"cx": 168.0, "cy": 59.0},
                "depth_mm": 532.3,
                "camera_xyz_m": {"x": -0.14257, "y": -0.163469, "z": 0.5323},
            }
        }
        summary = {
            "label": "random_far",
            "sample_count": 20,
            "valid_count": 20,
            "invalid_count": 0,
            "mean": {
                "confidence": 0.78,
                "cx": 191.0,
                "cy": 43.6,
                "depth_mm": 647.7,
                "camera_x_m": -0.1489,
                "camera_y_m": -0.2154,
                "camera_z_m": 0.6477,
            },
            "range": {},
        }

        report = build_record_report(
            summary,
            reference,
            tolerance_m={"x": 0.04, "y": 0.04, "z": 0.06},
            tolerance_px={"cx": 45, "cy": 45},
        )

        self.assertEqual(report["label"], "random_far")
        self.assertEqual(report["status"], "out_of_range")
        self.assertEqual(report["failed_axes"], ["y", "z"])
        self.assertAlmostEqual(report["delta_camera_m"]["z"], 0.1154)


class AutoGraspDecisionTests(unittest.TestCase):
    def test_build_decision_returns_ready_for_small_target_offset(self):
        offset_report = {
            "valid": True,
            "status": "ready",
            "delta_camera_m": {"x": 0.002, "y": -0.003, "z": 0.004},
            "delta_px": {"cx": 2.0, "cy": -3.0},
            "failed_axes": [],
        }

        decision = build_decision(offset_report)

        self.assertEqual(decision["decision"], "READY")
        self.assertEqual(decision["risk_level"], "low")
        self.assertEqual(decision["exceeded_ready_axes"], [])
        self.assertEqual(decision["exceeded_correction_axes"], [])

    def test_build_decision_returns_needs_correction_inside_outer_window(self):
        offset_report = {
            "valid": True,
            "status": "ready",
            "delta_camera_m": {"x": -0.0012, "y": -0.0280, "z": 0.0576},
            "delta_px": {"cx": 14.65, "cy": -10.55},
            "failed_axes": [],
        }

        decision = build_decision(offset_report)

        self.assertEqual(decision["decision"], "NEEDS_CORRECTION")
        self.assertEqual(decision["risk_level"], "medium")
        self.assertEqual(decision["exceeded_correction_axes"], [])
        self.assertIn("z", decision["exceeded_ready_axes"])

    def test_build_decision_rejects_large_offset(self):
        offset_report = {
            "valid": True,
            "status": "out_of_range",
            "delta_camera_m": {"x": -0.0063, "y": -0.0519, "z": 0.1154},
            "delta_px": {"cx": 23.0, "cy": -15.4},
            "failed_axes": ["y", "z"],
        }

        decision = build_decision(offset_report)

        self.assertEqual(decision["decision"], "REJECT")
        self.assertEqual(decision["risk_level"], "high")
        self.assertEqual(decision["exceeded_correction_axes"], ["y", "z"])

    def test_build_decision_rejects_invalid_detection(self):
        decision = build_decision({"valid": False, "status": "invalid_current"})

        self.assertEqual(decision["decision"], "REJECT")
        self.assertEqual(decision["reason"], "invalid_current")


if __name__ == "__main__":
    unittest.main()
