#!/usr/bin/env python3
"""
Compare the latest bottle 3D detection with a recorded target grasp reference.

This script is vision-only. It reads JSON files, computes camera-frame and pixel
offsets, and reports whether the bottle is inside a conservative task window.
It never controls the robot arm or Revo2 hand.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Dict, Optional


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_LATEST = SCRIPT_DIR / "latest_bottle_3d.json"
DEFAULT_REFERENCE = SCRIPT_DIR / "target_grasp_reference.json"
DEFAULT_REPORT_OUT = SCRIPT_DIR / "latest_target_offset.json"

DEFAULT_TOLERANCE_M = {"x": 0.04, "y": 0.04, "z": 0.06}
DEFAULT_TOLERANCE_PX = {"cx": 45.0, "cy": 45.0}


def _round(value, digits: int = 6):
    return round(float(value), int(digits))


def read_json(path: Path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json_atomic(path: Path, payload) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(path.name + ".tmp")
    tmp_path.write_text(
        json.dumps(payload, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    tmp_path.replace(path)


def _reference_section(reference_payload: Dict) -> Dict:
    return reference_payload.get("reference") or reference_payload


def extract_point(payload: Dict, *, require_valid: bool) -> Optional[Dict]:
    if require_valid and not bool(payload.get("valid", False)):
        return None

    detection = payload.get("detection") or {}
    center = detection.get("center_px") or payload.get("center_px") or {}
    depth = payload.get("depth") or {}
    xyz = payload.get("camera_xyz_m") or {}

    depth_mm = depth.get("median_mm", payload.get("depth_mm"))
    required = [center.get("cx"), center.get("cy"), depth_mm, xyz.get("x"), xyz.get("y"), xyz.get("z")]
    if any(value is None for value in required):
        return None

    return {
        "cx": _round(center["cx"], 3),
        "cy": _round(center["cy"], 3),
        "depth_mm": _round(depth_mm, 3),
        "camera_x_m": _round(xyz["x"]),
        "camera_y_m": _round(xyz["y"]),
        "camera_z_m": _round(xyz["z"]),
        "confidence": None if detection.get("confidence") is None else _round(detection["confidence"]),
        "label": detection.get("label", payload.get("label")),
    }


def build_offset_report(
    current_payload: Dict,
    reference_payload: Dict,
    tolerance_m: Optional[Dict[str, float]] = None,
    tolerance_px: Optional[Dict[str, float]] = None,
) -> Dict:
    tolerance_m = dict(DEFAULT_TOLERANCE_M if tolerance_m is None else tolerance_m)
    tolerance_px = dict(DEFAULT_TOLERANCE_PX if tolerance_px is None else tolerance_px)

    reference_point = extract_point(_reference_section(reference_payload), require_valid=False)
    if reference_point is None:
        return {
            "valid": False,
            "within_tolerance": False,
            "status": "invalid_reference",
            "message": "target reference is missing center/depth/camera_xyz fields",
        }

    current_point = extract_point(current_payload, require_valid=True)
    if current_point is None:
        return {
            "valid": False,
            "within_tolerance": False,
            "status": "invalid_current",
            "message": "current detection is invalid or missing center/depth/camera_xyz fields",
            "reference": reference_point,
        }

    delta_camera_m = {
        "x": _round(current_point["camera_x_m"] - reference_point["camera_x_m"]),
        "y": _round(current_point["camera_y_m"] - reference_point["camera_y_m"]),
        "z": _round(current_point["camera_z_m"] - reference_point["camera_z_m"]),
    }
    delta_px = {
        "cx": _round(current_point["cx"] - reference_point["cx"], 3),
        "cy": _round(current_point["cy"] - reference_point["cy"], 3),
    }
    delta_depth_mm = _round(current_point["depth_mm"] - reference_point["depth_mm"], 3)

    failed_axes = []
    for axis in ("x", "y", "z"):
        if abs(delta_camera_m[axis]) > float(tolerance_m[axis]):
            failed_axes.append(axis)
    for axis in ("cx", "cy"):
        if abs(delta_px[axis]) > float(tolerance_px[axis]):
            failed_axes.append(axis)

    within_tolerance = len(failed_axes) == 0
    status = "ready" if within_tolerance else "out_of_range"
    message = (
        "bottle is inside the task-level target window"
        if within_tolerance
        else "bottle is outside the task-level target window"
    )

    return {
        "valid": True,
        "within_tolerance": within_tolerance,
        "status": status,
        "message": message,
        "failed_axes": failed_axes,
        "reference": reference_point,
        "current": current_point,
        "delta_camera_m": delta_camera_m,
        "delta_px": delta_px,
        "delta_depth_mm": delta_depth_mm,
        "tolerance_m": {key: _round(value) for key, value in tolerance_m.items()},
        "tolerance_px": {key: _round(value, 3) for key, value in tolerance_px.items()},
        "timestamp_s": _round(time.time()),
        "frame_index": current_payload.get("frame_index"),
        "note": "camera-frame offset only; no arm control and no hand-eye transform applied",
    }


def format_report(report: Dict) -> str:
    if not report.get("valid"):
        return f"{report.get('status')}: {report.get('message')}"

    delta = report["delta_camera_m"]
    dpx = report["delta_px"]
    current = report["current"]
    failed = ",".join(report["failed_axes"]) if report["failed_axes"] else "none"
    return (
        f"status={report['status']} within_tolerance={report['within_tolerance']} "
        f"delta_m=({delta['x']:+.4f},{delta['y']:+.4f},{delta['z']:+.4f}) "
        f"delta_px=({dpx['cx']:+.1f},{dpx['cy']:+.1f}) "
        f"depth={current['depth_mm']:.1f}mm conf={current['confidence']} failed={failed}"
    )


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Compare latest_bottle_3d.json with target_grasp_reference.json."
    )
    parser.add_argument("--latest", default=str(DEFAULT_LATEST), help="Current latest_bottle_3d.json")
    parser.add_argument("--reference", default=str(DEFAULT_REFERENCE), help="Target reference JSON")
    parser.add_argument("--json-out", default=str(DEFAULT_REPORT_OUT), help="Write offset report JSON")
    parser.add_argument("--no-json-out", action="store_true", help="Do not write report JSON")
    parser.add_argument("--x-tol-m", type=float, default=DEFAULT_TOLERANCE_M["x"])
    parser.add_argument("--y-tol-m", type=float, default=DEFAULT_TOLERANCE_M["y"])
    parser.add_argument("--z-tol-m", type=float, default=DEFAULT_TOLERANCE_M["z"])
    parser.add_argument("--cx-tol-px", type=float, default=DEFAULT_TOLERANCE_PX["cx"])
    parser.add_argument("--cy-tol-px", type=float, default=DEFAULT_TOLERANCE_PX["cy"])
    parser.add_argument("--print-json", action="store_true", help="Print full JSON report")
    return parser


def run(args) -> int:
    current = read_json(Path(args.latest))
    reference = read_json(Path(args.reference))
    report = build_offset_report(
        current,
        reference,
        tolerance_m={"x": args.x_tol_m, "y": args.y_tol_m, "z": args.z_tol_m},
        tolerance_px={"cx": args.cx_tol_px, "cy": args.cy_tol_px},
    )

    if not args.no_json_out:
        write_json_atomic(Path(args.json_out), report)

    if args.print_json:
        print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print(format_report(report))
    return 0 if report.get("valid") else 1


def main():
    return run(build_arg_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
