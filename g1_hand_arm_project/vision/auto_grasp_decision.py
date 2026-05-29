#!/usr/bin/env python3
"""
Decide whether the current bottle position is ready for the next grasp stage.

This script is a decision gate only. It reads vision data, compares it with the
target grasp reference, and prints READY / NEEDS_CORRECTION / REJECT. It never
controls the robot arm or Revo2 hand.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, Optional

from compare_bottle_to_target import (
    DEFAULT_LATEST,
    DEFAULT_REFERENCE,
    build_offset_report,
    read_json,
    write_json_atomic,
)
from evaluate_grasp_window import summarize_csv


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DECISION_OUT = SCRIPT_DIR / "latest_grasp_decision.json"

READY_TOLERANCE_M = {"x": 0.015, "y": 0.020, "z": 0.030}
READY_TOLERANCE_PX = {"cx": 12.0, "cy": 12.0}
CORRECTION_TOLERANCE_M = {"x": 0.040, "y": 0.040, "z": 0.080}
CORRECTION_TOLERANCE_PX = {"cx": 45.0, "cy": 45.0}


def _round(value, digits: int = 6):
    return round(float(value), int(digits))


def _exceeded_axes(delta_m: Dict, delta_px: Dict, tolerance_m: Dict, tolerance_px: Dict):
    failed = []
    for axis in ("x", "y", "z"):
        if abs(float(delta_m.get(axis, 0.0))) > float(tolerance_m[axis]):
            failed.append(axis)
    for axis in ("cx", "cy"):
        if abs(float(delta_px.get(axis, 0.0))) > float(tolerance_px[axis]):
            failed.append(axis)
    return failed


def _summary_to_payload(summary: Dict) -> Dict:
    mean = summary.get("mean") or {}
    required = ("cx", "cy", "depth_mm", "camera_x_m", "camera_y_m", "camera_z_m")
    if any(mean.get(field) is None for field in required):
        return {"valid": False, "detection": None, "depth": None, "camera_xyz_m": None}

    return {
        "valid": True,
        "detection": {
            "label": "bottle",
            "confidence": mean.get("confidence"),
            "center_px": {"cx": mean["cx"], "cy": mean["cy"]},
        },
        "depth": {"median_mm": mean["depth_mm"]},
        "camera_xyz_m": {
            "x": mean["camera_x_m"],
            "y": mean["camera_y_m"],
            "z": mean["camera_z_m"],
        },
    }


def build_decision(
    offset_report: Dict,
    ready_tolerance_m: Optional[Dict[str, float]] = None,
    ready_tolerance_px: Optional[Dict[str, float]] = None,
    correction_tolerance_m: Optional[Dict[str, float]] = None,
    correction_tolerance_px: Optional[Dict[str, float]] = None,
) -> Dict:
    ready_tolerance_m = dict(READY_TOLERANCE_M if ready_tolerance_m is None else ready_tolerance_m)
    ready_tolerance_px = dict(READY_TOLERANCE_PX if ready_tolerance_px is None else ready_tolerance_px)
    correction_tolerance_m = dict(
        CORRECTION_TOLERANCE_M if correction_tolerance_m is None else correction_tolerance_m
    )
    correction_tolerance_px = dict(
        CORRECTION_TOLERANCE_PX if correction_tolerance_px is None else correction_tolerance_px
    )

    if not offset_report.get("valid", False):
        status = offset_report.get("status", "invalid_current")
        return {
            "decision": "REJECT",
            "risk_level": "high",
            "reason": status,
            "message": "current vision result is invalid; do not start grasp",
            "exceeded_ready_axes": [],
            "exceeded_correction_axes": [],
            "arm_control": "disabled",
            "offset": offset_report,
        }

    delta_m = offset_report.get("delta_camera_m") or {}
    delta_px = offset_report.get("delta_px") or {}
    exceeded_ready = _exceeded_axes(delta_m, delta_px, ready_tolerance_m, ready_tolerance_px)
    exceeded_correction = _exceeded_axes(
        delta_m,
        delta_px,
        correction_tolerance_m,
        correction_tolerance_px,
    )

    if exceeded_correction:
        decision = "REJECT"
        risk_level = "high"
        reason = "outside_correction_window"
        message = "bottle offset is too large for the current automated grasp stage"
        next_step = "ask operator to move the bottle closer to the target or use a higher-level planner"
    elif exceeded_ready:
        decision = "NEEDS_CORRECTION"
        risk_level = "medium"
        reason = "inside_correction_window"
        message = "bottle is close enough for a small correction, but not for direct fixed-trajectory grasp"
        next_step = "compute a small correction after arm-side mapping is validated"
    else:
        decision = "READY"
        risk_level = "low"
        reason = "inside_ready_window"
        message = "bottle is close to the recorded target grasp point"
        next_step = "fixed-trajectory grasp can be considered after explicit safety confirmation"

    return {
        "decision": decision,
        "risk_level": risk_level,
        "reason": reason,
        "message": message,
        "next_step": next_step,
        "exceeded_ready_axes": exceeded_ready,
        "exceeded_correction_axes": exceeded_correction,
        "ready_tolerance_m": {key: _round(value) for key, value in ready_tolerance_m.items()},
        "ready_tolerance_px": {key: _round(value, 3) for key, value in ready_tolerance_px.items()},
        "correction_tolerance_m": {
            key: _round(value) for key, value in correction_tolerance_m.items()
        },
        "correction_tolerance_px": {
            key: _round(value, 3) for key, value in correction_tolerance_px.items()
        },
        "arm_control": "disabled",
        "offset": offset_report,
    }


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Decide READY / NEEDS_CORRECTION / REJECT from bottle vision offset."
    )
    parser.add_argument("--latest", default=str(DEFAULT_LATEST), help="Current latest_bottle_3d.json")
    parser.add_argument("--record-csv", default="", help="Use averaged CSV record instead of --latest")
    parser.add_argument("--reference", default=str(DEFAULT_REFERENCE), help="Target reference JSON")
    parser.add_argument("--json-out", default=str(DEFAULT_DECISION_OUT), help="Write decision JSON")
    parser.add_argument("--no-json-out", action="store_true")
    parser.add_argument("--print-json", action="store_true")
    parser.add_argument("--ready-x-tol-m", type=float, default=READY_TOLERANCE_M["x"])
    parser.add_argument("--ready-y-tol-m", type=float, default=READY_TOLERANCE_M["y"])
    parser.add_argument("--ready-z-tol-m", type=float, default=READY_TOLERANCE_M["z"])
    parser.add_argument("--ready-cx-tol-px", type=float, default=READY_TOLERANCE_PX["cx"])
    parser.add_argument("--ready-cy-tol-px", type=float, default=READY_TOLERANCE_PX["cy"])
    parser.add_argument("--correction-x-tol-m", type=float, default=CORRECTION_TOLERANCE_M["x"])
    parser.add_argument("--correction-y-tol-m", type=float, default=CORRECTION_TOLERANCE_M["y"])
    parser.add_argument("--correction-z-tol-m", type=float, default=CORRECTION_TOLERANCE_M["z"])
    parser.add_argument("--correction-cx-tol-px", type=float, default=CORRECTION_TOLERANCE_PX["cx"])
    parser.add_argument("--correction-cy-tol-px", type=float, default=CORRECTION_TOLERANCE_PX["cy"])
    return parser


def format_decision(decision: Dict) -> str:
    offset = decision.get("offset") or {}
    delta = offset.get("delta_camera_m") or {}
    dpx = offset.get("delta_px") or {}
    return (
        f"decision={decision['decision']} risk={decision['risk_level']} "
        f"reason={decision['reason']} "
        f"delta_m=({delta.get('x', 0):+.4f},{delta.get('y', 0):+.4f},{delta.get('z', 0):+.4f}) "
        f"delta_px=({dpx.get('cx', 0):+.1f},{dpx.get('cy', 0):+.1f}) "
        f"arm_control={decision['arm_control']}"
    )


def run(args) -> int:
    reference = read_json(Path(args.reference))
    if args.record_csv:
        current = _summary_to_payload(summarize_csv(Path(args.record_csv)))
    else:
        current = read_json(Path(args.latest))

    correction_tolerance_m = {
        "x": args.correction_x_tol_m,
        "y": args.correction_y_tol_m,
        "z": args.correction_z_tol_m,
    }
    correction_tolerance_px = {
        "cx": args.correction_cx_tol_px,
        "cy": args.correction_cy_tol_px,
    }
    offset = build_offset_report(
        current,
        reference,
        tolerance_m=correction_tolerance_m,
        tolerance_px=correction_tolerance_px,
    )
    decision = build_decision(
        offset,
        ready_tolerance_m={
            "x": args.ready_x_tol_m,
            "y": args.ready_y_tol_m,
            "z": args.ready_z_tol_m,
        },
        ready_tolerance_px={
            "cx": args.ready_cx_tol_px,
            "cy": args.ready_cy_tol_px,
        },
        correction_tolerance_m=correction_tolerance_m,
        correction_tolerance_px=correction_tolerance_px,
    )

    if not args.no_json_out:
        write_json_atomic(Path(args.json_out), decision)

    if args.print_json:
        print(json.dumps(decision, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print(format_decision(decision))
    return 0


def main():
    return run(build_arg_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
