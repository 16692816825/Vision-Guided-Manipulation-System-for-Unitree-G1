#!/usr/bin/env python3
"""
Transform the latest bottle camera-frame point into the G1 base frame.

Input:
  vision/latest_bottle_3d.json from detect_bottle_depth.py

Output:
  vision/latest_bottle_base.json with base_xyz_m added.

This script is vision-data only. It does not control the robot arm or hand.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_INPUT = SCRIPT_DIR / "latest_bottle_3d.json"
DEFAULT_OUTPUT = SCRIPT_DIR / "latest_bottle_base.json"

# Latest fixed-head hand-eye result from the remeasured board points:
# O=(0.67, 0.21, 0.64), X=(0.69, -0.10, 0.64), Y=(0.23, 0.21, 0.64)
# T_base_camera maps camera optical-frame points into the robot base frame.
DEFAULT_T_BASE_CAMERA = np.array(
    [
        [-0.000781, -0.971795, 0.235827, 0.170172],
        [-0.999940, -0.001808, -0.010764, 0.008979],
        [0.010887, -0.235822, -0.971735, 1.230231],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=float,
)


def read_json(path: Path) -> Dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(path.name + ".tmp")
    tmp_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    tmp_path.replace(path)


def load_transform(path: Optional[str]) -> np.ndarray:
    if not path:
        return DEFAULT_T_BASE_CAMERA.copy()

    payload = read_json(Path(path))
    matrix = payload.get("T_base_camera", payload)
    arr = np.array(matrix, dtype=float)
    if arr.shape != (4, 4):
        raise ValueError(f"T_base_camera must be a 4x4 matrix, got shape={arr.shape}")
    return arr


def transform_point(T_base_camera: np.ndarray, camera_xyz: Dict[str, Any]) -> Dict[str, float]:
    p_camera = np.array(
        [
            float(camera_xyz["x"]),
            float(camera_xyz["y"]),
            float(camera_xyz["z"]),
            1.0,
        ],
        dtype=float,
    )
    p_base = T_base_camera @ p_camera
    return {
        "x": round(float(p_base[0]), 6),
        "y": round(float(p_base[1]), 6),
        "z": round(float(p_base[2]), 6),
    }


def matrix_to_list(T: np.ndarray) -> List[List[float]]:
    return [[round(float(value), 9) for value in row] for row in T.tolist()]


def build_payload(source: Dict[str, Any], T_base_camera: np.ndarray) -> Dict[str, Any]:
    valid = bool(source.get("valid", False))
    camera_xyz = source.get("camera_xyz_m")

    base_xyz = None
    reason = None
    if not valid:
        reason = "source_detection_invalid"
    elif not isinstance(camera_xyz, dict):
        reason = "missing_camera_xyz_m"
    else:
        try:
            base_xyz = transform_point(T_base_camera, camera_xyz)
        except Exception as exc:
            reason = f"transform_failed: {exc}"

    payload = dict(source)
    payload["base_transform"] = {
        "valid": base_xyz is not None,
        "reason": reason,
        "frame_from": "camera_color_optical_frame",
        "frame_to": "g1_base",
        "T_base_camera": matrix_to_list(T_base_camera),
        "base_xyz_m": base_xyz,
        "timestamp_s": round(time.time(), 6),
        "note": "vision transform only; no arm or hand command was sent",
    }
    return payload


def format_line(payload: Dict[str, Any]) -> str:
    detection = payload.get("detection") or {}
    center = detection.get("center_px") or {}
    camera_xyz = payload.get("camera_xyz_m") or {}
    base = (payload.get("base_transform") or {}).get("base_xyz_m")
    conf = detection.get("confidence")

    if base is None:
        reason = (payload.get("base_transform") or {}).get("reason")
        return f"INVALID base transform: {reason}"

    return (
        "bottle_base "
        f"cx={center.get('cx')} cy={center.get('cy')} "
        f"conf={conf} "
        f"camera=({camera_xyz.get('x')},{camera_xyz.get('y')},{camera_xyz.get('z')})m "
        f"base=({base['x']:.4f},{base['y']:.4f},{base['z']:.4f})m"
    )


def run_once(args: argparse.Namespace, T_base_camera: np.ndarray) -> int:
    source = read_json(Path(args.input))
    payload = build_payload(source, T_base_camera)

    if not args.no_json_out:
        write_json_atomic(Path(args.output), payload)

    if args.print_json:
        print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print(format_line(payload))

    return 0 if (payload.get("base_transform") or {}).get("valid") else 1


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert latest_bottle_3d.json camera XYZ into G1 base XYZ."
    )
    parser.add_argument("--input", default=str(DEFAULT_INPUT))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument(
        "--handeye-json",
        default="",
        help="Optional JSON containing T_base_camera as a 4x4 list. Defaults to the latest measured matrix.",
    )
    parser.add_argument("--watch", action="store_true", help="Continuously transform latest input")
    parser.add_argument("--interval", type=float, default=0.2, help="Watch interval in seconds")
    parser.add_argument("--no-json-out", action="store_true")
    parser.add_argument("--print-json", action="store_true")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    T_base_camera = load_transform(args.handeye_json)

    if not args.watch:
        return run_once(args, T_base_camera)

    print(f"watching: {args.input}")
    print(f"writing: {args.output}")
    print("press Ctrl+C to stop")
    last_frame_index = object()
    try:
        while True:
            try:
                source = read_json(Path(args.input))
                frame_index = source.get("frame_index")
                if frame_index != last_frame_index:
                    payload = build_payload(source, T_base_camera)
                    if not args.no_json_out:
                        write_json_atomic(Path(args.output), payload)
                    print(format_line(payload))
                    last_frame_index = frame_index
            except FileNotFoundError:
                print(f"waiting for input: {args.input}")
            except Exception as exc:
                print(f"WARN: {exc}")
            time.sleep(max(0.05, float(args.interval)))
    except KeyboardInterrupt:
        print("stopped by user")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
