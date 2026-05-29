#!/usr/bin/env python3
"""
Record latest bottle 2D/3D detections from latest_bottle_3d.json into CSV.

This script is vision-data logging only. It does not open cameras, control the
robot arm, or control the Revo2 hand.
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path
from typing import Dict, Optional


DEFAULT_INPUT = Path(__file__).resolve().parent / "latest_bottle_3d.json"
DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "records"


def csv_header():
    return [
        "read_time_s",
        "timestamp_s",
        "age_s",
        "frame_index",
        "valid",
        "label",
        "confidence",
        "cx",
        "cy",
        "x1",
        "y1",
        "x2",
        "y2",
        "depth_mm",
        "camera_x_m",
        "camera_y_m",
        "camera_z_m",
        "fps",
        "color_source",
        "depth_source",
    ]


def _round_or_none(value, digits: int = 6):
    if value is None:
        return None
    return round(float(value), int(digits))


def flatten_payload(payload: Dict, read_time_s: Optional[float] = None):
    read_time = float(time.time() if read_time_s is None else read_time_s)
    timestamp = payload.get("timestamp_s")
    age_s = None if timestamp is None else read_time - float(timestamp)

    detection = payload.get("detection") or {}
    center = detection.get("center_px") or {}
    xyxy = detection.get("xyxy") or {}
    depth = payload.get("depth") or {}
    xyz = payload.get("camera_xyz_m") or {}

    return {
        "read_time_s": _round_or_none(read_time),
        "timestamp_s": _round_or_none(timestamp),
        "age_s": _round_or_none(age_s),
        "frame_index": payload.get("frame_index"),
        "valid": bool(payload.get("valid", False)),
        "label": detection.get("label"),
        "confidence": _round_or_none(detection.get("confidence")),
        "cx": center.get("cx"),
        "cy": center.get("cy"),
        "x1": _round_or_none(xyxy.get("x1"), 3),
        "y1": _round_or_none(xyxy.get("y1"), 3),
        "x2": _round_or_none(xyxy.get("x2"), 3),
        "y2": _round_or_none(xyxy.get("y2"), 3),
        "depth_mm": _round_or_none(depth.get("median_mm"), 3),
        "camera_x_m": _round_or_none(xyz.get("x")),
        "camera_y_m": _round_or_none(xyz.get("y")),
        "camera_z_m": _round_or_none(xyz.get("z")),
        "fps": _round_or_none(payload.get("fps"), 3),
        "color_source": payload.get("color_source"),
        "depth_source": payload.get("depth_source"),
    }


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def default_output_path(out_dir: Path):
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path(out_dir) / f"bottle_positions_{stamp}.csv"


def append_row(path: Path, row: Dict):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    needs_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=csv_header())
        if needs_header:
            writer.writeheader()
        writer.writerow(row)


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Record latest_bottle_3d.json into a timestamped CSV file."
    )
    parser.add_argument("--input", default=str(DEFAULT_INPUT), help="latest_bottle_3d.json path")
    parser.add_argument("--out", default="", help="CSV output path")
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR), help="Default output directory")
    parser.add_argument("--interval", type=float, default=0.5, help="Sample interval in seconds")
    parser.add_argument("--duration", type=float, default=0.0, help="Run duration. 0 means forever")
    parser.add_argument("--include-invalid", action="store_true", help="Record invalid rows too")
    parser.add_argument("--dedupe", action="store_true", help="Skip rows with repeated frame_index")
    return parser


def run(args) -> int:
    input_path = Path(args.input)
    out_path = Path(args.out) if args.out else default_output_path(Path(args.out_dir))
    interval = max(0.05, float(args.interval))
    duration = max(0.0, float(args.duration))
    deadline = None if duration <= 0 else time.time() + duration
    last_frame_index = None
    count = 0

    print(f"input: {input_path}")
    print(f"output: {out_path}")
    print("press Ctrl+C to stop")

    try:
        while True:
            now = time.time()
            if deadline is not None and now >= deadline:
                break

            try:
                payload = read_json(input_path)
                row = flatten_payload(payload, read_time_s=now)
            except FileNotFoundError:
                print(f"waiting for input file: {input_path}")
                time.sleep(interval)
                continue
            except Exception as exc:
                print(f"WARN: failed to read input: {exc}")
                time.sleep(interval)
                continue

            if args.dedupe and row["frame_index"] == last_frame_index:
                time.sleep(interval)
                continue
            last_frame_index = row["frame_index"]

            if row["valid"] or args.include_invalid:
                append_row(out_path, row)
                count += 1
                print(
                    f"recorded #{count}: valid={row['valid']} "
                    f"cx={row['cx']} cy={row['cy']} depth_mm={row['depth_mm']} "
                    f"xyz=({row['camera_x_m']},{row['camera_y_m']},{row['camera_z_m']})"
                )

            time.sleep(interval)

    except KeyboardInterrupt:
        print("stopped by user")

    print(f"wrote {count} rows to {out_path}")
    return 0


def main():
    return run(build_arg_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
