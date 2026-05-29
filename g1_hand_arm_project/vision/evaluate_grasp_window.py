#!/usr/bin/env python3
"""
Evaluate recorded bottle positions against the target grasp window.

This script is vision-data analysis only. It reads CSV/JSON files and reports
whether each recorded bottle position is inside the current task-level grasp
window. It never controls the robot arm or Revo2 hand.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, Iterable, List, Optional

from compare_bottle_to_target import (
    DEFAULT_REFERENCE,
    DEFAULT_TOLERANCE_M,
    DEFAULT_TOLERANCE_PX,
    build_offset_report,
    read_json,
    write_json_atomic,
)


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_RECORDS_DIR = SCRIPT_DIR / "records"
DEFAULT_REPORT_OUT = SCRIPT_DIR / "grasp_window_report.json"
NUMERIC_FIELDS = [
    "confidence",
    "cx",
    "cy",
    "depth_mm",
    "camera_x_m",
    "camera_y_m",
    "camera_z_m",
    "fps",
]


def _round_or_none(value, digits: int = 6):
    if value is None:
        return None
    return round(float(value), int(digits))


def _truthy(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def _to_float(value) -> Optional[float]:
    if value is None or value == "":
        return None
    return float(value)


def _mean(values: List[float]) -> Optional[float]:
    if not values:
        return None
    return sum(values) / float(len(values))


def _value_range(values: List[float]) -> Optional[float]:
    if not values:
        return None
    return max(values) - min(values)


def summarize_rows(rows: Iterable[Dict], label: str):
    rows = list(rows)
    valid_rows = [row for row in rows if _truthy(row.get("valid", False))]

    values = {field: [] for field in NUMERIC_FIELDS}
    for row in valid_rows:
        for field in NUMERIC_FIELDS:
            value = _to_float(row.get(field))
            if value is not None:
                values[field].append(value)

    return {
        "label": label,
        "sample_count": len(rows),
        "valid_count": len(valid_rows),
        "invalid_count": len(rows) - len(valid_rows),
        "mean": {field: _round_or_none(_mean(vals)) for field, vals in values.items()},
        "range": {field: _round_or_none(_value_range(vals)) for field, vals in values.items()},
    }


def summarize_csv(path: Path):
    path = Path(path)
    with path.open("r", newline="", encoding="utf-8") as f:
        return summarize_rows(csv.DictReader(f), label=path.stem)


def _summary_to_current_payload(summary: Dict) -> Dict:
    mean = summary.get("mean") or {}
    has_point = all(
        mean.get(field) is not None
        for field in ("cx", "cy", "depth_mm", "camera_x_m", "camera_y_m", "camera_z_m")
    )
    if not has_point:
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


def build_record_report(
    summary: Dict,
    reference_payload: Dict,
    tolerance_m: Optional[Dict[str, float]] = None,
    tolerance_px: Optional[Dict[str, float]] = None,
) -> Dict:
    offset = build_offset_report(
        _summary_to_current_payload(summary),
        reference_payload,
        tolerance_m=tolerance_m,
        tolerance_px=tolerance_px,
    )
    return {
        "label": summary["label"],
        "sample_count": summary["sample_count"],
        "valid_count": summary["valid_count"],
        "invalid_count": summary["invalid_count"],
        "mean": summary["mean"],
        "range": summary["range"],
        "status": offset.get("status"),
        "within_tolerance": bool(offset.get("within_tolerance", False)),
        "failed_axes": offset.get("failed_axes", []),
        "delta_camera_m": offset.get("delta_camera_m"),
        "delta_px": offset.get("delta_px"),
        "delta_depth_mm": offset.get("delta_depth_mm"),
        "message": offset.get("message"),
    }


def resolve_record_paths(records_dir: Path, patterns: List[str]) -> List[Path]:
    paths: List[Path] = []
    for pattern in patterns:
        candidate = Path(pattern)
        if candidate.exists():
            paths.append(candidate)
            continue
        base = Path(records_dir)
        matches = sorted(base.glob(pattern))
        paths.extend(matches)

    seen = set()
    unique = []
    for path in paths:
        resolved = str(path.resolve())
        if resolved not in seen and path.is_file():
            seen.add(resolved)
            unique.append(path)
    return unique


def build_reports(record_paths: Iterable[Path], reference_payload: Dict, tolerance_m, tolerance_px):
    reports = []
    for path in record_paths:
        summary = summarize_csv(path)
        reports.append(build_record_report(summary, reference_payload, tolerance_m, tolerance_px))
    return reports


def format_report_table(reports: Iterable[Dict]) -> str:
    lines = [
        "label,status,valid,dcx_px,dcy_px,dx_m,dy_m,dz_m,ddepth_mm,failed_axes",
    ]
    for report in reports:
        dpx = report.get("delta_px") or {}
        dm = report.get("delta_camera_m") or {}
        failed_axes = "|".join(report.get("failed_axes") or [])
        lines.append(
            "{label},{status},{valid},{dcx},{dcy},{dx},{dy},{dz},{ddepth},{failed}".format(
                label=report.get("label"),
                status=report.get("status"),
                valid=report.get("valid_count"),
                dcx=_round_or_none(dpx.get("cx"), 3),
                dcy=_round_or_none(dpx.get("cy"), 3),
                dx=_round_or_none(dm.get("x")),
                dy=_round_or_none(dm.get("y")),
                dz=_round_or_none(dm.get("z")),
                ddepth=_round_or_none(report.get("delta_depth_mm"), 3),
                failed=failed_axes or "none",
            )
        )
    return "\n".join(lines)


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Evaluate recorded bottle CSV files against the target grasp window."
    )
    parser.add_argument(
        "records",
        nargs="*",
        help="CSV files or glob patterns under --records-dir. Defaults to target_grasp_*.csv and random*.csv.",
    )
    parser.add_argument("--records-dir", default=str(DEFAULT_RECORDS_DIR))
    parser.add_argument("--reference", default=str(DEFAULT_REFERENCE))
    parser.add_argument("--json-out", default=str(DEFAULT_REPORT_OUT))
    parser.add_argument("--no-json-out", action="store_true")
    parser.add_argument("--x-tol-m", type=float, default=DEFAULT_TOLERANCE_M["x"])
    parser.add_argument("--y-tol-m", type=float, default=DEFAULT_TOLERANCE_M["y"])
    parser.add_argument("--z-tol-m", type=float, default=DEFAULT_TOLERANCE_M["z"])
    parser.add_argument("--cx-tol-px", type=float, default=DEFAULT_TOLERANCE_PX["cx"])
    parser.add_argument("--cy-tol-px", type=float, default=DEFAULT_TOLERANCE_PX["cy"])
    parser.add_argument("--print-json", action="store_true")
    return parser


def run(args) -> int:
    patterns = args.records or ["target_grasp_*.csv", "random*.csv"]
    record_paths = resolve_record_paths(Path(args.records_dir), patterns)
    if not record_paths:
        print(f"ERROR: no record CSV files matched: {patterns}")
        return 2

    reference = read_json(Path(args.reference))
    tolerance_m = {"x": args.x_tol_m, "y": args.y_tol_m, "z": args.z_tol_m}
    tolerance_px = {"cx": args.cx_tol_px, "cy": args.cy_tol_px}
    reports = build_reports(record_paths, reference, tolerance_m, tolerance_px)
    payload = {
        "reference": str(args.reference),
        "records_dir": str(args.records_dir),
        "tolerance_m": tolerance_m,
        "tolerance_px": tolerance_px,
        "reports": reports,
    }

    if not args.no_json_out:
        write_json_atomic(Path(args.json_out), payload)

    if args.print_json:
        print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print(format_report_table(reports))
    return 0


def main():
    return run(build_arg_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
