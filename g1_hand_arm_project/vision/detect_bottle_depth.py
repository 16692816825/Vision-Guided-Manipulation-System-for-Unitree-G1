#!/usr/bin/env python3
"""
YOLOv8 bottle detection with lightweight RealSense depth sampling.

This script is vision-only. It reads camera frames, estimates the bottle center
and depth, prints an approximate camera-frame XYZ point, and optionally shows a
debug window. It never controls the robot arm or Revo2 hand.
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

from detect_bottle_2d import (
    DEFAULT_MODEL,
    Detection,
    build_arg_parser as build_2d_arg_parser,
    configure_low_latency_capture,
    draw_detection,
    normalize_target_names,
    open_camera,
    read_latest_frame,
    select_best_detection,
)


# RealSense D435I color intrinsics for 640x480 from rs-enumerate-devices -c.
DEFAULT_COLOR_FX = 605.888977050781
DEFAULT_COLOR_FY = 605.459533691406
DEFAULT_COLOR_PPX = 330.280029296875
DEFAULT_COLOR_PPY = 244.936431884766


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    ppx: float
    ppy: float


def sample_depth_median_mm(
    depth_mm: np.ndarray,
    center: Tuple[int, int],
    radius: int = 5,
    min_mm: int = 100,
    max_mm: int = 5000,
) -> Optional[float]:
    u, v = center
    h, w = depth_mm.shape[:2]
    x1 = max(0, int(u) - int(radius))
    x2 = min(w, int(u) + int(radius) + 1)
    y1 = max(0, int(v) - int(radius))
    y2 = min(h, int(v) + int(radius) + 1)
    if x1 >= x2 or y1 >= y2:
        return None

    roi = depth_mm[y1:y2, x1:x2]
    valid = roi[(roi >= int(min_mm)) & (roi <= int(max_mm))]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def scale_point(
    point: Tuple[int, int],
    source_size: Tuple[int, int],
    target_size: Tuple[int, int],
) -> Tuple[int, int]:
    u, v = point
    src_w, src_h = source_size
    dst_w, dst_h = target_size
    if src_w <= 0 or src_h <= 0:
        return int(u), int(v)
    return int(round(float(u) * float(dst_w) / float(src_w))), int(
        round(float(v) * float(dst_h) / float(src_h))
    )


def deproject_pixel(
    u: int,
    v: int,
    depth_m: float,
    intrinsics: CameraIntrinsics,
) -> Tuple[float, float, float]:
    z = float(depth_m)
    x = (float(u) - float(intrinsics.ppx)) * z / float(intrinsics.fx)
    y = (float(v) - float(intrinsics.ppy)) * z / float(intrinsics.fy)
    return x, y, z


def build_output_payload(
    frame_index: int,
    timestamp_s: float,
    fps: Optional[float],
    detection: Optional[Detection],
    depth_mm: Optional[float],
    xyz_m: Optional[Tuple[float, float, float]],
    intrinsics: CameraIntrinsics,
    color_source: str,
    depth_source: str,
):
    payload = {
        "valid": detection is not None and depth_mm is not None and xyz_m is not None,
        "timestamp_s": round(float(timestamp_s), 6),
        "frame_index": int(frame_index),
        "fps": None if fps is None else round(float(fps), 3),
        "color_source": str(color_source),
        "depth_source": str(depth_source),
        "depth_alignment": "direct_v4l2_depth_not_hardware_aligned_to_color",
        "intrinsics": {
            "fx": float(intrinsics.fx),
            "fy": float(intrinsics.fy),
            "ppx": float(intrinsics.ppx),
            "ppy": float(intrinsics.ppy),
        },
        "detection": None,
        "depth": None,
        "camera_xyz_m": None,
    }

    if detection is not None:
        x1, y1, x2, y2 = detection.xyxy
        cx, cy = detection.center
        payload["detection"] = {
            "label": detection.label,
            "confidence": round(float(detection.confidence), 6),
            "center_px": {"cx": int(cx), "cy": int(cy)},
            "xyxy": {
                "x1": round(float(x1), 3),
                "y1": round(float(y1), 3),
                "x2": round(float(x2), 3),
                "y2": round(float(y2), 3),
            },
        }

    if depth_mm is not None:
        payload["depth"] = {"median_mm": round(float(depth_mm), 3)}

    if xyz_m is not None:
        x, y, z = xyz_m
        payload["camera_xyz_m"] = {
            "x": round(float(x), 6),
            "y": round(float(y), 6),
            "z": round(float(z), 6),
        }

    return payload


def write_json_atomic(path: Path, payload) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(path.name + ".tmp")
    tmp_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    tmp_path.replace(path)


def should_retry_reconnect(attempts: int, max_attempts: int) -> bool:
    if int(max_attempts) <= 0:
        return True
    return int(attempts) < int(max_attempts)


def release_capture(cap) -> None:
    try:
        if cap is not None:
            cap.release()
    except Exception:
        pass


def open_color_depth_cameras(args):
    color_cap, selected_source = open_camera(cv2, args.source, args.width, args.height)
    depth_cap = open_depth_camera(args.depth_source, args.width, args.height)
    return color_cap, depth_cap, selected_source


def open_depth_camera(source: str, width: int, height: int):
    cap = cv2.VideoCapture(source)
    configure_low_latency_capture(cv2, cap)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open depth source: {source}")

    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        raise RuntimeError(f"depth source opened but returned no frame: {source}")
    if frame.ndim != 2 or frame.dtype != np.uint16:
        cap.release()
        raise RuntimeError(
            f"depth source must return uint16 Z16 frame, got shape={frame.shape} dtype={frame.dtype}"
        )
    return cap


def draw_depth_text(
    frame,
    detection: Optional[Detection],
    depth_mm: Optional[float],
    xyz_m: Optional[Tuple[float, float, float]],
    fps: Optional[float] = None,
):
    annotated = draw_detection(frame, detection)
    lines = []
    if fps is not None:
        lines.append(f"fps={fps:.1f}")

    if detection is None:
        lines.append("no bottle")
    else:
        cx, cy = detection.center
        if depth_mm is None or xyz_m is None:
            lines.append("depth: invalid")
        else:
            x, y, z = xyz_m
            lines.append(f"depth={depth_mm:.0f}mm  XYZ=({x:.3f},{y:.3f},{z:.3f})m")
        cv2.circle(annotated, (cx, cy), 8, (0, 0, 255), 2)

    y = max(28, annotated.shape[0] - 52)
    for line in lines:
        cv2.putText(
            annotated,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        y += 26
    return annotated


def build_arg_parser() -> argparse.ArgumentParser:
    base = build_2d_arg_parser()
    base.description = "Run YOLOv8 bottle detection and sample RealSense Z16 depth."
    base.set_defaults(
        model="/home/unitree/G1_Docker_Deploy/python_gui/yolov8n.pt",
        source="1",
        conf=0.30,
    )
    base.add_argument("--depth-source", default="/dev/video0", help="RealSense Z16 depth device")
    base.add_argument("--depth-radius", type=int, default=7, help="Median depth ROI radius")
    base.add_argument("--min-depth-mm", type=int, default=100, help="Minimum valid depth")
    base.add_argument("--max-depth-mm", type=int, default=5000, help="Maximum valid depth")
    base.add_argument("--fx", type=float, default=DEFAULT_COLOR_FX)
    base.add_argument("--fy", type=float, default=DEFAULT_COLOR_FY)
    base.add_argument("--ppx", type=float, default=DEFAULT_COLOR_PPX)
    base.add_argument("--ppy", type=float, default=DEFAULT_COLOR_PPY)
    base.add_argument(
        "--json-out",
        default=str(Path(__file__).resolve().parent / "latest_bottle_3d.json"),
        help="Write latest detection/depth payload here",
    )
    base.add_argument("--json-every", type=int, default=1, help="Write JSON every N frames")
    base.add_argument("--no-json", action="store_true", help="Disable JSON output")
    base.add_argument(
        "--reconnect-delay",
        type=float,
        default=2.0,
        help="Seconds to wait before reopening cameras after a read/open failure",
    )
    base.add_argument(
        "--max-reconnect-attempts",
        type=int,
        default=0,
        help="Maximum reconnect attempts after a camera failure. 0 means unlimited.",
    )
    base.add_argument(
        "--no-reconnect",
        action="store_true",
        help="Exit on camera failure instead of reopening camera devices",
    )
    return base


def run(args: argparse.Namespace) -> int:
    from ultralytics import YOLO

    model_path = Path(args.model)
    if not model_path.exists():
        print(f"ERROR: model not found: {model_path}")
        return 2

    target_names = normalize_target_names(args.targets)
    if not target_names:
        print("ERROR: --targets must contain at least one class name")
        return 2

    color_cap = None
    depth_cap = None
    selected_source = None
    open_attempts = 0
    while True:
        try:
            color_cap, depth_cap, selected_source = open_color_depth_cameras(args)
            break
        except RuntimeError as exc:
            open_attempts += 1
            print(f"WARN: camera open failed: {exc}")
            release_capture(color_cap)
            release_capture(depth_cap)
            color_cap = None
            depth_cap = None
            if args.no_reconnect or not should_retry_reconnect(
                open_attempts, int(args.max_reconnect_attempts)
            ):
                print(f"ERROR: {exc}")
                return 3
            print(f"waiting {float(args.reconnect_delay):.1f}s before reconnect...")
            time.sleep(max(0.1, float(args.reconnect_delay)))

    model = YOLO(str(model_path))
    intrinsics = CameraIntrinsics(args.fx, args.fy, args.ppx, args.ppy)
    save_dir = Path(args.save_dir)
    if args.save_every > 0:
        save_dir.mkdir(parents=True, exist_ok=True)

    print(f"model: {model_path}")
    print(f"color source: {args.source} (selected: {selected_source})")
    print(f"depth source: {args.depth_source}")
    print(f"targets: {', '.join(sorted(target_names))}")
    print("vision-depth-only mode: no arm control will be used")
    print("note: direct V4L2 depth is not hardware-aligned to color; use this as a first check")
    print("camera reconnect: enabled" if not args.no_reconnect else "camera reconnect: disabled")
    if not args.no_json:
        print(f"json output: {args.json_out}")

    frame_index = 0
    saved_count = 0
    window_name = "G1 bottle depth"
    fps: Optional[float] = None
    last_loop_s: Optional[float] = None

    if args.show:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 960, 720)
        cv2.moveWindow(window_name, 120, 120)
        if hasattr(cv2, "WND_PROP_TOPMOST"):
            cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)

    try:
        while True:
            ok_color, color = read_latest_frame(color_cap, args.flush_frames)
            ok_depth, depth = read_latest_frame(depth_cap, args.flush_frames)
            if not ok_color or color is None:
                print(f"WARN: failed to read color frame at index {frame_index}; reconnecting cameras")
                release_capture(color_cap)
                release_capture(depth_cap)
                reconnect_attempts = 0
                while True:
                    try:
                        color_cap, depth_cap, selected_source = open_color_depth_cameras(args)
                        print(f"reconnected cameras; color selected: {selected_source}")
                        break
                    except RuntimeError as exc:
                        reconnect_attempts += 1
                        print(f"WARN: reconnect failed: {exc}")
                        if args.no_reconnect or not should_retry_reconnect(
                            reconnect_attempts, int(args.max_reconnect_attempts)
                        ):
                            print(f"ERROR: {exc}")
                            return 4
                        time.sleep(max(0.1, float(args.reconnect_delay)))
                continue
            if not ok_depth or depth is None:
                print(f"WARN: failed to read depth frame at index {frame_index}; reconnecting cameras")
                release_capture(color_cap)
                release_capture(depth_cap)
                reconnect_attempts = 0
                while True:
                    try:
                        color_cap, depth_cap, selected_source = open_color_depth_cameras(args)
                        print(f"reconnected cameras; color selected: {selected_source}")
                        break
                    except RuntimeError as exc:
                        reconnect_attempts += 1
                        print(f"WARN: reconnect failed: {exc}")
                        if args.no_reconnect or not should_retry_reconnect(
                            reconnect_attempts, int(args.max_reconnect_attempts)
                        ):
                            print(f"ERROR: {exc}")
                            return 4
                        time.sleep(max(0.1, float(args.reconnect_delay)))
                continue

            if depth.ndim == 3:
                depth = depth[:, :, 0].astype(np.uint16)
            if depth.dtype != np.uint16:
                depth = depth.astype(np.uint16)

            result = model(color, conf=float(args.conf), imgsz=int(args.imgsz), verbose=False)[0]
            detection = select_best_detection(
                boxes=result.boxes,
                names=model.names,
                target_names=target_names,
                min_confidence=float(args.conf),
            )

            depth_mm: Optional[float] = None
            xyz_m: Optional[Tuple[float, float, float]] = None

            if detection is not None:
                color_h, color_w = color.shape[:2]
                depth_h, depth_w = depth.shape[:2]
                depth_uv = scale_point(
                    detection.center,
                    source_size=(color_w, color_h),
                    target_size=(depth_w, depth_h),
                )
                depth_mm = sample_depth_median_mm(
                    depth,
                    center=depth_uv,
                    radius=int(args.depth_radius),
                    min_mm=int(args.min_depth_mm),
                    max_mm=int(args.max_depth_mm),
                )
                if depth_mm is not None:
                    depth_m = float(depth_mm) / 1000.0
                    xyz_m = deproject_pixel(
                        detection.center[0],
                        detection.center[1],
                        depth_m,
                        intrinsics,
                    )

            now_s = time.time()
            if last_loop_s is not None:
                dt = max(1e-6, now_s - last_loop_s)
                instant_fps = 1.0 / dt
                fps = instant_fps if fps is None else (0.85 * fps + 0.15 * instant_fps)
            last_loop_s = now_s

            annotated = draw_depth_text(color, detection, depth_mm, xyz_m, fps=fps)

            if not args.no_json and int(args.json_every) > 0 and frame_index % int(args.json_every) == 0:
                payload = build_output_payload(
                    frame_index=frame_index,
                    timestamp_s=now_s,
                    fps=fps,
                    detection=detection,
                    depth_mm=depth_mm,
                    xyz_m=xyz_m,
                    intrinsics=intrinsics,
                    color_source=str(args.source),
                    depth_source=str(args.depth_source),
                )
                write_json_atomic(Path(args.json_out), payload)

            if (
                detection is not None
                and args.print_every > 0
                and frame_index % args.print_every == 0
            ):
                cx, cy = detection.center
                if xyz_m is None or depth_mm is None:
                    print(
                        f"bottle center: cx={cx}, cy={cy}, conf={detection.confidence:.3f}, depth=invalid"
                    )
                else:
                    x, y, z = xyz_m
                    print(
                        "bottle 3d: "
                        f"cx={cx}, cy={cy}, conf={detection.confidence:.3f}, "
                        f"depth_mm={depth_mm:.0f}, "
                        f"camera_xyz_m=({x:.3f}, {y:.3f}, {z:.3f})"
                    )
            elif args.print_every > 0 and frame_index % args.print_every == 0:
                print(f"no bottle detected at frame {frame_index}")

            should_save = (
                args.save_every > 0
                and frame_index % args.save_every == 0
                and (args.max_save == 0 or saved_count < args.max_save)
            )
            if should_save:
                stamp = time.strftime("%Y%m%d_%H%M%S")
                out_path = save_dir / f"bottle_depth_debug_{stamp}_{frame_index:06d}.jpg"
                cv2.imwrite(str(out_path), annotated)
                saved_count += 1
                print(f"saved debug image: {out_path}")

            if args.show:
                cv2.imshow(window_name, annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            frame_index += 1
            if args.frames > 0 and frame_index >= args.frames:
                break

    except KeyboardInterrupt:
        print("stopped by user")
    finally:
        release_capture(color_cap)
        release_capture(depth_cap)
        if args.show:
            cv2.destroyAllWindows()

    return 0


def main() -> int:
    return run(build_arg_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
