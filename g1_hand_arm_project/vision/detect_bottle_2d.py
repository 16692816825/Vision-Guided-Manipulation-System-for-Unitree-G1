#!/usr/bin/env python3
"""
YOLOv8 2D bottle detector for the G1 + Revo2 project.

This script is intentionally vision-only. It does not import Unitree arm
control code, does not publish to rt/arm_sdk, and does not move the robot.
"""

from __future__ import annotations

import argparse
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Set, Tuple


DEFAULT_MODEL = "/home/unitree/runs/detect/train4/weights/best.pt"
DEFAULT_SOURCE = "auto"


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    xyxy: Tuple[float, float, float, float]
    center: Tuple[int, int]


def box_center(xyxy: Sequence[float]) -> Tuple[int, int]:
    x1, y1, x2, y2 = [float(v) for v in xyxy]
    return int(round((x1 + x2) / 2.0)), int(round((y1 + y2) / 2.0))


def normalize_target_names(value: str) -> Set[str]:
    return {item.strip().lower() for item in value.split(",") if item.strip()}


def _as_scalar(value) -> float:
    if hasattr(value, "item"):
        return float(value.item())
    if isinstance(value, (list, tuple)):
        return _as_scalar(value[0])
    return float(value)


def _as_xyxy(value) -> Tuple[float, float, float, float]:
    if hasattr(value, "tolist"):
        value = value.tolist()
    if value and isinstance(value[0], (list, tuple)):
        value = value[0]
    return tuple(float(v) for v in value[:4])  # type: ignore[return-value]


def _label_for_class(names, class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    try:
        return str(names[class_id])
    except Exception:
        return str(class_id)


def center_distance(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return ((float(a[0] - b[0]) ** 2) + (float(a[1] - b[1]) ** 2)) ** 0.5


def collect_target_detections(
    boxes: Iterable,
    names,
    target_names: Set[str],
    min_confidence: float,
) -> List[Detection]:
    detections: List[Detection] = []

    for box in boxes or []:
        confidence = _as_scalar(box.conf)
        if confidence < min_confidence:
            continue

        class_id = int(_as_scalar(box.cls))
        label = _label_for_class(names, class_id)
        if label.lower() not in target_names:
            continue

        xyxy = _as_xyxy(box.xyxy[0])
        detections.append(
            Detection(
                label=label,
                confidence=round(confidence, 6),
                xyxy=xyxy,
                center=box_center(xyxy),
            )
        )

    return detections


def select_best_detection(
    boxes: Iterable,
    names,
    target_names: Set[str],
    min_confidence: float,
) -> Optional[Detection]:
    detections = collect_target_detections(
        boxes=boxes,
        names=names,
        target_names=target_names,
        min_confidence=min_confidence,
    )
    return max(detections, key=lambda item: item.confidence, default=None)


def smooth_detection(previous: Detection, current: Detection, alpha: float) -> Detection:
    alpha = min(1.0, max(0.0, alpha))
    xyxy = tuple(
        (old_value * (1.0 - alpha)) + (new_value * alpha)
        for old_value, new_value in zip(previous.xyxy, current.xyxy)
    )
    return Detection(
        label=current.label,
        confidence=current.confidence,
        xyxy=xyxy,  # type: ignore[arg-type]
        center=box_center(xyxy),
    )


class TargetTracker:
    """Keep the bottle target stable when YOLO briefly detects another object."""

    def __init__(
        self,
        max_jump_px: float,
        smooth_alpha: float,
        lost_frames: int,
        switch_frames: int,
        lock_confidence: float = 0.0,
    ):
        self.max_jump_px = max(1.0, float(max_jump_px))
        self.smooth_alpha = min(1.0, max(0.0, float(smooth_alpha)))
        self.lost_frames = max(0, int(lost_frames))
        self.switch_frames = max(1, int(switch_frames))
        self.lock_confidence = max(0.0, float(lock_confidence))
        self.current: Optional[Detection] = None
        self.missed_frames = 0
        self.pending: Optional[Detection] = None
        self.pending_frames = 0

    def update(self, detections: Iterable[Detection]) -> Optional[Detection]:
        candidates = list(detections)
        if not candidates:
            self.pending = None
            self.pending_frames = 0
            self.missed_frames += 1
            if self.current is not None and self.missed_frames <= self.lost_frames:
                return self.current
            self.current = None
            return None

        if self.current is None:
            confident = [
                item for item in candidates if item.confidence >= self.lock_confidence
            ]
            if not confident:
                return None
            return self._accept(max(confident, key=lambda item: item.confidence))

        nearby = [
            item
            for item in candidates
            if center_distance(item.center, self.current.center) <= self.max_jump_px
        ]
        if nearby:
            selected = max(
                nearby,
                key=lambda item: item.confidence
                - 0.25 * center_distance(item.center, self.current.center) / self.max_jump_px,
            )
            return self._accept(selected)

        switch_candidates = [
            item for item in candidates if item.confidence >= self.lock_confidence
        ]
        if not switch_candidates:
            self.missed_frames += 1
            if self.missed_frames <= self.lost_frames:
                return self.current
            self.current = None
            return None

        strongest = max(switch_candidates, key=lambda item: item.confidence)
        if (
            self.pending is not None
            and center_distance(strongest.center, self.pending.center) <= self.max_jump_px
        ):
            self.pending_frames += 1
            self.pending = strongest
        else:
            self.pending = strongest
            self.pending_frames = 1

        if self.pending_frames >= self.switch_frames:
            return self._accept(strongest)

        self.missed_frames += 1
        if self.missed_frames <= self.lost_frames:
            return self.current

        self.current = None
        return None

    def _accept(self, detection: Detection) -> Detection:
        if self.current is None:
            self.current = detection
        else:
            self.current = smooth_detection(self.current, detection, self.smooth_alpha)
        self.missed_frames = 0
        self.pending = None
        self.pending_frames = 0
        return self.current


def parse_source(value: str):
    if value.lower() == "auto":
        return "auto"
    if value.isdigit():
        return int(value)
    return value


def camera_candidates(source) -> List:
    if source == "auto":
        return ["/dev/video4", "/dev/video2", 1, 4, 2, 0, 3, 5]
    match = re.fullmatch(r"/dev/video(\d+)", str(source))
    if match:
        return [source, int(match.group(1))]
    return [source]


def backend_names_for_candidate(candidate) -> List[str]:
    if isinstance(candidate, int):
        return ["v4l2", "default"]
    if re.fullmatch(r"/dev/video\d+", str(candidate)):
        return ["v4l2", "default"]
    return ["default"]


def create_capture(cv2, candidate, backend_name: str):
    if backend_name == "v4l2":
        return cv2.VideoCapture(candidate, cv2.CAP_V4L2)
    return cv2.VideoCapture(candidate)


def configure_low_latency_capture(cv2, cap) -> None:
    if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)


def read_latest_frame(cap, flush_frames: int = 0):
    grabbed = False
    for _ in range(max(0, int(flush_frames))):
        if not cap.grab():
            break
        grabbed = True
    if grabbed:
        return cap.retrieve()
    return cap.read()


def open_camera(cv2, source_value: str, width: int, height: int):
    parsed_source = parse_source(source_value)
    last_error = None

    for candidate in camera_candidates(parsed_source):
        for backend_name in backend_names_for_candidate(candidate):
            cap = create_capture(cv2, candidate, backend_name)
            configure_low_latency_capture(cv2, cap)
            if width > 0:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            if height > 0:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

            if not cap.isOpened():
                last_error = f"candidate {candidate} with {backend_name} did not open"
                cap.release()
                continue

            ok, frame = cap.read()
            if not ok or frame is None:
                last_error = (
                    f"candidate {candidate} with {backend_name} opened "
                    "but did not return a frame"
                )
                cap.release()
                continue

            return cap, f"{candidate}/{backend_name}"

    raise RuntimeError(last_error or f"no usable camera source for {source_value}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run YOLOv8 bottle detection and print 2D center points only."
    )
    parser.add_argument("--model", default=DEFAULT_MODEL, help="YOLOv8 model path")
    parser.add_argument(
        "--source",
        default=DEFAULT_SOURCE,
        help="Camera source: auto, /dev/videoN, numeric index, image/video path",
    )
    parser.add_argument("--conf", type=float, default=0.30, help="Confidence threshold")
    parser.add_argument(
        "--targets",
        default="bottle,water bottle",
        help="Comma-separated class names to accept",
    )
    parser.add_argument("--width", type=int, default=640, help="Requested camera width")
    parser.add_argument("--height", type=int, default=480, help="Requested camera height")
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="Number of frames to process. 0 means run until Ctrl+C.",
    )
    parser.add_argument(
        "--save-dir",
        default=str(Path(__file__).resolve().parent / "debug"),
        help="Directory for debug images",
    )
    parser.add_argument(
        "--save-every",
        type=int,
        default=30,
        help="Save one debug image every N frames. 0 disables saving.",
    )
    parser.add_argument(
        "--max-save",
        type=int,
        default=20,
        help="Maximum number of debug images to save. 0 means no limit.",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=10,
        help="Print no-detection status every N frames.",
    )
    parser.add_argument("--show", action="store_true", help="Show live window")
    parser.add_argument(
        "--flush-frames",
        type=int,
        default=4,
        help="Grab and discard this many buffered frames before each inference",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=320,
        help="YOLO inference image size. Lower values reduce latency.",
    )
    parser.add_argument(
        "--infer-every",
        type=int,
        default=1,
        help="Run YOLO every N frames and reuse the last detection between inferences.",
    )
    parser.add_argument(
        "--no-track",
        action="store_true",
        help="Disable target tracking and always use the highest-confidence detection.",
    )
    parser.add_argument(
        "--track-max-jump",
        type=float,
        default=80.0,
        help="Maximum accepted center jump in pixels before treating a box as a new target.",
    )
    parser.add_argument(
        "--track-smooth-alpha",
        type=float,
        default=0.35,
        help="Smoothing factor for tracked boxes. Lower values reduce jitter.",
    )
    parser.add_argument(
        "--track-lost-frames",
        type=int,
        default=12,
        help="Keep the last tracked target for this many inference frames after missed detections.",
    )
    parser.add_argument(
        "--track-switch-frames",
        type=int,
        default=8,
        help="Require a far-away new target to persist this many inference frames before switching.",
    )
    parser.add_argument(
        "--track-lock-conf",
        type=float,
        default=0.25,
        help="Minimum confidence needed to lock the first target or switch to a far new target.",
    )
    return parser


def draw_detection(frame, detection: Optional[Detection]):
    import cv2

    annotated = frame.copy()
    if detection is None:
        return annotated

    x1, y1, x2, y2 = [int(round(v)) for v in detection.xyxy]
    cx, cy = detection.center
    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
    text = f"{detection.label} {detection.confidence:.2f} ({cx},{cy})"
    cv2.putText(
        annotated,
        text,
        (x1, max(20, y1 - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return annotated


def run(args: argparse.Namespace) -> int:
    import cv2
    from ultralytics import YOLO

    model_path = Path(args.model)
    if not model_path.exists():
        print(f"ERROR: model not found: {model_path}")
        return 2

    target_names = normalize_target_names(args.targets)
    if not target_names:
        print("ERROR: --targets must contain at least one class name")
        return 2

    model = YOLO(str(model_path))
    try:
        cap, selected_source = open_camera(cv2, args.source, args.width, args.height)
    except RuntimeError as exc:
        print(f"ERROR: cannot open camera source {args.source}: {exc}")
        return 3

    save_dir = Path(args.save_dir)
    if args.save_every > 0:
        save_dir.mkdir(parents=True, exist_ok=True)

    print(f"model: {model_path}")
    print(f"source: {args.source} (selected: {selected_source})")
    print(f"targets: {', '.join(sorted(target_names))}")
    tracker = None
    if not args.no_track:
        tracker = TargetTracker(
            max_jump_px=args.track_max_jump,
            smooth_alpha=args.track_smooth_alpha,
            lost_frames=args.track_lost_frames,
            switch_frames=args.track_switch_frames,
            lock_confidence=args.track_lock_conf,
        )
        print(
            "tracking: enabled "
            f"max_jump={tracker.max_jump_px:.1f}px "
            f"smooth_alpha={tracker.smooth_alpha:.2f} "
            f"lost_frames={tracker.lost_frames} "
            f"switch_frames={tracker.switch_frames} "
            f"lock_conf={tracker.lock_confidence:.2f}"
        )
    else:
        print("tracking: disabled")
    print("vision-only mode: no arm control will be used")
    print("press Ctrl+C to stop")

    frame_index = 0
    saved_count = 0
    window_name = "G1 bottle detection"
    infer_every = max(1, int(args.infer_every))
    last_detection: Optional[Detection] = None

    if args.show:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 960, 720)
        cv2.moveWindow(window_name, 80, 80)
        if hasattr(cv2, "WND_PROP_TOPMOST"):
            cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)

    try:
        while True:
            ok, frame = read_latest_frame(cap, args.flush_frames)
            if not ok or frame is None:
                print(f"ERROR: failed to read frame at index {frame_index}")
                return 4

            if frame_index % infer_every == 0:
                results = model(frame, conf=args.conf, imgsz=args.imgsz, verbose=False)
                result = results[0]
                candidates = collect_target_detections(
                    boxes=result.boxes,
                    names=model.names,
                    target_names=target_names,
                    min_confidence=args.conf,
                )
                if tracker is None:
                    last_detection = max(
                        candidates,
                        key=lambda item: item.confidence,
                        default=None,
                    )
                else:
                    last_detection = tracker.update(candidates)
            detection = last_detection

            annotated = draw_detection(frame, detection)

            if (
                detection is not None
                and args.print_every > 0
                and frame_index % args.print_every == 0
            ):
                x1, y1, x2, y2 = detection.xyxy
                cx, cy = detection.center
                print(
                    "bottle center: "
                    f"cx={cx}, cy={cy}, conf={detection.confidence:.3f}, "
                    f"box=({x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f})"
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
                out_path = save_dir / f"bottle_debug_{stamp}_{frame_index:06d}.jpg"
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
        cap.release()
        if args.show:
            cv2.destroyAllWindows()

    return 0


def main() -> int:
    parser = build_arg_parser()
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
