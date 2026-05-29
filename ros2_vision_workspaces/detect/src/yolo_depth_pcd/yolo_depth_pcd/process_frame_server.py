from __future__ import annotations

import json
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node

from yolo_depth_pcd_interfaces.srv import ProcessFrame


def _clip_bbox(x1: float, y1: float, x2: float, y2: float, w: int, h: int):
    x1i = max(0, min(int(round(x1)), w - 1))
    y1i = max(0, min(int(round(y1)), h - 1))
    x2i = max(0, min(int(round(x2)), w - 1))
    y2i = max(0, min(int(round(y2)), h - 1))
    if x2i < x1i:
        x1i, x2i = x2i, x1i
    if y2i < y1i:
        y1i, y2i = y2i, y1i
    return x1i, y1i, x2i, y2i


def _run_yolo_one(weights_path: Path, img_path: Path, conf: float, class_name=None, class_id=None):
    try:
        import torch
        from ultralytics.nn.tasks import DetectionModel

        if hasattr(torch, "serialization") and hasattr(torch.serialization, "add_safe_globals"):
            torch.serialization.add_safe_globals([DetectionModel])

        if hasattr(torch, "load") and not getattr(torch.load, "_yolo_depth_pcd_patched", False):
            _orig_torch_load = torch.load

            def _torch_load_no_weights_only(*args, **kwargs):
                if "weights_only" not in kwargs:
                    kwargs["weights_only"] = False
                return _orig_torch_load(*args, **kwargs)

            _torch_load_no_weights_only._yolo_depth_pcd_patched = True  # type: ignore[attr-defined]
            torch.load = _torch_load_no_weights_only  # type: ignore[assignment]
    except Exception:
        pass

    from ultralytics import YOLO

    model = YOLO(str(weights_path))
    res = model.predict(source=str(img_path), conf=float(conf), max_det=50, verbose=False)
    if not res:
        return None
    r0 = res[0]
    if r0.boxes is None:
        return None

    names = getattr(r0, "names", {})
    xyxy = r0.boxes.xyxy
    confs = r0.boxes.conf
    clss = r0.boxes.cls
    if xyxy is None or confs is None or clss is None:
        return None

    xyxy = xyxy.cpu().numpy()
    confs = confs.cpu().numpy()
    clss = clss.cpu().numpy().astype(int)

    best = None
    for i in range(int(xyxy.shape[0])):
        cid = int(clss[i])
        cname = str(names.get(cid, cid))
        if class_id is not None and cid != int(class_id):
            continue
        if class_name is not None and cname != str(class_name):
            continue
        x1, y1, x2, y2 = map(float, xyxy[i].tolist())
        d = {"cls_id": cid, "name": cname, "conf": float(confs[i]), "xyxy": (x1, y1, x2, y2)}
        if best is None or d["conf"] > best["conf"]:
            best = d
    return best


def order_polygon_clockwise_closed(poly_xy):
    import numpy as np

    if poly_xy is None:
        return None
    poly_xy = np.asarray(poly_xy, dtype=np.float32).reshape(-1, 2)
    if poly_xy.shape[0] < 3:
        return None

    keep = [0]
    for i in range(1, poly_xy.shape[0]):
        if not np.allclose(poly_xy[i], poly_xy[keep[-1]]):
            keep.append(i)
    poly_xy = poly_xy[keep]
    if poly_xy.shape[0] >= 2 and np.allclose(poly_xy[0], poly_xy[-1]):
        poly_xy = poly_xy[:-1]
    if poly_xy.shape[0] < 3:
        return None

    c = poly_xy.mean(axis=0)
    ang = np.arctan2(poly_xy[:, 1] - c[1], poly_xy[:, 0] - c[0])
    poly_xy = poly_xy[np.argsort(ang)]
    x = poly_xy[:, 0]
    y = poly_xy[:, 1]
    area2 = float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
    if area2 > 0:
        poly_xy = poly_xy[::-1]
    poly_xy = np.vstack([poly_xy, poly_xy[0]])
    return poly_xy


def preprocess_roi(roi_gray):
    import cv2

    if roi_gray is None or roi_gray.size == 0:
        return None
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    x = clahe.apply(roi_gray)
    x = cv2.bilateralFilter(x, d=7, sigmaColor=50, sigmaSpace=50)
    return x


def detect_edges_single_contour(roi_gray):
    import cv2
    import numpy as np

    if roi_gray is None or roi_gray.size == 0:
        return None, None

    v = float(np.median(roi_gray))
    lo = int(max(0, (1.0 - 0.33) * v))
    hi = int(min(255, (1.0 + 0.33) * v))
    edges = cv2.Canny(roi_gray, lo, hi, L2gradient=True)

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    edges = cv2.dilate(edges, k, iterations=1)
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, k, iterations=2)

    contours, _ = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if isinstance(contours, tuple):
        contours = list(contours)
    if not contours:
        return closed, None

    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    c = contours[0]
    if cv2.contourArea(c) < 30:
        return closed, None
    return closed, c


def fit_single_polygon(main_contour, roi_shape, poly_epsilon=0.02):
    import cv2
    import numpy as np

    if main_contour is None or len(main_contour) < 3:
        return None

    roi_area = float(roi_shape[0] * roi_shape[1])
    area = float(cv2.contourArea(main_contour))
    if area < max(60.0, roi_area * 0.01):
        return None

    peri = float(cv2.arcLength(main_contour, True))
    eps = float(poly_epsilon) * peri
    poly = cv2.approxPolyDP(main_contour, eps, True).reshape(-1, 2)
    poly = order_polygon_clockwise_closed(poly)
    if poly is not None and poly.shape[0] >= 4:
        return poly

    rect = cv2.minAreaRect(main_contour)
    box = cv2.boxPoints(rect).astype(np.float32)
    box = order_polygon_clockwise_closed(box)
    return box


def load_depth_png(depth_png: Path):
    import numpy as np
    from PIL import Image

    arr = np.array(Image.open(depth_png))
    return arr.astype(np.float32)


def _sample_depth(depth_m, u: int, v: int) -> float:
    h, w = depth_m.shape[:2]
    uu = max(0, min(int(u), w - 1))
    vv = max(0, min(int(v), h - 1))
    return float(depth_m[vv, uu])


def _parse_intrinsics(frame_json: dict):
    k = frame_json["intrinsics"]
    fx = float(k[0])
    fy = float(k[4])
    cx = float(k[2])
    cy = float(k[5])
    return fx, fy, cx, cy


def _parse_pose_4x4(frame_json: dict):
    import numpy as np

    p = frame_json["cameraPoseARFrame"]
    if len(p) != 16:
        raise ValueError("cameraPoseARFrame must have 16 values")
    return np.array(p, dtype=np.float32).reshape(4, 4)


def _backproject_pixel_to_world(u, v, depth_m, fx, fy, cx, cy, T_world_cam, forward_neg_z, image_y_down):
    import numpy as np

    zf = float(depth_m)
    x_cam = (float(u) - float(cx)) * zf / float(fx)
    y_cam = (-(float(v) - float(cy)) if bool(image_y_down) else (float(v) - float(cy))) * zf / float(fy)
    z_cam = (-zf) if bool(forward_neg_z) else zf
    p_cam = np.array([x_cam, y_cam, z_cam, 1.0], dtype=np.float32)
    p_w = (T_world_cam @ p_cam).astype(np.float32)
    return p_w[:3]


def _compute_obb_and_inliers(pcd_path: Path, points_3d):
    import numpy as np
    import open3d as o3d

    pcd = o3d.io.read_point_cloud(str(pcd_path))
    if pcd.is_empty():
        raise RuntimeError(f"PCD/PLY is empty: {pcd_path}")

    pts = np.asarray(points_3d, dtype=np.float32)
    o3_pts = o3d.utility.Vector3dVector(pts.astype(np.float64))
    obb = o3d.geometry.OrientedBoundingBox.create_from_points(o3_pts)

    pcd_pts = np.asarray(pcd.points)
    if pcd_pts.size == 0:
        raise RuntimeError(f"Point cloud has no points: {pcd_path}")

    in_idx = obb.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(pcd_pts))
    corners = np.asarray(obb.get_box_points(), dtype=np.float32)
    return corners, int(len(in_idx)), pcd_pts


def _header(node: Node, frame_id: str) -> Header:
    h = Header()
    h.stamp = node.get_clock().now().to_msg()
    h.frame_id = str(frame_id)
    return h


def _imgmsg_from_bgr(node: Node, frame_id: str, img_bgr) -> Image:
    msg = Image()
    msg.header = _header(node, frame_id)
    msg.height = int(img_bgr.shape[0])
    msg.width = int(img_bgr.shape[1])
    msg.encoding = 'bgr8'
    msg.is_bigendian = False
    msg.step = int(img_bgr.shape[1] * 3)
    msg.data = img_bgr.tobytes()
    return msg


def _pointcloud2_from_xyz(node: Node, frame_id: str, xyz):
    import numpy as np

    pts = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header = _header(node, frame_id)
    msg.height = 1
    msg.width = int(pts.shape[0])
    msg.is_bigendian = False
    msg.is_dense = False
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.data = pts.tobytes()
    return msg


def _marker_line_strip(node: Node, frame_id: str, ns: str, mid: int, pts_xyz, rgba, width: float) -> Marker:
    m = Marker()
    m.header = _header(node, frame_id)
    m.ns = str(ns)
    m.id = int(mid)
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = float(width)
    m.color.r = float(rgba[0])
    m.color.g = float(rgba[1])
    m.color.b = float(rgba[2])
    m.color.a = float(rgba[3])
    m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts_xyz]
    return m


def _marker_line_list(node: Node, frame_id: str, ns: str, mid: int, segs_xyz, rgba, width: float) -> Marker:
    m = Marker()
    m.header = _header(node, frame_id)
    m.ns = str(ns)
    m.id = int(mid)
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.scale.x = float(width)
    m.color.r = float(rgba[0])
    m.color.g = float(rgba[1])
    m.color.b = float(rgba[2])
    m.color.a = float(rgba[3])
    m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in segs_xyz]
    return m


def _resolve_paths(dataset_root: Path, frame_idx: str) -> Tuple[Path, Path, Optional[Path], Optional[Path]]:
    raw_root = dataset_root / "raw"

    candidates_img = [
        raw_root / "images" / f"img_{frame_idx}.jpg",
        raw_root / "images" / f"frame_{frame_idx}.jpg",
        raw_root / "images" / f"frame_{frame_idx}.png",
        raw_root / "images" / f"img_{frame_idx}.png",
        raw_root / "frames" / f"frame_{frame_idx}.jpg",
        raw_root / "frames" / f"frame_{frame_idx}.png",
    ]
    img_path = next((p for p in candidates_img if p.exists()), None)
    if img_path is None:
        raise FileNotFoundError(f"Cannot find RGB image for frame_idx={frame_idx} under: {raw_root}")

    candidates_depth = [
        raw_root / "depth" / f"depth_{frame_idx}.png",
        raw_root / "depth" / f"depth_{frame_idx}.exr",
    ]
    depth_path = next((p for p in candidates_depth if p.exists()), None)
    if depth_path is None:
        raise FileNotFoundError(f"Cannot find depth for frame_idx={frame_idx} under: {raw_root / 'depth'}")

    meta_json = raw_root / "frames" / f"frame_{frame_idx}.json"
    frame_json_path = meta_json if meta_json.exists() else None

    pcd_candidates = [
        dataset_root / "pcd_cropped_from_global" / f"crop_{frame_idx}.pcd",
        dataset_root / "pcd_per_frame" / f"frame_{frame_idx}.pcd",
        dataset_root / "pcd_cropped_from_global" / f"crop_{frame_idx}.ply",
        dataset_root / "pcd_per_frame" / f"frame_{frame_idx}.ply",
    ]
    pcd_path = next((p for p in pcd_candidates if p.exists()), None)

    return img_path, depth_path, frame_json_path, pcd_path


class ProcessFrameServer(Node):
    def __init__(self):
        super().__init__("process_frame_server")

        self.declare_parameter("weights", "")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("class_name", "")
        self.declare_parameter("class_id", -1)
        self.declare_parameter("depth_scale", 1000.0)
        self.declare_parameter("forward_neg_z", True)
        self.declare_parameter("image_y_down", True)
        self.declare_parameter("polygon_max_points", 40)
        self.declare_parameter("publish_viz", False)
        self.declare_parameter("viz_frame_id", "world")
        self.declare_parameter("viz_image_frame_id", "camera")

        self._viz_enabled = bool(self.get_parameter("publish_viz").value)
        self._viz_frame_id = str(self.get_parameter("viz_frame_id").value)
        self._viz_image_frame_id = str(self.get_parameter("viz_image_frame_id").value)

        if self._viz_enabled:
            self.pub_stage1 = self.create_publisher(Image, "viz/stage1_bbox", 1)
            self.pub_stage2 = self.create_publisher(Image, "viz/stage2_roi", 1)
            self.pub_stage3 = self.create_publisher(Image, "viz/stage3_polygon", 1)
            self.pub_stage4 = self.create_publisher(Image, "viz/stage4_points", 1)
            self.pub_cloud = self.create_publisher(PointCloud2, "viz/pointcloud", 1)
            self.pub_markers = self.create_publisher(MarkerArray, "viz/markers", 1)

        self.srv = self.create_service(ProcessFrame, "process_frame", self._handle)

    def _handle(self, request: ProcessFrame.Request, response: ProcessFrame.Response):
        try:
            dataset_root = Path(request.dataset_root)
            frame_idx = str(request.frame_idx)
            save_debug = bool(request.save_debug)

            weights = Path(str(self.get_parameter("weights").value))
            if not str(weights):
                raise FileNotFoundError("Parameter 'weights' is empty")
            if not weights.exists():
                raise FileNotFoundError(f"weights not found: {weights}")

            img_path, depth_path, frame_json_path, pcd_path = _resolve_paths(dataset_root, frame_idx)

            if frame_json_path is None:
                raise FileNotFoundError(
                    f"Missing frame metadata json: {dataset_root / 'raw' / 'frames' / f'frame_{frame_idx}.json'}"
                )

            frame_json = json.loads(frame_json_path.read_text(encoding="utf-8"))

            fx, fy, cx, cy = _parse_intrinsics(frame_json)
            T_world_cam = _parse_pose_4x4(frame_json)

            conf = float(self.get_parameter("conf").value)
            class_name = str(self.get_parameter("class_name").value).strip() or None
            class_id_raw = int(self.get_parameter("class_id").value)
            class_id = None if class_id_raw < 0 else class_id_raw

            det = _run_yolo_one(weights, img_path, conf, class_name=class_name, class_id=class_id)
            if det is None:
                response.success = False
                response.message = "no detection"
                response.output_dir = str(dataset_root / "demo2_outputs")
                return response

            from PIL import Image
            import cv2
            import numpy as np

            img_pil = Image.open(img_path).convert("RGB")
            w, h = img_pil.size
            img_rgb = np.array(img_pil)
            img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

            if self._viz_enabled:
                vis1 = img_bgr.copy()
                x1, y1, x2, y2 = det["xyxy"]
                cv2.rectangle(vis1, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 3)
                cv2.putText(
                    vis1,
                    f"{det['name']} {det['conf']:.2f}",
                    (int(x1), max(0, int(y1) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                self.pub_stage1.publish(_imgmsg_from_bgr(self, self._viz_image_frame_id, vis1))

            x1, y1, x2, y2 = det["xyxy"]
            x1i, y1i, x2i, y2i = _clip_bbox(x1, y1, x2, y2, w, h)
            roi_bgr = img_bgr[y1i : y2i + 1, x1i : x2i + 1].copy()

            if self._viz_enabled:
                self.pub_stage2.publish(_imgmsg_from_bgr(self, self._viz_image_frame_id, roi_bgr))

            roi_gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
            roi_pre = preprocess_roi(roi_gray)
            _, main_contour = detect_edges_single_contour(roi_pre)
            poly = fit_single_polygon(main_contour, roi_bgr.shape, poly_epsilon=0.015)
            if poly is None:
                response.success = False
                response.message = "no polygon"
                response.output_dir = str(dataset_root / "demo2_outputs")
                return response

            poly = poly.reshape(-1, 2)
            poly_abs = poly.astype(np.float32) + np.array([float(x1i), float(y1i)], dtype=np.float32)

            polygon_max_points = int(self.get_parameter("polygon_max_points").value)
            if poly_abs.shape[0] > polygon_max_points + 1:
                step = max(1, int(round((poly_abs.shape[0] - 1) / polygon_max_points)))
                poly_abs = np.vstack([poly_abs[:-1:step], poly_abs[-1:]])

            if self._viz_enabled:
                vis3 = img_bgr.copy()
                poly_i = poly_abs.astype(np.int32).reshape(-1, 1, 2)
                cv2.polylines(vis3, [poly_i], isClosed=True, color=(0, 255, 0), thickness=2)
                for (px, py) in poly_abs.tolist():
                    cv2.circle(vis3, (int(px), int(py)), 2, (0, 0, 255), -1)
                self.pub_stage3.publish(_imgmsg_from_bgr(self, self._viz_image_frame_id, vis3))

            depth_scale = float(self.get_parameter("depth_scale").value)
            depth_raw = load_depth_png(depth_path)
            depth_m = depth_raw.astype(np.float32) / depth_scale
            if depth_m.shape[0] != h or depth_m.shape[1] != w:
                depth_m = cv2.resize(depth_m, (int(w), int(h)), interpolation=cv2.INTER_LINEAR)

            forward_neg_z = bool(self.get_parameter("forward_neg_z").value)
            image_y_down = bool(self.get_parameter("image_y_down").value)

            points_2d = [(int(x), int(y)) for (x, y) in poly_abs.tolist()]
            points_3d = []
            for (uu, vv) in points_2d:
                dm = _sample_depth(depth_m, uu, vv)
                if not (dm > 0.0):
                    dm = float("nan")
                p3 = _backproject_pixel_to_world(uu, vv, dm, fx, fy, cx, cy, T_world_cam, forward_neg_z, image_y_down)
                points_3d.append(p3)

            points_3d = np.asarray(points_3d, dtype=np.float32)
            finite = np.isfinite(points_3d).all(axis=1)
            if not finite.any():
                response.success = False
                response.message = "all depth points are invalid"
                response.output_dir = str(dataset_root / "demo2_outputs")
                return response

            points_3d_valid = points_3d[finite]

            if self._viz_enabled:
                points_2d_valid = [p for p, ok in zip(points_2d, finite.tolist()) if ok]
                vis4 = img_bgr.copy()
                for (uu, vv) in points_2d_valid:
                    cv2.circle(vis4, (int(uu), int(vv)), 2, (255, 0, 255), -1)
                self.pub_stage4.publish(_imgmsg_from_bgr(self, self._viz_image_frame_id, vis4))

            output_dir = dataset_root / "demo2_outputs"
            output_dir.mkdir(parents=True, exist_ok=True)

            corners = None
            inlier_count = 0
            pcd_pts = None
            if pcd_path is not None:
                corners, inlier_count, pcd_pts = _compute_obb_and_inliers(pcd_path, points_3d_valid)

                if self._viz_enabled and pcd_pts is not None:
                    self.pub_cloud.publish(_pointcloud2_from_xyz(self, self._viz_frame_id, pcd_pts))

                if self._viz_enabled and corners is not None:
                    poly3d = points_3d_valid
                    if poly3d.shape[0] >= 2:
                        poly_marker = _marker_line_strip(self, self._viz_frame_id, 'polygon', 0, poly3d.tolist(), (0.0, 1.0, 0.0, 1.0), 0.01)
                    else:
                        poly_marker = Marker()

                    c = corners
                    edges = [
                        (0, 1), (1, 2), (2, 3), (3, 0),
                        (4, 5), (5, 6), (6, 7), (7, 4),
                        (0, 4), (1, 5), (2, 6), (3, 7),
                    ]
                    seg_pts = []
                    for a, b in edges:
                        seg_pts.append(c[a].tolist())
                        seg_pts.append(c[b].tolist())
                    obb_marker = _marker_line_list(self, self._viz_frame_id, 'obb', 1, seg_pts, (1.0, 0.0, 0.0, 1.0), 0.01)

                    ma = MarkerArray()
                    ma.markers = [poly_marker, obb_marker]
                    self.pub_markers.publish(ma)

                if save_debug:
                    import open3d as o3d

                    pcd = o3d.io.read_point_cloud(str(pcd_path))
                    merged = o3d.geometry.PointCloud()
                    merged_points = np.vstack([np.asarray(pcd.points), points_3d_valid]).astype(np.float64)
                    merged.points = o3d.utility.Vector3dVector(merged_points)
                    o3d.io.write_point_cloud(str(output_dir / f"stage5_merged_{Path(img_path).stem}.ply"), merged)

            response.success = True
            response.message = "ok"
            response.class_id = int(det["cls_id"])
            response.class_name = str(det["name"])
            response.confidence = float(det["conf"])
            response.bbox_xyxy = [float(x1), float(y1), float(x2), float(y2)]
            response.polygon_world = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points_3d_valid]

            if corners is not None:
                response.obb_corners_world = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in corners]
            else:
                response.obb_corners_world = []

            response.pcd_inlier_count = int(inlier_count)
            response.output_dir = str(output_dir)
            return response

        except Exception as e:
            response.success = False
            response.message = str(e)
            response.output_dir = str(Path(request.dataset_root) / "demo2_outputs")
            return response


def main():
    rclpy.init()
    node = ProcessFrameServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
