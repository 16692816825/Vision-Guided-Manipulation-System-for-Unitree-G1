from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image


class LivePipelineNode(Node):
    def __init__(self):
        super().__init__("live_pipeline_node")

        self.declare_parameter("weights", "")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("class_name", "handle")
        self.declare_parameter("class_id", -1)
        self.declare_parameter("device", "")
        self.declare_parameter("half", False)

        self.declare_parameter("yolo_every_n", 1)
        self.declare_parameter("save_dir", str(Path.home() / "yolo_depth_pcd_outputs"))
        self.declare_parameter("save_images", True)
        self.declare_parameter("save_every_n", 10)
        self.declare_parameter("publish_every_n", 1)

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("color_info_topic", "/camera/camera/color/camera_info")

        self.declare_parameter("sync_slop_sec", 0.1)

        self.declare_parameter("depth_scale", 1000.0)
        self.declare_parameter("publish_viz", True)
        self.declare_parameter("viz_frame_id", "")
        self.declare_parameter("viz_image_frame_id", "")

        self.declare_parameter("cloud_stride", 4)
        self.declare_parameter("depth_to_color_mode", "scale")

        self._viz_enabled = bool(self.get_parameter("publish_viz").value)

        from sensor_msgs.msg import PointCloud2

        from yolo_depth_pcd.process_frame_server import _imgmsg_from_bgr

        self._imgmsg_from_bgr = _imgmsg_from_bgr
        self.pub_yolo_rgb = self.create_publisher(Image, "yolo/rgb", 1)
        self.pub_cloud = self.create_publisher(PointCloud2, "yolo/pointcloud", 1)

        self._color_info: Optional[CameraInfo] = None

        self._frame_i = 0

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self._latest_color_msg: Optional[Image] = None
        self._latest_depth_msg: Optional[Image] = None

        self._sub_color = self.create_subscription(
            Image,
            str(self.get_parameter("color_topic").value),
            self._on_color,
            qos,
        )
        self._sub_depth = self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._on_depth,
            qos,
        )

        self._sub_color_info = self.create_subscription(
            CameraInfo,
            str(self.get_parameter("color_info_topic").value),
            self._on_color_info,
            qos,
        )

        self._lock = threading.Lock()
        self._latest_pair: Optional[Tuple[int, Image, Image]] = None
        self._new_pair_event = threading.Event()
        self._stop_event = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self._model = None
        self._load_model()

    def _on_color_info(self, msg: CameraInfo):
        self._color_info = msg

    def _on_color(self, msg: Image):
        self._latest_color_msg = msg
        self._try_enqueue_pair()

    def _on_depth(self, msg: Image):
        self._latest_depth_msg = msg
        self._try_enqueue_pair()

    def _try_enqueue_pair(self):
        color_msg = self._latest_color_msg
        depth_msg = self._latest_depth_msg
        if color_msg is None or depth_msg is None:
            return
        if self._color_info is None:
            return

        def _t(msg: Image) -> float:
            return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        slop = float(self.get_parameter("sync_slop_sec").value)
        dt = abs(_t(color_msg) - _t(depth_msg))
        if dt > max(0.0, slop):
            return

        self._on_pair(color_msg, depth_msg)

    def _load_model(self):
        weights = str(self.get_parameter("weights").value)
        if not weights:
            self.get_logger().error("Parameter 'weights' is empty")
            return

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

        self._model = YOLO(weights)

        dev = str(self.get_parameter("device").value).strip() or None
        half = bool(self.get_parameter("half").value)
        try:
            if dev is not None:
                self._model.to(dev)
            if half:
                m = getattr(self._model, "model", None)
                if m is not None and hasattr(m, "half"):
                    m.half()
        except Exception as e:
            self.get_logger().warn(f"Failed to set device/half: {e}")

        self.get_logger().info(f"Loaded YOLO weights: {weights}")

    def _run_yolo_one(self, img_rgb):
        if self._model is None:
            return None

        conf = float(self.get_parameter("conf").value)
        class_name = str(self.get_parameter("class_name").value).strip() or None
        class_id_raw = int(self.get_parameter("class_id").value)
        class_id = None if class_id_raw < 0 else class_id_raw

        res = self._model.predict(source=img_rgb, conf=float(conf), max_det=50, verbose=False)
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

    def _on_pair(self, color_msg: Image, depth_msg: Image):
        self._frame_i += 1
        frame_i = int(self._frame_i)
        if self._color_info is None:
            self.get_logger().warn("No CameraInfo yet; skipping frame")
            return

        publish_every_n = max(1, int(self.get_parameter("publish_every_n").value))
        if (self._frame_i % publish_every_n) != 0:
            return

        with self._lock:
            self._latest_pair = (frame_i, color_msg, depth_msg)
            self._new_pair_event.set()

    def destroy_node(self):
        self._stop_event.set()
        self._new_pair_event.set()
        try:
            if self._worker.is_alive():
                self._worker.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()

    def _worker_loop(self):
        import cv2
        import numpy as np

        from yolo_depth_pcd.process_frame_server import _clip_bbox

        last_saved_i = -1
        while not self._stop_event.is_set():
            self._new_pair_event.wait(timeout=0.2)
            self._new_pair_event.clear()
            if self._stop_event.is_set():
                break

            with self._lock:
                pair = self._latest_pair
                self._latest_pair = None

            if pair is None:
                continue

            frame_i, color_msg, depth_msg = pair
            if self._color_info is None:
                continue

            t0 = time.time()

            color_bgr = self._imgmsg_to_bgr(color_msg)
            if color_bgr is None:
                continue
            color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

            depth_u16 = self._imgmsg_to_depth_u16(depth_msg)
            if depth_u16 is None:
                continue

            depth_scale = float(self.get_parameter("depth_scale").value)
            depth_m = depth_u16.astype(np.float32) / float(depth_scale)

            h = int(color_bgr.shape[0])
            w = int(color_bgr.shape[1])

            yolo_every_n = max(1, int(self.get_parameter("yolo_every_n").value))
            if (frame_i % yolo_every_n) != 0:
                det = None
            else:
                det = self._run_yolo_one(color_rgb)

            frame_id_cloud = self._viz_frame(color_msg)
            frame_id_img = self._viz_image_frame(color_msg)

            vis = color_bgr.copy()
            highlight_bbox = None
            if det is not None:
                x1, y1, x2, y2 = det["xyxy"]
                x1i, y1i, x2i, y2i = _clip_bbox(x1, y1, x2, y2, w, h)
                highlight_bbox = (x1i, y1i, x2i, y2i)

                self.get_logger().info(
                    f"[stage1] detected={det['name']} conf={det['conf']:.3f} bbox=({x1i},{y1i},{x2i},{y2i})"
                )

                cv2.rectangle(vis, (int(x1i), int(y1i)), (int(x2i), int(y2i)), (0, 0, 255), 2)
                cv2.putText(
                    vis,
                    f"{det['name']} {det['conf']:.2f}",
                    (int(x1i), max(0, int(y1i) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
            else:
                self.get_logger().debug("[stage1] no detection")

            if self._viz_enabled:
                self.pub_yolo_rgb.publish(self._imgmsg_from_bgr(self, frame_id_img, vis))

            cloud_msg = self._build_colored_pointcloud(
                frame_id_cloud=frame_id_cloud,
                color_bgr=color_bgr,
                depth_m=depth_m,
                highlight_bbox=highlight_bbox,
            )
            if cloud_msg is not None:
                self.pub_cloud.publish(cloud_msg)

            save_images = bool(self.get_parameter("save_images").value)
            save_every_n = max(1, int(self.get_parameter("save_every_n").value))
            if save_images and (frame_i % save_every_n == 0) and last_saved_i != frame_i:
                last_saved_i = frame_i
                self._save_stage_images(color_bgr=color_bgr, vis_bgr=vis, highlight_bbox=highlight_bbox)

            dt_ms = (time.time() - t0) * 1000.0
            self.get_logger().debug(f"pipeline dt={dt_ms:.1f}ms")

    def _save_stage_images(self, color_bgr, vis_bgr, highlight_bbox):
        import cv2

        out_dir = Path(str(self.get_parameter("save_dir").value)).expanduser()
        out_dir.mkdir(parents=True, exist_ok=True)

        cv2.imwrite(str(out_dir / "stage0_raw.png"), color_bgr)
        cv2.imwrite(str(out_dir / "stage1_yolo.png"), vis_bgr)
        if highlight_bbox is not None:
            x1i, y1i, x2i, y2i = highlight_bbox
            roi = color_bgr[y1i : y2i + 1, x1i : x2i + 1].copy()
            cv2.imwrite(str(out_dir / "stage2_roi.png"), roi)

    def _build_colored_pointcloud(self, frame_id_cloud: str, color_bgr, depth_m, highlight_bbox):
        import numpy as np

        from sensor_msgs.msg import PointCloud2, PointField

        fx = float(self._color_info.k[0])
        fy = float(self._color_info.k[4])
        cx = float(self._color_info.k[2])
        cy = float(self._color_info.k[5])

        h, w = depth_m.shape[:2]
        cloud_stride = max(1, int(self.get_parameter("cloud_stride").value))

        us = np.arange(0, w, cloud_stride, dtype=np.int32)
        vs = np.arange(0, h, cloud_stride, dtype=np.int32)
        uu, vv = np.meshgrid(us, vs)
        z = depth_m[vv, uu]
        valid = (z > 0.0) & np.isfinite(z)
        if not bool(np.any(valid)):
            return None

        uu = uu[valid].astype(np.float32)
        vv = vv[valid].astype(np.float32)
        z = z[valid].astype(np.float32)

        x = (uu - float(cx)) * z / float(fx)
        y = (vv - float(cy)) * z / float(fy)

        rgb = color_bgr[vv.astype(np.int32), uu.astype(np.int32), :].astype(np.uint8)

        if highlight_bbox is not None:
            x1i, y1i, x2i, y2i = highlight_bbox
            in_box = (
                (uu >= float(x1i))
                & (uu <= float(x2i))
                & (vv >= float(y1i))
                & (vv <= float(y2i))
            )
            if bool(np.any(in_box)):
                rgb[in_box] = np.array([0, 0, 255], dtype=np.uint8)
                self.get_logger().info(f"[stage5] highlight points={int(np.count_nonzero(in_box))}")
            else:
                self.get_logger().info("[stage5] highlight points=0")
        else:
            self.get_logger().debug("[stage5] no highlight")

        pts = np.zeros(
            (x.shape[0],),
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgb", np.float32)],
        )
        pts["x"] = x
        pts["y"] = y
        pts["z"] = z

        rgb_u32 = (rgb[:, 2].astype(np.uint32) << 16) | (rgb[:, 1].astype(np.uint32) << 8) | rgb[:, 0].astype(np.uint32)
        pts["rgb"] = rgb_u32.view(np.float32)

        msg = PointCloud2()
        msg.header = self._header(frame_id_cloud)
        msg.height = 1
        msg.width = int(pts.shape[0])
        msg.is_bigendian = False
        msg.is_dense = False
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.data = pts.tobytes()
        return msg

    def _header(self, frame_id: str):
        from std_msgs.msg import Header

        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = str(frame_id)
        return h

    def _imgmsg_to_bgr(self, msg: Image):
        import numpy as np
        import cv2

        enc = str(msg.encoding).lower().strip()
        h = int(msg.height)
        w = int(msg.width)
        step = int(msg.step)

        if h <= 0 or w <= 0 or step <= 0:
            self.get_logger().warn("Invalid image dimensions")
            return None

        if enc not in ("bgr8", "rgb8"):
            self.get_logger().warn(f"Unsupported color encoding: {msg.encoding} (expected bgr8/rgb8)")
            return None

        row = np.frombuffer(msg.data, dtype=np.uint8)
        if row.size < h * step:
            self.get_logger().warn("Image data size is smaller than height*step")
            return None

        img = row[: h * step].reshape((h, step))
        img = img[:, : w * 3].reshape((h, w, 3))
        if enc == "bgr8":
            return img.copy()
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def _imgmsg_to_depth_u16(self, msg: Image):
        import numpy as np

        enc = str(msg.encoding).lower().strip()
        h = int(msg.height)
        w = int(msg.width)
        step = int(msg.step)

        if h <= 0 or w <= 0 or step <= 0:
            self.get_logger().warn("Invalid depth image dimensions")
            return None

        if enc != "16uc1":
            return None

        row = np.frombuffer(msg.data, dtype=np.uint8)
        if row.size < h * step:
            self.get_logger().warn("Depth data size is smaller than height*step")
            return None

        img = row[: h * step].reshape((h, step))
        img_u16 = img[:, : w * 2].view(np.uint16).reshape((h, w))
        return img_u16.copy()

    def _viz_frame(self, msg: Image) -> str:
        override = str(self.get_parameter("viz_frame_id").value).strip()
        if override:
            return override
        if msg.header.frame_id:
            return str(msg.header.frame_id)
        if self._color_info is not None and self._color_info.header.frame_id:
            return str(self._color_info.header.frame_id)
        return "camera"

    def _viz_image_frame(self, msg: Image) -> str:
        override = str(self.get_parameter("viz_image_frame_id").value).strip()
        if override:
            return override
        if msg.header.frame_id:
            return str(msg.header.frame_id)
        return self._viz_frame(msg)

    def _map_color_uv_to_depth_uv(self, u: float, v: float, cw: int, ch: int, dw: int, dh: int):
        mode = str(self.get_parameter("depth_to_color_mode").value).strip().lower()
        if cw == dw and ch == dh:
            uu = int(round(u))
            vv = int(round(v))
        else:
            if mode != "scale":
                self.get_logger().warn(
                    "Depth resolution differs from color. Set RealSense to publish aligned depth to color for better accuracy. "
                    "Falling back to scale mapping."
                )
            uu = int(round(u * float(dw) / float(cw)))
            vv = int(round(v * float(dh) / float(ch)))

        uu = max(0, min(int(uu), dw - 1))
        vv = max(0, min(int(vv), dh - 1))
        return uu, vv

    def _depth_to_xyz_cloud(self, depth_m, fx, fy, cx, cy, stride: int):
        import numpy as np

        h, w = depth_m.shape[:2]
        us = np.arange(0, w, stride, dtype=np.int32)
        vs = np.arange(0, h, stride, dtype=np.int32)
        uu, vv = np.meshgrid(us, vs)
        z = depth_m[vv, uu]
        valid = (z > 0.0) & np.isfinite(z)
        uu = uu[valid].astype(np.float32)
        vv = vv[valid].astype(np.float32)
        z = z[valid].astype(np.float32)

        x = (uu - float(cx)) * z / float(fx)
        y = (vv - float(cy)) * z / float(fy)
        xyz = np.stack([x, y, z], axis=1)
        return xyz


def main(args=None):
    rclpy.init(args=args)
    node = LivePipelineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
