from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import TransformBroadcaster


class RealtimeYoloPcdTfNode(Node):
    def __init__(self):
        super().__init__('realtime_yolo_pcd_tf')

        self.declare_parameter('weights', '')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('class_name', 'handle')
        self.declare_parameter('class_id', -1)
        self.declare_parameter('device', '')
        self.declare_parameter('half', False)

        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('color_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_scale', 1000.0)
        self.declare_parameter('sync_slop_sec', 0.3)

        self.declare_parameter('publish_every_n', 1)
        self.declare_parameter('yolo_every_n', 2)
        self.declare_parameter('cloud_stride', 4)

        self.declare_parameter('extrinsics_json', '/home/unitree/detect/robot_center_to_camera.json')
        self.declare_parameter('tf_publish_hz', 10.0)
        self.declare_parameter('output_frame', 'robot_center')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self._color_info: Optional[CameraInfo] = None
        self._latest_color_msg: Optional[Image] = None
        self._latest_depth_msg: Optional[Image] = None

        self._frame_i = 0

        self._sub_color = self.create_subscription(Image, str(self.get_parameter('color_topic').value), self._on_color, qos)
        self._sub_depth = self.create_subscription(Image, str(self.get_parameter('depth_topic').value), self._on_depth, qos)
        self._sub_info = self.create_subscription(CameraInfo, str(self.get_parameter('color_info_topic').value), self._on_info, qos)

        self.pub_rgb = self.create_publisher(Image, '/realtime_yolo/rgb', 1)
        self.pub_cloud = self.create_publisher(PointCloud2, '/realtime_yolo/pointcloud', 1)
        self.pub_bbox2d = self.create_publisher(Float32MultiArray, '/realtime_yolo/bbox2d', 1)
        self.pub_points3d_cam = self.create_publisher(Float32MultiArray, '/realtime_yolo/points3d_camera', 1)
        self.pub_points3d_robot = self.create_publisher(Float32MultiArray, '/realtime_yolo/points3d_robot', 1)
        self.pub_markers = self.create_publisher(MarkerArray, '/realtime_yolo/markers', 1)
        self.pub_status = self.create_publisher(String, '/realtime_yolo/status', 10)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._T_cam_robot = None
        self._T_robot_cam = None
        self._load_extrinsics()

        tf_hz = float(self.get_parameter('tf_publish_hz').value)
        if tf_hz > 0:
            self._tf_timer = self.create_timer(1.0 / tf_hz, self._publish_tf)

        self._lock = threading.Lock()
        self._latest_pair: Optional[Tuple[int, Image, Image]] = None
        self._new_pair_event = threading.Event()
        self._stop_event = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)

        self._model = None
        self._load_model()

        self._worker.start()

    def destroy_node(self):
        self._stop_event.set()
        self._new_pair_event.set()
        try:
            if self._worker.is_alive():
                self._worker.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()

    def _on_info(self, msg: CameraInfo):
        self._color_info = msg

    def _on_color(self, msg: Image):
        self._latest_color_msg = msg
        self._try_pair()

    def _on_depth(self, msg: Image):
        self._latest_depth_msg = msg
        self._try_pair()

    def _t(self, msg: Image) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _try_pair(self):
        if self._color_info is None:
            return
        c = self._latest_color_msg
        d = self._latest_depth_msg
        if c is None or d is None:
            return

        slop = float(self.get_parameter('sync_slop_sec').value)
        if abs(self._t(c) - self._t(d)) > max(0.0, slop):
            return

        self._frame_i += 1
        frame_i = int(self._frame_i)

        publish_every_n = max(1, int(self.get_parameter('publish_every_n').value))
        if (frame_i % publish_every_n) != 0:
            return

        with self._lock:
            self._latest_pair = (frame_i, c, d)
            self._new_pair_event.set()

    def _load_model(self):
        weights = str(self.get_parameter('weights').value).strip()
        if not weights:
            self._publish_status('weights parameter empty; node will run but no detections will be produced')
            return

        try:
            import torch
            from ultralytics.nn.tasks import DetectionModel

            if hasattr(torch, 'serialization') and hasattr(torch.serialization, 'add_safe_globals'):
                torch.serialization.add_safe_globals([DetectionModel])

            if hasattr(torch, 'load') and not getattr(torch.load, '_realtime_yolo_pcd_tf_patched', False):
                _orig = torch.load

                def _torch_load_no_weights_only(*args, **kwargs):
                    if 'weights_only' not in kwargs:
                        kwargs['weights_only'] = False
                    return _orig(*args, **kwargs)

                _torch_load_no_weights_only._realtime_yolo_pcd_tf_patched = True  # type: ignore[attr-defined]
                torch.load = _torch_load_no_weights_only  # type: ignore[assignment]
        except Exception:
            pass

        from ultralytics import YOLO

        self._model = YOLO(weights)

        dev = str(self.get_parameter('device').value).strip() or None
        half = bool(self.get_parameter('half').value)
        try:
            if dev is not None:
                self._model.to(dev)
            if half:
                m = getattr(self._model, 'model', None)
                if m is not None and hasattr(m, 'half'):
                    m.half()
        except Exception as e:
            self._publish_status(f'failed to set device/half: {e}')

        self._publish_status(f'loaded YOLO weights: {weights}')

    def _run_yolo_one(self, img_rgb):
        if self._model is None:
            return None

        conf = float(self.get_parameter('conf').value)
        class_name = str(self.get_parameter('class_name').value).strip() or None
        class_id_raw = int(self.get_parameter('class_id').value)
        class_id = None if class_id_raw < 0 else class_id_raw

        res = self._model.predict(source=img_rgb, conf=float(conf), max_det=50, verbose=False)
        if not res:
            return None
        r0 = res[0]
        if r0.boxes is None:
            return None

        names = getattr(r0, 'names', {})
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
            d = {'cls_id': cid, 'name': cname, 'conf': float(confs[i]), 'xyxy': (x1, y1, x2, y2)}
            if best is None or d['conf'] > best['conf']:
                best = d
        return best

    def _load_extrinsics(self):
        try:
            p = Path(str(self.get_parameter('extrinsics_json').value)).expanduser()
            if not p.exists():
                self._publish_status(f'extrinsics_json not found: {p}')
                return
            data = json.loads(p.read_text(encoding='utf-8'))
            t = data.get('t_xyz_m', None)
            q = data.get('q_xyzw', None)
            if t is None or q is None or len(t) != 3 or len(q) != 4:
                self._publish_status('extrinsics_json missing t_xyz_m/q_xyzw')
                return

            # User confirmed this JSON represents camera -> robot.
            self._T_cam_robot = (t, q)
            self._T_robot_cam = self._invert_transform(t, q)

            self._publish_status(
                f"loaded extrinsics (assumed camera->robot) file_parent={data.get('parent_frame')} file_child={data.get('child_frame')}"
            )
        except Exception as e:
            self._publish_status(f'failed to load extrinsics: {e}')

    def _invert_transform(self, t_xyz, q_xyzw):
        import numpy as np

        t = np.asarray([float(v) for v in t_xyz], dtype=np.float32).reshape(3)
        q = [float(v) for v in q_xyzw]
        R = self._quat_to_R(q)
        Rt = R.T
        t_inv = -(Rt @ t)

        # For unit quaternion, inverse is conjugate.
        qx, qy, qz, qw = q
        q_inv = (-qx, -qy, -qz, qw)
        return (float(t_inv[0]), float(t_inv[1]), float(t_inv[2])), (float(q_inv[0]), float(q_inv[1]), float(q_inv[2]), float(q_inv[3]))

    def _publish_tf(self):
        if self._T_robot_cam is None:
            return
        t, q = self._T_robot_cam

        # Publish robot -> camera (more convenient TF tree rooted at robot)
        parent = str(self.get_parameter('output_frame').value)
        child = str(self.get_parameter('camera_frame').value)

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = parent
        ts.child_frame_id = child
        ts.transform.translation.x = float(t[0])
        ts.transform.translation.y = float(t[1])
        ts.transform.translation.z = float(t[2])
        ts.transform.rotation.x = float(q[0])
        ts.transform.rotation.y = float(q[1])
        ts.transform.rotation.z = float(q[2])
        ts.transform.rotation.w = float(q[3])
        self._tf_broadcaster.sendTransform(ts)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = str(text)
        self.pub_status.publish(msg)
        self.get_logger().info(str(text))

    def _imgmsg_to_bgr(self, msg: Image):
        import numpy as np
        import cv2

        enc = str(msg.encoding).lower().strip()
        h = int(msg.height)
        w = int(msg.width)
        step = int(msg.step)

        if h <= 0 or w <= 0 or step <= 0:
            return None

        if enc not in ('bgr8', 'rgb8'):
            return None

        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if buf.size < h * step:
            return None

        img = buf[: h * step].reshape((h, step))
        img = img[:, : w * 3].reshape((h, w, 3))
        if enc == 'bgr8':
            return img.copy()
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def _imgmsg_to_depth_u16(self, msg: Image):
        import numpy as np

        enc = str(msg.encoding).lower().strip()
        h = int(msg.height)
        w = int(msg.width)
        step = int(msg.step)

        if h <= 0 or w <= 0 or step <= 0:
            return None

        if enc != '16uc1':
            return None

        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if buf.size < h * step:
            return None

        img = buf[: h * step].reshape((h, step))
        u16 = img[:, : w * 2].view(np.uint16).reshape((h, w))
        return u16.copy()

    def _imgmsg_from_bgr(self, frame_id: str, img_bgr) -> Image:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(frame_id)
        msg.height = int(img_bgr.shape[0])
        msg.width = int(img_bgr.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = int(img_bgr.shape[1] * 3)
        msg.data = img_bgr.tobytes()
        return msg

    def _clip_bbox(self, x1, y1, x2, y2, w, h):
        x1i = max(0, min(int(round(x1)), w - 1))
        y1i = max(0, min(int(round(y1)), h - 1))
        x2i = max(0, min(int(round(x2)), w - 1))
        y2i = max(0, min(int(round(y2)), h - 1))
        if x2i < x1i:
            x1i, x2i = x2i, x1i
        if y2i < y1i:
            y1i, y2i = y2i, y1i
        return x1i, y1i, x2i, y2i

    def _sample_depth(self, depth_m, u: int, v: int) -> float:
        import numpy as np

        h, w = depth_m.shape[:2]
        uu = max(0, min(int(u), w - 1))
        vv = max(0, min(int(v), h - 1))
        z = float(depth_m[vv, uu])
        if not (z > 0.0) or not np.isfinite(z):
            return float('nan')
        return z

    def _pixel_to_cam(self, u: float, v: float, z: float):
        fx = float(self._color_info.k[0])
        fy = float(self._color_info.k[4])
        cx = float(self._color_info.k[2])
        cy = float(self._color_info.k[5])
        x = (float(u) - cx) * float(z) / fx
        y = (float(v) - cy) * float(z) / fy
        return x, y, z

    def _quat_to_R(self, q_xyzw):
        import numpy as np

        x, y, z, w = [float(v) for v in q_xyzw]
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z
        R = np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float32,
        )
        return R

    def _cam_to_robot(self, p_cam_xyz):
        import numpy as np

        if self._T_cam_robot is None:
            return None
        t, q = self._T_cam_robot
        R = self._quat_to_R(q)
        p = np.asarray(p_cam_xyz, dtype=np.float32).reshape(3)
        tt = np.asarray(t, dtype=np.float32).reshape(3)
        pr = (R @ p) + tt
        return float(pr[0]), float(pr[1]), float(pr[2])

    def _build_cloud(self, frame_id: str, color_bgr, depth_m, highlight_bbox):
        import numpy as np

        if self._color_info is None:
            return None

        fx = float(self._color_info.k[0])
        fy = float(self._color_info.k[4])
        cx = float(self._color_info.k[2])
        cy = float(self._color_info.k[5])

        h, w = depth_m.shape[:2]
        stride = max(1, int(self.get_parameter('cloud_stride').value))

        us = np.arange(0, w, stride, dtype=np.int32)
        vs = np.arange(0, h, stride, dtype=np.int32)
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
                rgb[in_box] = (0, 0, 255)

        pts = np.zeros(
            (x.shape[0],),
            dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)],
        )
        pts['x'] = x
        pts['y'] = y
        pts['z'] = z

        rgb_u32 = (rgb[:, 2].astype(np.uint32) << 16) | (rgb[:, 1].astype(np.uint32) << 8) | rgb[:, 0].astype(np.uint32)
        pts['rgb'] = rgb_u32.view(np.float32)

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(frame_id)
        msg.height = 1
        msg.width = int(pts.shape[0])
        msg.is_bigendian = False
        msg.is_dense = False
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.data = pts.tobytes()
        return msg

    def _publish_bbox_msgs(self, bbox_xyxy, conf: float, pts_cam, pts_robot):
        x1, y1, x2, y2 = [float(v) for v in bbox_xyxy]

        bb = Float32MultiArray()
        bb.data = [x1, y1, x2, y2, float(conf)]
        self.pub_bbox2d.publish(bb)

        cam = Float32MultiArray()
        cam.data = [float(v) for p in pts_cam for v in p]
        self.pub_points3d_cam.publish(cam)

        rob = Float32MultiArray()
        rob.data = [float(v) for p in pts_robot for v in p]
        self.pub_points3d_robot.publish(rob)

    def _publish_markers(self, frame_id: str, corners_xyz, mid_base: int = 0):
        ma = MarkerArray()

        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = str(frame_id)
        m.ns = 'bbox3d'
        m.id = int(mid_base)
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.01
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0

        from geometry_msgs.msg import Point

        pts = []
        for p in corners_xyz + [corners_xyz[0]]:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = float(p[2])
            pts.append(pt)
        m.points = pts

        ma.markers.append(m)
        self.pub_markers.publish(ma)

    def _worker_loop(self):
        import cv2
        import numpy as np

        last_det = None
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

            color_bgr = self._imgmsg_to_bgr(color_msg)
            if color_bgr is None:
                continue

            depth_u16 = self._imgmsg_to_depth_u16(depth_msg)
            if depth_u16 is None:
                continue

            depth_scale = float(self.get_parameter('depth_scale').value)
            depth_m = depth_u16.astype(np.float32) / float(depth_scale)

            vis = color_bgr.copy()

            det = None
            yolo_every_n = max(1, int(self.get_parameter('yolo_every_n').value))
            if (frame_i % yolo_every_n) == 0:
                rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
                det = self._run_yolo_one(rgb)
                if det is not None:
                    last_det = det
            else:
                det = last_det

            h, w = int(color_bgr.shape[0]), int(color_bgr.shape[1])
            highlight_bbox = None

            camera_frame = str(self.get_parameter('camera_frame').value)
            output_frame = str(self.get_parameter('output_frame').value)

            if det is not None:
                x1, y1, x2, y2 = det['xyxy']
                x1i, y1i, x2i, y2i = self._clip_bbox(x1, y1, x2, y2, w, h)
                highlight_bbox = (x1i, y1i, x2i, y2i)

                cv2.rectangle(vis, (x1i, y1i), (x2i, y2i), (0, 0, 255), 2)
                cv2.putText(
                    vis,
                    f"{det['name']} {det['conf']:.2f}",
                    (x1i, max(0, y1i - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

                uvs = [
                    (x1i, y1i),
                    (x2i, y1i),
                    (x2i, y2i),
                    (x1i, y2i),
                    ((x1i + x2i) // 2, (y1i + y2i) // 2),
                ]
                pts_cam = []
                pts_robot = []
                for (uu, vv) in uvs:
                    z = self._sample_depth(depth_m, uu, vv)
                    if not np.isfinite(z):
                        pts_cam.append((float('nan'), float('nan'), float('nan')))
                        pts_robot.append((float('nan'), float('nan'), float('nan')))
                        continue
                    pc = self._pixel_to_cam(float(uu), float(vv), float(z))
                    pr = self._cam_to_robot(pc)
                    pts_cam.append(tuple(pc))
                    if pr is None:
                        pts_robot.append((float('nan'), float('nan'), float('nan')))
                    else:
                        pts_robot.append(tuple(pr))

                self._publish_bbox_msgs((x1i, y1i, x2i, y2i), float(det['conf']), pts_cam, pts_robot)

                corners_cam = pts_cam[:4]
                corners_robot = pts_robot[:4]

                if bool(np.isfinite(np.asarray(corners_cam, dtype=np.float32)).all()):
                    self._publish_markers(camera_frame, corners_cam, mid_base=0)
                if bool(np.isfinite(np.asarray(corners_robot, dtype=np.float32)).all()):
                    self._publish_markers(output_frame, corners_robot, mid_base=1000)

                self._publish_status(
                    f"det={det['name']} conf={det['conf']:.3f} center_cam={pts_cam[4]} center_robot={pts_robot[4]}"
                )

            self.pub_rgb.publish(self._imgmsg_from_bgr(camera_frame, vis))

            cloud = self._build_cloud(camera_frame, color_bgr, depth_m, highlight_bbox)
            if cloud is not None:
                self.pub_cloud.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeYoloPcdTfNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
