from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


def _apriltag_dict(dict_name: str):
    import cv2

    name = dict_name.strip().upper()
    mapping = {
        "APRILTAG_36H11": cv2.aruco.DICT_APRILTAG_36h11,
        "APRILTAG_25H9": cv2.aruco.DICT_APRILTAG_25h9,
        "APRILTAG_16H5": cv2.aruco.DICT_APRILTAG_16h5,
    }
    if name not in mapping:
        raise ValueError(f"Unsupported dict_name={dict_name}. Use: {sorted(mapping.keys())}")
    return cv2.aruco.getPredefinedDictionary(mapping[name])


def _mat_from_quat_xyzw(q):
    import numpy as np

    x, y, z, w = [float(v) for v in q]
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n <= 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / n, y / n, z / n, w / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _quat_xyzw_from_mat(R):
    import numpy as np

    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    tr = float(np.trace(R))
    if tr > 0.0:
        S = (tr + 1.0) ** 0.5 * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = (1.0 + R[0, 0] - R[1, 1] - R[2, 2]) ** 0.5 * 2.0
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = (1.0 + R[1, 1] - R[0, 0] - R[2, 2]) ** 0.5 * 2.0
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = (1.0 + R[2, 2] - R[0, 0] - R[1, 1]) ** 0.5 * 2.0
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S

    q = np.array([x, y, z, w], dtype=np.float64)
    q /= max(1e-12, float(np.linalg.norm(q)))
    return q


def _T(R, t):
    import numpy as np

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def _invert_T(Tm):
    import numpy as np

    Tm = np.asarray(Tm, dtype=np.float64).reshape(4, 4)
    R = Tm[:3, :3]
    t = Tm[:3, 3]
    Ri = R.T
    ti = -Ri @ t
    return _T(Ri, ti)


def _imgmsg_to_bgr(msg: Image):
    import numpy as np
    import cv2

    enc = str(msg.encoding).lower().strip()
    h = int(msg.height)
    w = int(msg.width)
    step = int(msg.step)

    if h <= 0 or w <= 0 or step <= 0:
        return None

    if enc not in ("bgr8", "rgb8"):
        return None

    row = np.frombuffer(msg.data, dtype=np.uint8)
    if row.size < h * step:
        return None

    img = row[: h * step].reshape((h, step))
    img = img[:, : w * 3].reshape((h, w, 3))
    if enc == "bgr8":
        return img.copy()
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


class AprilTagStaticCalibNode(Node):
    def __init__(self):
        super().__init__("apriltag_static_calib_node")

        self.declare_parameter("board_meta", "")
        self.declare_parameter("dict", "APRILTAG_36H11")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")

        self.declare_parameter("assume_board_parallel_robot", False)
        self.declare_parameter("robot_board_center_xyz", "")
        self.declare_parameter("robot_tag_id", -1)
        self.declare_parameter("robot_tag_xyz", "")
        self.declare_parameter("robot_board_xyz", "0,0,0")
        self.declare_parameter("robot_board_quat_xyzw", "0,0,0,1")
        self.declare_parameter("parent_frame", "robot_center")
        self.declare_parameter("child_frame", "")

        self.declare_parameter("save_camera_info_json", "")

        self.declare_parameter("save_robot_cam_json", "")
        self.declare_parameter("save_robot_cam_json_every_n", 1)

        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate_hz", 10.0)

        board_meta_path = str(self.get_parameter("board_meta").value).strip()
        if not board_meta_path:
            raise RuntimeError("Parameter 'board_meta' is empty")
        self._board_meta = json.loads(Path(board_meta_path).read_text(encoding="utf-8"))

        self._tag_map = {int(t["tag_id"]): t for t in self._board_meta.get("tags", [])}
        if not self._tag_map:
            raise RuntimeError("Board meta has no tags")

        self._dict_name = str(self.get_parameter("dict").value).strip()

        assume_parallel = bool(self.get_parameter("assume_board_parallel_robot").value)
        center_xyz_raw = str(self.get_parameter("robot_board_center_xyz").value).strip()
        tag_xyz_raw = str(self.get_parameter("robot_tag_xyz").value).strip()
        if assume_parallel:
            import numpy as np

            R_robot_board = [
                [1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
                [0.0, 1.0, 0.0],
            ]
            Rrb = np.asarray(R_robot_board, dtype=np.float64).reshape(3, 3)

            if tag_xyz_raw:
                tag_xyz = [float(v) for v in tag_xyz_raw.split(",")]
                if len(tag_xyz) != 3:
                    raise RuntimeError("robot_tag_xyz must be 3 values")

                tag_id = int(self.get_parameter("robot_tag_id").value)
                if tag_id < 0:
                    tag_id = int(sorted(self._tag_map.keys())[0])
                if tag_id not in self._tag_map:
                    raise RuntimeError(f"robot_tag_id={tag_id} not in board_meta")

                c2 = np.asarray(self._tag_map[tag_id]["corners_xy_m"], dtype=np.float64).reshape(4, 2)
                p_tag_center_board = np.array([float(c2[:, 0].mean()), float(c2[:, 1].mean()), 0.0], dtype=np.float64)

                t_tag = np.asarray(tag_xyz, dtype=np.float64).reshape(3)
                t_origin = t_tag - (Rrb @ p_tag_center_board)
                self._T_robot_board = _T(Rrb, t_origin)
            else:
                if not center_xyz_raw:
                    raise RuntimeError(
                        "assume_board_parallel_robot=true requires robot_tag_xyz='x,y,z' (recommended) or robot_board_center_xyz='x,y,z'"
                    )

                center_xyz = [float(v) for v in center_xyz_raw.split(",")]
                if len(center_xyz) != 3:
                    raise RuntimeError("robot_board_center_xyz must be 3 values")

                paper = self._board_meta.get("paper", {})
                w = float(paper.get("width_m", 0.0))
                h = float(paper.get("height_m", 0.0))
                if w <= 0.0 or h <= 0.0:
                    raise RuntimeError(
                        "board_meta.paper.width_m/height_m must be > 0 when assume_board_parallel_robot=true and using robot_board_center_xyz"
                    )

                p_board_center = np.array([0.5 * w, 0.5 * h, 0.0], dtype=np.float64)
                t_center = np.asarray(center_xyz, dtype=np.float64).reshape(3)
                t_origin = t_center - Rrb @ p_board_center
                self._T_robot_board = _T(Rrb, t_origin)
        else:
            xyz = [float(v) for v in str(self.get_parameter("robot_board_xyz").value).split(",")]
            quat = [float(v) for v in str(self.get_parameter("robot_board_quat_xyzw").value).split(",")]
            if len(xyz) != 3 or len(quat) != 4:
                raise RuntimeError("robot_board_xyz must be 3 values; robot_board_quat_xyzw must be 4 values")
            self._T_robot_board = _T(_mat_from_quat_xyzw(quat), xyz)

        self._parent_frame = str(self.get_parameter("parent_frame").value).strip() or "robot_center"
        self._child_frame_override = str(self.get_parameter("child_frame").value).strip()

        self._tfb = TransformBroadcaster(self)

        self._camera_info_dumped = False

        self._save_i = 0

        self._last_T_robot_cam = None
        self._last_pub_time = self.get_clock().now()

        from message_filters import ApproximateTimeSynchronizer, Subscriber
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        self._sub_img = Subscriber(self, Image, str(self.get_parameter("color_topic").value), qos_profile=qos)
        self._sub_info = Subscriber(self, CameraInfo, str(self.get_parameter("camera_info_topic").value), qos_profile=qos)

        self._ats = ApproximateTimeSynchronizer([self._sub_img, self._sub_info], queue_size=10, slop=0.1)
        self._ats.registerCallback(self._on_pair)

    def _estimate_T_cam_board(self, img_bgr, cam_info: CameraInfo):
        import cv2
        import numpy as np

        K = np.asarray(cam_info.k, dtype=np.float64).reshape(3, 3)
        D = np.asarray(cam_info.d, dtype=np.float64).reshape(-1) if len(cam_info.d) > 0 else np.zeros((5,), dtype=np.float64)

        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        dct = _apriltag_dict(self._dict_name)
        params = cv2.aruco.DetectorParameters()

        detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(dct, params)

        if detector is not None:
            corners_list, ids, _ = detector.detectMarkers(gray)
        else:
            corners_list, ids, _ = cv2.aruco.detectMarkers(gray, dct, parameters=params)

        if ids is None or len(ids) == 0:
            return None, []

        ids = ids.reshape(-1).astype(int)
        corners_list = [np.asarray(c, dtype=np.float64).reshape(-1, 2) for c in corners_list]

        obj_pts = []
        img_pts = []
        used_ids = []

        for tid, c2 in zip(ids.tolist(), corners_list):
            if tid not in self._tag_map:
                continue
            t = self._tag_map[tid]
            c3 = np.asarray(t["corners_xy_m"], dtype=np.float64).reshape(4, 2)
            c3 = np.hstack([c3, np.zeros((4, 1), dtype=np.float64)])
            obj_pts.append(c3)
            img_pts.append(c2)
            used_ids.append(tid)

        if not obj_pts:
            return None, []

        obj_pts = np.vstack(obj_pts).reshape(-1, 3)
        img_pts = np.vstack(img_pts).reshape(-1, 2)

        ok, rvec, tvec = cv2.solvePnP(
            objectPoints=obj_pts,
            imagePoints=img_pts,
            cameraMatrix=K,
            distCoeffs=D,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None, used_ids

        R, _ = cv2.Rodrigues(rvec)
        T_cam_board = _T(R, tvec.reshape(3))
        return T_cam_board, used_ids

    def _on_pair(self, img_msg: Image, info_msg: CameraInfo):
        now = self.get_clock().now()

        if not self._camera_info_dumped:
            out_path = str(self.get_parameter("save_camera_info_json").value).strip()
            if out_path:
                self._dump_camera_info_json(Path(out_path), info_msg)
                self._camera_info_dumped = True

        img_bgr = _imgmsg_to_bgr(img_msg)
        if img_bgr is None:
            return

        T_cam_board, used_ids = self._estimate_T_cam_board(img_bgr, info_msg)
        if T_cam_board is None:
            return

        T_board_cam = _invert_T(T_cam_board)
        T_robot_cam = self._T_robot_board @ T_board_cam
        self._last_T_robot_cam = T_robot_cam

        child_frame = self._child_frame_override or (str(info_msg.header.frame_id).strip() if info_msg.header.frame_id else "camera")

        q = _quat_xyzw_from_mat(T_robot_cam[:3, :3])
        t = T_robot_cam[:3, 3]

        self.get_logger().info(
            f"tags={used_ids} parent={self._parent_frame} child={child_frame} t=[{t[0]:.4f},{t[1]:.4f},{t[2]:.4f}] q=[{q[0]:.4f},{q[1]:.4f},{q[2]:.4f},{q[3]:.4f}]"
        )

        self._maybe_save_robot_cam_json(now, used_ids, child_frame, t, q)

        publish_tf = bool(self.get_parameter("publish_tf").value)
        if not publish_tf:
            return

        rate = float(self.get_parameter("publish_rate_hz").value)
        rate = max(0.1, rate)
        if (now - self._last_pub_time).nanoseconds < int(1e9 / rate):
            return
        self._last_pub_time = now

        tfm = TransformStamped()
        tfm.header.stamp = now.to_msg()
        tfm.header.frame_id = self._parent_frame
        tfm.child_frame_id = child_frame
        tfm.transform.translation.x = float(t[0])
        tfm.transform.translation.y = float(t[1])
        tfm.transform.translation.z = float(t[2])
        tfm.transform.rotation.x = float(q[0])
        tfm.transform.rotation.y = float(q[1])
        tfm.transform.rotation.z = float(q[2])
        tfm.transform.rotation.w = float(q[3])
        self._tfb.sendTransform(tfm)

    def _maybe_save_robot_cam_json(self, now, used_ids, child_frame: str, t_xyz, q_xyzw):
        out_path = str(self.get_parameter("save_robot_cam_json").value).strip()
        if not out_path:
            return

        every_n = max(1, int(self.get_parameter("save_robot_cam_json_every_n").value))
        self._save_i += 1
        if (self._save_i % every_n) != 0:
            return

        data = {
            "stamp": {"sec": int(now.nanoseconds // 1_000_000_000), "nanosec": int(now.nanoseconds % 1_000_000_000)},
            "parent_frame": str(self._parent_frame),
            "child_frame": str(child_frame),
            "t_xyz_m": [float(t_xyz[0]), float(t_xyz[1]), float(t_xyz[2])],
            "q_xyzw": [float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2]), float(q_xyzw[3])],
            "used_tag_ids": [int(x) for x in used_ids],
        }
        p = Path(out_path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(data, indent=2), encoding="utf-8")

    def _dump_camera_info_json(self, out_path: Path, msg: CameraInfo):
        data = {
            "width": int(msg.width),
            "height": int(msg.height),
            "frame_id": str(msg.header.frame_id),
            "k": [float(v) for v in msg.k],
            "d": [float(v) for v in msg.d],
        }
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        self.get_logger().info(f"Saved camera_info json: {out_path}")


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagStaticCalibNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
