import math
import threading
from typing import Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from tf2_ros import Buffer, TransformException, TransformListener


_PC2_DATATYPE_TO_NP = {
    1: np.int8,    # INT8
    2: np.uint8,   # UINT8
    3: np.int16,   # INT16
    4: np.uint16,  # UINT16
    5: np.int32,   # INT32
    6: np.uint32,  # UINT32
    7: np.float32, # FLOAT32
    8: np.float64, # FLOAT64
}


def _extract_xyz_array(msg: PointCloud2) -> Optional[np.ndarray]:
    """Return Nx3 float64 xyz array in the cloud's frame.

    Works on ROS2 Foxy without sensor_msgs_py.
    """

    if msg.point_step <= 0 or msg.width * msg.height <= 0:
        return None

    offsets = {}
    dtypes = {}
    for f in msg.fields:
        if f.name in ('x', 'y', 'z'):
            offsets[f.name] = int(f.offset)
            dtypes[f.name] = _PC2_DATATYPE_TO_NP.get(int(f.datatype))

    if any(k not in offsets for k in ('x', 'y', 'z')):
        return None
    if any(dtypes[k] is None for k in ('x', 'y', 'z')):
        return None

    itemsize = int(msg.point_step)
    dtype = np.dtype(
        {
            'names': ['x', 'y', 'z'],
            'formats': [dtypes['x'], dtypes['y'], dtypes['z']],
            'offsets': [offsets['x'], offsets['y'], offsets['z']],
            'itemsize': itemsize,
        }
    )

    bo = '>' if bool(msg.is_bigendian) else '<'
    dtype = dtype.newbyteorder(bo)

    n = int(msg.width * msg.height)
    try:
        arr = np.frombuffer(msg.data, dtype=dtype, count=n)
    except Exception:
        return None

    xyz = np.empty((n, 3), dtype=np.float64)
    xyz[:, 0] = arr['x'].astype(np.float64, copy=False)
    xyz[:, 1] = arr['y'].astype(np.float64, copy=False)
    xyz[:, 2] = arr['z'].astype(np.float64, copy=False)
    return xyz


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    x2 = qx + qx
    y2 = qy + qy
    z2 = qz + qz

    xx = qx * x2
    yy = qy * y2
    zz = qz * z2
    xy = qx * y2
    xz = qx * z2
    yz = qy * z2
    wx = qw * x2
    wy = qw * y2
    wz = qw * z2

    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


class LidarOverlayNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_overlay')

        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('color_info_topic', '/camera/color/camera_info')
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('overlay_topic', '/lidar/overlay/image')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('lidar_frame', 'livox_frame')
        self.declare_parameter('max_points', 8000)
        self.declare_parameter('z_min_m', 0.1)
        self.declare_parameter('z_max_m', 20.0)
        self.declare_parameter('time_slop_sec', 0.12)

        self._color_topic = str(self.get_parameter('color_topic').value)
        self._color_info_topic = str(self.get_parameter('color_info_topic').value)
        self._lidar_topic = str(self.get_parameter('lidar_topic').value)
        self._overlay_topic = str(self.get_parameter('overlay_topic').value)
        self._camera_frame = str(self.get_parameter('camera_frame').value)
        self._lidar_frame = str(self.get_parameter('lidar_frame').value)
        self._max_points = int(self.get_parameter('max_points').value)
        self._z_min_m = float(self.get_parameter('z_min_m').value)
        self._z_max_m = float(self.get_parameter('z_max_m').value)
        self._time_slop_sec = float(self.get_parameter('time_slop_sec').value)

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        self._last_img_msg: Optional[Image] = None
        self._last_img_cv: Optional[np.ndarray] = None
        self._last_info: Optional[CameraInfo] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(Image, self._overlay_topic, 10)

        self.create_subscription(Image, self._color_topic, self._on_image, 10)
        self.create_subscription(CameraInfo, self._color_info_topic, self._on_info, 10)
        self.create_subscription(PointCloud2, self._lidar_topic, self._on_lidar, 10)

        self.get_logger().info(
            f"Lidar overlay ready. Subscribing color={self._color_topic}, info={self._color_info_topic}, lidar={self._lidar_topic}. "
            f"Publishing overlay={self._overlay_topic}. TF uses {self._camera_frame} <- {self._lidar_frame}"
        )

    def _on_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._last_info = msg

    def _on_image(self, msg: Image) -> None:
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'imgmsg_to_cv2 failed: {e}')
            return

        with self._lock:
            self._last_img_msg = msg
            self._last_img_cv = cv_img

    def _try_get_image(self, stamp_sec: float) -> Tuple[Optional[Image], Optional[np.ndarray]]:
        with self._lock:
            if self._last_img_msg is None or self._last_img_cv is None:
                return (None, None)
            img_stamp = float(self._last_img_msg.header.stamp.sec) + float(self._last_img_msg.header.stamp.nanosec) * 1e-9
            if abs(img_stamp - stamp_sec) > self._time_slop_sec:
                return (None, None)
            return (self._last_img_msg, self._last_img_cv.copy())

    def _get_intrinsics(self) -> Optional[Tuple[float, float, float, float]]:
        with self._lock:
            info = self._last_info
        if info is None or len(info.k) != 9:
            return None
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        return (fx, fy, cx, cy)

    def _on_lidar(self, msg: PointCloud2) -> None:
        intr = self._get_intrinsics()
        if intr is None:
            return
        fx, fy, cx, cy = intr

        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        img_msg, img = self._try_get_image(stamp_sec)
        if img_msg is None or img is None:
            return

        try:
            tf = self._tf_buffer.lookup_transform(
                self._camera_frame,
                self._lidar_frame,
                msg.header.stamp,
            )
        except TransformException:
            return

        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        tz = float(tf.transform.translation.z)
        qx = float(tf.transform.rotation.x)
        qy = float(tf.transform.rotation.y)
        qz = float(tf.transform.rotation.z)
        qw = float(tf.transform.rotation.w)

        rot = _quat_to_rot(qx, qy, qz, qw)
        t = np.array([tx, ty, tz], dtype=np.float64)

        h, w = img.shape[:2]

        xyz_l = _extract_xyz_array(msg)
        if xyz_l is None:
            return

        n = int(xyz_l.shape[0])
        if n == 0:
            return

        step = 1
        if self._max_points > 0 and n > self._max_points:
            step = max(1, n // self._max_points)

        xyz_l = xyz_l[::step, :]
        if xyz_l.size == 0:
            return

        # Transform lidar points into camera frame
        xyz_c = (rot @ xyz_l.T).T + t[None, :]
        X = xyz_c[:, 0]
        Y = xyz_c[:, 1]
        Z = xyz_c[:, 2]

        valid = np.isfinite(Z) & (Z > 0.0) & (Z >= self._z_min_m) & (Z <= self._z_max_m)
        if not np.any(valid):
            return
        X = X[valid]
        Y = Y[valid]
        Z = Z[valid]

        u = fx * (X / Z) + cx
        v = fy * (Y / Z) + cy
        ui = np.rint(u).astype(np.int32)
        vi = np.rint(v).astype(np.int32)

        in_img = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
        if not np.any(in_img):
            return
        ui = ui[in_img]
        vi = vi[in_img]
        Z = Z[in_img]

        # Draw points (loop over pixels; still fast enough for <= max_points)
        denom = max(1e-6, (self._z_max_m - self._z_min_m))
        for px, py, zz in zip(ui.tolist(), vi.tolist(), Z.tolist()):
            d_norm = float((zz - self._z_min_m) / denom)
            d_norm = float(min(1.0, max(0.0, d_norm)))
            color = (int(255 * (1.0 - d_norm)), int(255 * d_norm), 0)
            cv2.circle(img, (int(px), int(py)), 2, color, -1)

        out = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out.header = img_msg.header
        self._pub.publish(out)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = LidarOverlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

