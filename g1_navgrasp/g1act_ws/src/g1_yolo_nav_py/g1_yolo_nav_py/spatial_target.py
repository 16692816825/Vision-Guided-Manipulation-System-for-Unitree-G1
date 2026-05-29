"""空间投影节点 — 将 2D 检测结果投影到 3D 空间，发布目标位姿。"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf2_ros
from cv_bridge import CvBridge
import cv2


class SpatialTargetNode(Node):
    """将 2D 检测框投影到 odom 坐标系下的 3D 坐标。"""

    def __init__(self) -> None:
        super().__init__("g1_spatial_target_node")

        # ---- 参数 ----
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("target_class_id", "0")  # person
        self.declare_parameter("default_depth", 2.0)  # 无深度传感器时的默认距离 (m)
        self.declare_parameter("use_depth_sensor", False)
        self.declare_parameter("nav_target_topic", "/g1/nav/target_pose")
        self.declare_parameter("approach_distance", 0.8)  # 接近到距目标多远停下 (m)
        self.declare_parameter("publish_rate", 5.0)

        self._target_class = self.get_parameter("target_class_id").value
        self._default_depth = float(self.get_parameter("default_depth").value)
        self._approach_dist = float(self.get_parameter("approach_distance").value)
        self._use_depth = bool(self.get_parameter("use_depth_sensor").value)
        self._target_frame = self.get_parameter("target_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._publish_rate = float(self.get_parameter("publish_rate").value)

        # ---- CV Bridge ----
        self._bridge = CvBridge()

        # ---- 相机内参 ----
        self._camera_info: CameraInfo | None = None
        self._depth_image: np.ndarray | None = None
        self._fx = 0.0
        self._fy = 0.0
        self._cx = 0.0
        self._cy = 0.0

        # ---- 最新检测结果缓存 ----
        self._latest_detection: Detection2DArray | None = None

        # ---- TF ----
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- QoS ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ---- 订阅 ----
        det_topic = self.get_parameter("detection_topic").value
        self._det_sub = self.create_subscription(
            Detection2DArray, det_topic, self._detection_callback, 10
        )
        self._info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_callback,
            sensor_qos,
        )

        if self._use_depth:
            self._depth_sub = self.create_subscription(
                Image,
                self.get_parameter("depth_topic").value,
                self._depth_callback,
                sensor_qos,
            )

        # ---- 发布 ----
        nav_topic = self.get_parameter("nav_target_topic").value
        self._target_pub = self.create_publisher(PoseStamped, nav_topic, 10)

        # ---- 定时发布 ----
        self._timer = self.create_timer(1.0 / self._publish_rate, self._publish_target)

        self.get_logger().info(
            f"空间投影节点启动: 目标帧={self._target_frame}, "
            f"使用深度={self._use_depth}, 默认深度={self._default_depth}m"
        )

    # ------------------------------------------------------------------
    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """缓存相机内参。"""
        self._camera_info = msg
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]

    # ------------------------------------------------------------------
    def _depth_callback(self, msg: Image) -> None:
        """缓存深度图像。"""
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"深度图像转换失败: {e}")

    # ------------------------------------------------------------------
    def _detection_callback(self, msg: Detection2DArray) -> None:
        """缓存最新检测结果。"""
        self._latest_detection = msg

    # ------------------------------------------------------------------
    def _get_depth_at_pixel(self, u: float, v: float, width: float, height: float) -> float:
        """获取像素 (u,v) 处的深度值。"""
        if self._depth_image is None:
            return self._default_depth

        # 采样中心区域
        px = int(u * width)
        py = int(v * height)
        h, w = self._depth_image.shape[:2]
        half = 5
        x0 = max(0, px - half)
        x1 = min(w, px + half)
        y0 = max(0, py - half)
        y1 = min(h, py + half)
        region = self._depth_image[y0:y1, x0:x1]
        valid = region[region > 0]
        if len(valid) > 0:
            return float(np.median(valid)) / 1000.0  # mm → m
        return self._default_depth

    # ------------------------------------------------------------------
    def _pixel_to_camera_point(self, u: float, v: float, depth: float) -> np.ndarray:
        """将归一化像素坐标 + 深度 → 相机坐标系下的 3D 点。"""
        if self._fx == 0.0:
            self.get_logger().warn("相机内参尚未收到，无法投影")
            return np.array([0.0, 0.0, self._default_depth])

        px = u * self._camera_info.width
        py = v * self._camera_info.height
        z = depth
        x = (px - self._cx) * z / self._fx
        y = (py - self._cy) * z / self._fy
        return np.array([x, y, z])

    # ------------------------------------------------------------------
    def _publish_target(self) -> None:
        """定时发布目标位姿到 odom 坐标系。"""
        if self._latest_detection is None or len(self._latest_detection.detections) == 0:
            return

        if self._fx == 0.0:
            return

        # 过滤目标类别
        best_det = None
        best_score = 0.0
        for det in self._latest_detection.detections:
            if det.results and det.results[0].id == self._target_class:
                if det.results[0].score > best_score:
                    best_score = det.results[0].score
                    best_det = det

        if best_det is None:
            return

        # 取框底部中心（更贴近地面）
        u = best_det.bbox.center.position.x
        v = best_det.bbox.center.position.y + best_det.bbox.size_y / 2.0

        # 获取深度
        depth = self._get_depth_at_pixel(
            u, v,
            self._camera_info.width,
            self._camera_info.height,
        )

        # 像素 → 相机坐标系
        camera_point = self._pixel_to_camera_point(u, v, depth)

        # 相机坐标系 → odom 坐标系
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                self._camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().debug(f"TF 查询失败: {e}")
            return

        # 平移变换
        q = transform.transform.rotation
        rot = _quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        odom_point = rot @ camera_point + np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ])

        # 获取机器人当前朝向
        try:
            base_tf = self._tf_buffer.lookup_transform(
                self._target_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            yaw = _quaternion_to_yaw(
                base_tf.transform.rotation.x,
                base_tf.transform.rotation.y,
                base_tf.transform.rotation.z,
                base_tf.transform.rotation.w,
            )
        except Exception:
            yaw = 0.0

        # 目标点 = 检测到的 3D 位置 - approach_distance * 朝向方向
        target_x = odom_point[0] - self._approach_dist * math.cos(yaw)
        target_y = odom_point[1] - self._approach_dist * math.sin(yaw)

        # 发布目标位姿
        pose_msg = PoseStamped()
        pose_msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self._target_frame,
        )
        pose_msg.pose.position.x = float(target_x)
        pose_msg.pose.position.y = float(target_y)
        pose_msg.pose.position.z = 0.0
        # 朝向目标
        angle_to_target = math.atan2(
            odom_point[1] - base_tf.transform.translation.y,
            odom_point[0] - base_tf.transform.translation.x,
        )
        q_target = _yaw_to_quaternion(angle_to_target)
        pose_msg.pose.orientation.x = q_target[0]
        pose_msg.pose.orientation.y = q_target[1]
        pose_msg.pose.orientation.z = q_target[2]
        pose_msg.pose.orientation.w = q_target[3]

        self._target_pub.publish(pose_msg)
        self.get_logger().debug(
            f"目标发布: ({target_x:.2f}, {target_y:.2f}), "
            f"置信度: {best_score:.2f}, 深度: {depth:.2f}m"
        )


# ======================================================================
# 辅助函数
# ======================================================================

def _quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """四元数 → 3x3 旋转矩阵。"""
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])
    return R


def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """四元数 → yaw 角 (弧度)。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(yaw: float) -> tuple:
    """yaw 角 (弧度) → 四元数 (x, y, z, w)。"""
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpatialTargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
