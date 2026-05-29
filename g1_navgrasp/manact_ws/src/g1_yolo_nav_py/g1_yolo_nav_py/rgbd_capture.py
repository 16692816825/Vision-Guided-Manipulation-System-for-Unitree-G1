"""
RGBD 数据采集节点 — 定时采集彩色图像（含检测框标注）+ 深度图，保存到本地目录。

用法：
    ros2 run g1_yolo_nav_py rgbd_capture
    ros2 run g1_yolo_nav_py rgbd_capture --ros-args \
        -p interval_sec:=5.0 -p duration_sec:=60.0 -p output_dir:=/tmp/rgbd_data

输出目录结构：
    output_dir/
    ├── img/        # 彩色图像（带检测框标注），.jpg 格式
    │   ├── 0000.jpg
    │   ├── 0005.jpg
    │   └── ...
    └── d/          # 深度图，.png 格式（16位，单位 mm）
        ├── 0000.png
        ├── 0005.png
        └── ...
    文件名 = 采集序号，img/ 和 d/ 一一对应。
"""

import os
import sys
import time
from pathlib import Path

# ROS2 colcon 隔离 PYTHONPATH，必须在所有 import 之前追加路径
for _p in [
    "/usr/lib/python3/dist-packages",
    os.path.expanduser("~/.local/lib/python3.8/site-packages"),
    "/usr/local/lib/python3.8/dist-packages",
]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

from g1_yolo_nav_py._vis_utils import get_color as _get_color

class RGBDCaptureNode(Node):
    """RGBD 数据采集节点 — 定时保存彩色图像 + 深度图。"""

    def __init__(self) -> None:
        super().__init__("g1_rgbd_capture_node")

        self.declare_parameter("color_topic", "/D455_1/color/image_raw")
        self.declare_parameter("depth_topic", "/D455_1/aligned_depth_to_color/image_raw")
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("interval_sec", 5.0)
        self.declare_parameter("duration_sec", 60.0)
        self.declare_parameter("output_dir", "")

        p = lambda n: self.get_parameter(n).value
        self._color_topic = p("color_topic")
        self._depth_topic = p("depth_topic")
        self._det_topic = p("detection_topic")
        self._interval = float(p("interval_sec"))
        self._duration = float(p("duration_sec"))

        output_dir = p("output_dir")
        if not output_dir:
            output_dir = str(Path(__file__).resolve().parent.parent / "rgbd_data")
        self._output_dir = Path(output_dir)
        self._img_dir = self._output_dir / "img"
        self._depth_dir = self._output_dir / "d"

        self._bridge = CvBridge()

        self._color_image = None
        self._depth_image = None
        self._detections = None

        self._capture_index = 0
        self._start_time = None
        self._last_capture_time = 0.0
        self._done = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Image, self._color_topic, self._color_cb, sensor_qos)
        self.create_subscription(Image, self._depth_topic, self._depth_cb, sensor_qos)
        self.create_subscription(Detection2DArray, self._det_topic, self._det_cb, 10)

        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"RGBD 采集节点就绪: 间隔={self._interval}s, "
            f"时长={self._duration}s, 输出={self._output_dir}"
        )

    def _color_cb(self, msg: Image) -> None:
        try:
            self._color_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def _depth_cb(self, msg: Image) -> None:
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        except Exception:
            try:
                # 某些驱动用 passthrough 编码
                self._depth_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            except Exception:
                pass

    def _det_cb(self, msg: Detection2DArray) -> None:
        self._detections = msg

    def _draw_detections(self, frame: np.ndarray) -> np.ndarray:
        """在图像上绘制检测框。"""
        if self._detections is None:
            return frame
        h, w = frame.shape[:2]
        for det in self._detections.detections:
            if not det.results:
                continue
            class_id = det.results[0].id
            score = det.results[0].score
            color = _get_color(class_id)

            cx = det.bbox.center.x * w
            cy = det.bbox.center.y * h
            bw = det.bbox.size_x * w
            bh = det.bbox.size_y * h
            x1 = int(cx - bw / 2)
            y1 = int(cy - bh / 2)
            x2 = int(cx + bw / 2)
            y2 = int(cy + bh / 2)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"{class_id} {score:.0%}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            (tw, th), _ = cv2.getTextSize(label, font, 0.6, 1)
            cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
            cv2.putText(frame, label, (x1, y1 - 4), font, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        return frame

    def _tick(self) -> None:
        """定时检查是否需要采集。"""
        if self._done:
            return

        now = time.time()
        if self._start_time is None:
            self._start_time = now
            self._last_capture_time = now

        elapsed = now - self._start_time
        if elapsed >= self._duration:
            self._done = True
            total = self._capture_index
            self.get_logger().info(
                f"采集完成! 共 {total} 帧, 保存至 {self._output_dir}"
            )
            rclpy.shutdown()
            return

        if (now - self._last_capture_time) >= self._interval:
            self._capture()

    def _capture(self) -> None:
        """执行一次采集。"""
        if self._color_image is None:
            self.get_logger().warn("等待彩色图像...")
            return

        idx = self._capture_index
        name = f"{idx:04d}"

        self._img_dir.mkdir(parents=True, exist_ok=True)
        self._depth_dir.mkdir(parents=True, exist_ok=True)

        annotated = self._draw_detections(self._color_image.copy())
        img_path = self._img_dir / f"{name}.jpg"
        cv2.imwrite(str(img_path), annotated)

        # 保存深度图（16位 PNG，单位 mm）
        depth_saved = False
        if self._depth_image is not None:
            depth_path = self._depth_dir / f"{name}.png"
            cv2.imwrite(str(depth_path), self._depth_image)
            depth_saved = True

        self._capture_index += 1
        self._last_capture_time = time.time()

        depth_hint = " + 深度" if depth_saved else " (无深度)"
        remaining = self._duration - (time.time() - self._start_time)
        self.get_logger().info(
            f"采集 #{idx}: {name}.jpg{depth_hint}, "
            f"剩余 {remaining:.0f}s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = RGBDCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"中断采集, 已保存 {node._capture_index} 帧")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
