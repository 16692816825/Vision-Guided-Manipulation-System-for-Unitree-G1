#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
前进控制节点 — 检测到目标对齐后，通过 Loco API SET_VELOCITY 控制机器人前进。

运动控制通过 SportClient 统一封装（Loco API 方式）。
不自动执行 FSM 初始化，需手动进入走跑模式。

前进逻辑封装在 ForwardApproach 中（_forward_approach.py），
grasp_task 也使用同一个 ForwardApproach，保证行为一致。

运行：
    ros2 run g1_yolo_nav_py loco_forward
    ros2 run g1_yolo_nav_py loco_forward --ros-args -p forward_speed:=0.15
"""

import math
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from g1_yolo_nav_py._detection_utils import find_best_detection, sample_depth_at_pixel, depth_to_meters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

from g1_yolo_nav_py.sport_client import SportClient
from g1_yolo_nav_py._forward_approach import ForwardApproach, ApproachAction

class LocoForwardNode(Node):
    """前进节点 — 使用 ForwardApproach 前进到目标。"""

    def __init__(self) -> None:
        super().__init__("g1_loco_forward_node")

        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("depth_topic", "/D455_1/depth/image_rect_raw")
        self.declare_parameter("target_class_id", "chair")
        self.declare_parameter("use_depth_distance", True)
        self.declare_parameter("stop_distance", 0.5)
        self.declare_parameter("depth_sample_radius", 5)
        self.declare_parameter("forward_speed", 0.3)
        self.declare_parameter("center_tolerance", 0.08)
        self.declare_parameter("align_stable_time", 0.8)
        self.declare_parameter("arrive_bbox_ratio", 0.45)
        self.declare_parameter("lost_timeout", 1.0)
        self.declare_parameter("check_rate", 10.0)
        self.declare_parameter("sit_on_exit", True)

        p = lambda n: self.get_parameter(n).value
        self._det_topic = p("detection_topic")
        self._depth_topic = p("depth_topic")
        self._target_class = p("target_class_id")
        self._use_depth = bool(p("use_depth_distance"))
        self._depth_radius = max(1, int(p("depth_sample_radius")))
        self._lost_timeout = float(p("lost_timeout"))
        self._rate = float(p("check_rate"))
        self._sit_on_exit = bool(p("sit_on_exit"))

        self._target_u: Optional[float] = None
        self._target_v: Optional[float] = None
        self._target_distance: Optional[float] = None
        self._depth_image: Optional[np.ndarray] = None
        self._depth_encoding: str = ""
        self._bbox_size_x: float = 0.0
        self._bbox_size_y: float = 0.0
        self._last_detect_time: float = 0.0

        self._bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        self.create_subscription(Detection2DArray, self._det_topic,
                                 self._on_detection, 10)
        if self._use_depth:
            self.create_subscription(
                Image, self._depth_topic, self._on_depth, sensor_qos
            )

        self._sport = SportClient(self)

        self._sport.skip_init()

        self._approach = ForwardApproach(
            move_fn=self._sport.move,
            stop_fn=self._sport.stop,
            logger=self.get_logger(),
            forward_speed=float(p("forward_speed")),
            center_tolerance=float(p("center_tolerance")),
            align_stable_time=float(p("align_stable_time")),
            use_depth=self._use_depth,
            stop_distance=float(p("stop_distance")),
            arrive_bbox_ratio=float(p("arrive_bbox_ratio")),
        )

        self._timer = self.create_timer(1.0 / self._rate, self._tick)

        self._diag_done = False
        self._diag_timer = self.create_timer(3.0, self._diag_check)

        self.get_logger().info(
            f"前进节点就绪: 速度={p('forward_speed')}m/s, "
            f"深度={self._use_depth}(停止≤{p('stop_distance')}m), "
            f"bbox到达={p('arrive_bbox_ratio')}"
        )

    def _diag_check(self):
        if self._diag_done:
            return
        self._diag_done = True
        if hasattr(self, '_diag_timer') and self._diag_timer is not None:
            self._diag_timer.cancel()

        det_pub_count = self.count_publishers(self._det_topic)
        self.get_logger().info(
            f"[诊断] 检测话题 '{self._det_topic}': 发布者数={det_pub_count}"
        )
        if det_pub_count == 0:
            self.get_logger().warn(
                f"[诊断] 检测话题无发布者! "
                f"请先启动 yolo_detector: "
                f"ros2 run g1_yolo_nav_py yolo_detector"
            )

        sport_sub_count = self.count_subscribers('/api/sport/request')
        self.get_logger().info(
            f"[诊断] 运动话题 '/api/sport/request': 订阅者数={sport_sub_count}"
        )
        if sport_sub_count == 0:
            self.get_logger().error(
                "[诊断] 运动话题无订阅者! "
                "请确认 unitree SDK bridge 已启动。"
            )

    def _on_detection(self, msg: Detection2DArray) -> None:
        best_det, best_score = find_best_detection(msg.detections, self._target_class)
        if best_det is not None:
            bbox = best_det.bbox
            self._target_u = bbox.center.x
            self._target_v = bbox.center.y
            self._bbox_size_x = bbox.size_x
            self._bbox_size_y = bbox.size_y
            self._last_detect_time = time.time()
            self._update_target_distance()
        else:
            self._target_u = None
            self._target_v = None
            self._target_distance = None

    def _on_depth(self, msg: Image) -> None:
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            self._depth_encoding = msg.encoding
        except Exception as e:
            self.get_logger().warn(f"深度图转换失败: {e}")

    def _update_target_distance(self) -> None:
        self._target_distance = None
        if not self._use_depth or self._depth_image is None:
            return
        if self._target_u is None or self._target_v is None:
            return

        raw = sample_depth_at_pixel(self._depth_image, self._target_u, self._target_v, self._depth_radius)
        if raw is None:
            return
        distance = depth_to_meters(raw, self._depth_encoding)
        if math.isfinite(distance) and distance > 0.0:
            self._target_distance = distance

    #  tick（委托给 ForwardApproach）
    def _tick(self) -> None:
        if not self._sport.ready:
            return

        now = time.time()

        if self._target_u is None or (now - self._last_detect_time > self._lost_timeout):
            self._approach.stop()
            return

        action, msg = self._approach.tick(
            target_u=self._target_u,
            target_distance=self._target_distance,
            bbox_size_x=self._bbox_size_x,
            bbox_size_y=self._bbox_size_y,
        )

        if action == ApproachAction.ARRIVED and msg:
            self.get_logger().info(f"[前进] {msg}")

    def destroy_node(self) -> None:
        self._sport.stop()
        if self._sit_on_exit:
            self.get_logger().info("退出: 自动坐下 (sit_on_exit=true)")
            time.sleep(0.2)
            self._sport.sit()
        else:
            self.get_logger().info("退出: 仅停止运动 (sit_on_exit=false)")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LocoForwardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
