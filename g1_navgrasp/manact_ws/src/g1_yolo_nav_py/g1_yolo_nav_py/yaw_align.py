"""
偏航对齐节点 — 步进式旋转对齐（适配慢速相机更新）。

运动控制通过 SportClient 统一封装（Loco API 方式）。
不自动执行 FSM 初始化，需手动进入走跑模式。

对齐逻辑封装在 StepAligner 中（_step_aligner.py），
grasp_task 也使用同一个 StepAligner，保证行为一致。

控制逻辑（步进式）：
    1. 检测目标位置 u（归一化 0~1，0.5 = 画面中央）
    2. 若目标偏离中心，发送一次短时间小幅度旋转（SET_VELOCITY, duration=step_duration）
    3. 等待 camera_settle_time（默认 2 秒）让相机更新
    4. 重新检测目标位置，重复步骤 2~3
    5. 目标居中后停止

运行：
    ros2 run g1_yolo_nav_py yaw_align
"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from g1_yolo_nav_py._detection_utils import find_best_detection

from g1_yolo_nav_py.sport_client import SportClient
from g1_yolo_nav_py._step_aligner import StepAligner, AlignAction

class YawAlignNode(Node):
    """偏航对齐节点 — 使用 StepAligner 步进式旋转。

    每次只移动一小步，等相机更新后再决定下一步，
    避免连续旋转时相机延迟导致过冲。
    """

    def __init__(self) -> None:
        super().__init__("g1_yaw_align_node")

        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("target_class_id", "chair")
        self.declare_parameter("center_tolerance", 0.08)
        self.declare_parameter("step_yaw_speed", 0.3)
        self.declare_parameter("step_duration", 0.8)
        self.declare_parameter("camera_settle_time", 2.0)
        self.declare_parameter("max_consecutive_steps", 10)
        self.declare_parameter("lost_timeout", 10.0)
        self.declare_parameter("check_rate", 2.0)

        p = lambda n: self.get_parameter(n).value
        self._det_topic = p("detection_topic")
        self._target_class = p("target_class_id")
        self._lost_timeout = float(p("lost_timeout"))
        self._check_rate = float(p("check_rate"))

        self._target_u: Optional[float] = None
        self._last_detect_time: float = 0.0
        self._aligned_logged: bool = False

        self.create_subscription(Detection2DArray, self._det_topic,
                                 self._on_detection, 10)

        self._sport = SportClient(self)

        self._sport.skip_init()

        self._aligner = StepAligner(
            move_fn=self._sport.move,
            logger=self.get_logger(),
            center_tolerance=float(p("center_tolerance")),
            step_yaw_speed=float(p("step_yaw_speed")),
            step_duration=float(p("step_duration")),
            camera_settle_time=float(p("camera_settle_time")),
            max_consecutive_steps=int(p("max_consecutive_steps")),
        )

        self._timer = self.create_timer(1.0 / self._check_rate, self._tick)

        self._diag_done = False
        self._diag_timer = self.create_timer(3.0, self._diag_check)

        self.get_logger().info(
            f"偏航对齐节点就绪（步进模式）: 目标={self._target_class}, "
            f"步速={p('step_yaw_speed')}rad/s, 步时={p('step_duration')}s, "
            f"等待={p('camera_settle_time')}s, 容差={p('center_tolerance')}"
        )

    def _on_detection(self, msg: Detection2DArray) -> None:
        """从检测结果中提取最佳目标的 u 坐标。"""
        best_det, best_score = find_best_detection(msg.detections, self._target_class)
        if best_det is not None:
            prev_u = self._target_u
            self._target_u = best_det.bbox.center.x
            self._last_detect_time = time.time()
            if prev_u is None:
                self.get_logger().info(
                    f"[对齐] 检测到目标: u={self._target_u:.3f}"
                )
        else:
            self._target_u = None

    def _diag_check(self):
        """启动后延迟检查关键话题状态。"""
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

    def _tick(self) -> None:
        """步进式对齐：委托给 StepAligner。"""
        if not self._sport.ready:
            return

        now = time.time()

        if self._target_u is None or (now - self._last_detect_time > self._lost_timeout):
            self._aligned_logged = False

        action, extra = self._aligner.tick(self._target_u)

        if action == AlignAction.ALIGNED and not self._aligned_logged:
            self._aligned_logged = True
            self.get_logger().info(f"[对齐] {extra}")
        elif action == AlignAction.ROTATING and extra:
            self.get_logger().info(f"[对齐] {extra}")
        elif action == AlignAction.LOST and extra:
            self.get_logger().info(f"[对齐] {extra}")

    def destroy_node(self) -> None:
        self._sport.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YawAlignNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._sport.stop()
    finally:
        node._sport.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
