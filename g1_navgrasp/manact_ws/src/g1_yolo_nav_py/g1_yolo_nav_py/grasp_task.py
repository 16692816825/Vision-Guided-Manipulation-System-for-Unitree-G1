#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 抓取任务主控程序
==================
一键执行：YOLO 检测 → 搜索 → 对齐 → 接近 → 抓取 → 交互菜单

状态机：
    WORKING     → 搜索 + StepAligner 对齐 + ForwardApproach 接近（连续行为，无状态切换）
    GRABBING    → 执行 armup.py 抓取
    MENU        → 交互菜单

对齐逻辑直接使用 StepAligner（_step_aligner.py），与 yaw_align.py 共用同一份代码。
接近逻辑直接使用 ForwardApproach（_forward_approach.py），与 loco_forward.py 共用同一份代码。

运行：
    ros2 run g1_yolo_nav_py grasp_task
    python3 -m g1_yolo_nav_py.grasp_task
"""

import rclpy
from rclpy.node import Node

from g1_yolo_nav_py._grasp_state import GraspStateMachineMixin, GraspState

class GraspTaskNode(Node, GraspStateMachineMixin):
    """G1 抓取任务主控节点 — 通过 SportClient (Loco API) 统一控制运动。"""

    def __init__(self) -> None:
        Node.__init__(self, "g1_grasp_task_node")

        self._init_grasp_state(
            self,
            start_state=GraspState.WORKING,
        )

        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info("=" * 50)
        self.get_logger().info("G1 抓取任务启动（Loco API 模式）")
        self.get_logger().info(f"目标类别: {self._gs_target_class}")
        self.get_logger().info(f"armup: {self._gs_armup_script}")
        self.get_logger().info(f"armdown: {self._gs_armdown_script}")
        self.get_logger().info("=" * 50)

    def _log_info(self, msg: str) -> None:
        self.get_logger().info(msg)

    def _log_error(self, msg: str) -> None:
        self.get_logger().error(msg)

    def _on_state_changed(self, old_state: GraspState, new_state: GraspState) -> None:
        pass  # 终端模式无需额外 UI 更新

    def _tick(self) -> None:
        self._gs_tick()

    def destroy_node(self) -> None:
        self._gs_destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GraspTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
