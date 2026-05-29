"""导航规划节点 — 接收目标位姿，通过 Nav2 进行路径规划，控制机器人移动。"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class NavPlannerNode(Node):
    """导航规划节点：接收目标位姿，调用 Nav2 导航到目标附近。"""

    def __init__(self) -> None:
        super().__init__("g1_nav_planner_node")

        # ---- 参数 ----
        self.declare_parameter("target_topic", "/g1/nav/target_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("max_linear_speed", 0.3)       # 最大线速度 (m/s)
        self.declare_parameter("max_angular_speed", 0.5)      # 最大角速度 (rad/s)
        self.declare_parameter("goal_tolerance", 0.3)         # 到达容差 (m)
        self.declare_parameter("navigation_server", "navigate_to_pose")
        self.declare_parameter("use_nav2", True)              # 是否使用 Nav2（False 则用简单趋近）

        self._max_lin = float(self.get_parameter("max_linear_speed").value)
        self._max_ang = float(self.get_parameter("max_angular_speed").value)
        self._tolerance = float(self.get_parameter("goal_tolerance").value)
        self._use_nav2 = bool(self.get_parameter("use_nav2").value)

        self._clamp_speed(self._max_lin, "max_linear_speed", 0.0, 1.0)
        self._clamp_speed(self._max_ang, "max_angular_speed", 0.0, 2.0)

        self.get_logger().info(
            f"导航节点启动: use_nav2={self._use_nav2}, "
            f"max_lin={self._max_lin}m/s, max_ang={self._max_ang}rad/s"
        )

        # ---- 状态 ----
        self._current_goal: PoseStamped | None = None
        self._nav2_active = False
        self._emergency_stop = False

        # ---- Nav2 Action Client ----
        if self._use_nav2:
            self._nav_client = ActionClient(
                self, NavigateToPose, self.get_parameter("navigation_server").value
            )

        # ---- 话题 ----
        self._goal_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("target_topic").value,
            self._goal_callback,
            10,
        )
        self._cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )

        # ---- 控制定时器 ----
        self._control_timer = self.create_timer(0.1, self._control_loop)

        # ---- 紧急停止服务（简易版：订阅话题触发） ----
        self._estop_sub = self.create_subscription(
            Twist, "/g1/nav/emergency_stop", self._emergency_stop_callback, 10
        )

    # ------------------------------------------------------------------
    def _clamp_speed(self, value: float, name: str, lo: float, hi: float) -> float:
        """限幅检查。"""
        clamped = max(lo, min(hi, value))
        if clamped != value:
            self.get_logger().warn(
                f"参数 {name}={value} 超出范围 [{lo}, {hi}]，已限制为 {clamped}"
            )
        return clamped

    # ------------------------------------------------------------------
    def _emergency_stop_callback(self, msg: Twist) -> None:
        """紧急停止回调。"""
        self._emergency_stop = True
        self.get_logger().warn("收到紧急停止指令！")
        self._publish_stop()

    # ------------------------------------------------------------------
    def _publish_stop(self) -> None:
        """发送零速度指令。"""
        twist = Twist()
        self._cmd_pub.publish(twist)

    # ------------------------------------------------------------------
    def _goal_callback(self, msg: PoseStamped) -> None:
        """接收到新目标位姿。"""
        self._current_goal = msg
        self._emergency_stop = False
        self.get_logger().info(
            f"收到导航目标: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )

        if self._use_nav2 and self._nav_client.wait_for_server(timeout_sec=2.0):
            self._send_nav2_goal(msg)
        else:
            if self._use_nav2:
                self.get_logger().warn("Nav2 服务不可用，回退到简单趋近模式")

    # ------------------------------------------------------------------
    def _send_nav2_goal(self, goal_pose: PoseStamped) -> None:
        """向 Nav2 发送导航目标。"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._nav2_goal_response_callback)
        self._nav2_active = True

    # ------------------------------------------------------------------
    def _nav2_goal_response_callback(self, future) -> None:
        """Nav2 目标发送结果回调。"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 拒绝了导航目标")
            self._nav2_active = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_callback)

    # ------------------------------------------------------------------
    def _nav2_result_callback(self, future) -> None:
        """Nav2 导航结果回调。"""
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("导航成功到达目标！")
        else:
            self.get_logger().warn(f"导航结束，状态码: {result.status}")
        self._nav2_active = False

    # ------------------------------------------------------------------
    def _control_loop(self) -> None:
        """控制循环 — Nav2 不可用时使用简单趋近策略。"""
        if self._emergency_stop:
            self._publish_stop()
            return

        # Nav2 活跃时不干预
        if self._use_nav2 and self._nav2_active:
            return

        if self._current_goal is None:
            return

        # 简单趋近模式：发布目标位姿作为 Nav2 goal 或直接用 Twist
        if not self._use_nav2:
            self._simple_approach()

    # ------------------------------------------------------------------
    def _simple_approach(self) -> None:
        """简单趋近控制（无 Nav2 时的回退方案）。

        通过发布 Twist 指令让机器人朝目标点移动。
        实际使用中建议配合里程计 TF 使用。
        """
        if self._current_goal is None:
            return

        # 这里仅发布提示日志，实际趋近需要里程计反馈
        # 完整实现请配合 TF 获取机器人当前位置进行 PD 控制
        self.get_logger().debug(
            "简单趋近模式：需要里程计反馈闭环，建议启用 Nav2"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
