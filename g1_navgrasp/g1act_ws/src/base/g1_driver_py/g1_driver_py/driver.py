"""G1 驱动节点 — 订阅机器人状态，发布里程计、TF 变换和关节状态。"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState, SportModeState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class Driver(Node):
    """宇树 G1 驱动节点。"""

    def __init__(self) -> None:
        super().__init__("g1_driver_node")

        # ---- 参数 ----
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.publish_odom_tf = self.get_parameter("publish_odom_tf").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # ---- QoS ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- 里程计发布 ----
        self.sport_mode_state_sub_ = self.create_subscription(
            SportModeState, "lf/sportmodestate", self.state_callback, sensor_qos
        )
        self.odom_pub_ = self.create_publisher(Odometry, "/g1/sensor/odom", 10)
        self.tf_broadcaster_ = TransformBroadcaster(self)

        # ---- 关节状态发布 ----
        # G1 29DOF 关节名称（与 URDF 一致）
        self.joint_names_ = [
            # 左腿 (6)
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            # 右腿 (6)
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            # 腰部 (3)
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
            # 左臂 (7)
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            # 右臂 (7)
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        ]
        self.joint_count_ = len(self.joint_names_)  # 29
        self.joint_state_pub_ = self.create_publisher(JointState, "/joint_states", sensor_qos)
        self.low_state_sub_ = self.create_subscription(
            LowState, "lf/lowstate", self.low_state_callback, sensor_qos
        )

        self.get_logger().info(
            f"G1 驱动节点启动: {self.joint_count_} 关节, "
            f"odom_frame={self.odom_frame}, base_frame={self.base_frame}"
        )

    def state_callback(self, data: SportModeState) -> None:
        """里程计 + TF 回调。"""
        now = self.get_clock().now().to_msg()

        # 里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # 位置
        odom_msg.pose.pose.position.x = float(data.position[0])
        odom_msg.pose.pose.position.y = float(data.position[1])
        odom_msg.pose.pose.position.z = float(data.position[2])

        # 姿态 (四元数)
        odom_msg.pose.pose.orientation.w = float(data.imu_state.quaternion[0])
        odom_msg.pose.pose.orientation.x = float(data.imu_state.quaternion[1])
        odom_msg.pose.pose.orientation.y = float(data.imu_state.quaternion[2])
        odom_msg.pose.pose.orientation.z = float(data.imu_state.quaternion[3])

        # 线速度
        odom_msg.twist.twist.linear.x = float(data.velocity[0])
        odom_msg.twist.twist.linear.y = float(data.velocity[1])
        odom_msg.twist.twist.linear.z = float(data.velocity[2])

        # 角速度
        odom_msg.twist.twist.angular.z = float(data.yaw_speed)

        self.odom_pub_.publish(odom_msg)

        # TF 变换
        if self.publish_odom_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = float(data.position[0])
            tf_msg.transform.translation.y = float(data.position[1])
            tf_msg.transform.translation.z = float(data.position[2])
            tf_msg.transform.rotation = odom_msg.pose.pose.orientation
            self.tf_broadcaster_.sendTransform(tf_msg)

    def low_state_callback(self, data: LowState) -> None:
        """关节状态回调。"""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names_

        ms = data.motor_state
        for i in range(self.joint_count_):
            joint_state_msg.position.append(float(ms[i].q))

        self.joint_state_pub_.publish(joint_state_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
