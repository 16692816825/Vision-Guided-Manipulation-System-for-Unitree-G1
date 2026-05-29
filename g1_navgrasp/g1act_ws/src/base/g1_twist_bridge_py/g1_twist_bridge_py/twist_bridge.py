"""G1 Twist Bridge — 将 geometry_msgs/Twist 转换为 unitree_api/Request。"""

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
from geometry_msgs.msg import Twist
from .sport_model import ROBOT_SPORT_API_IDS
import json


class TwistBridge(Node):
    """Twist → unitree_api Request 桥接节点。"""

    def __init__(self) -> None:
        super().__init__("g1_twist_bridge_node")
        self.request_pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.twist_sub = self.create_subscription(
            Twist, "/cmd_vel", self.twist_cb, 10
        )
        self.get_logger().info("G1 Twist Bridge 节点启动")

    def twist_cb(self, twist: Twist) -> None:
        request = Request()
        x = twist.linear.x
        y = twist.linear.y
        z = twist.angular.z
        api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]
        if x != 0 or y != 0 or z != 0:
            api_id = ROBOT_SPORT_API_IDS["MOVE"]
            js = {"x": x, "y": y, "z": z}
            request.parameter = json.dumps(js)
        request.header.identity.api_id = api_id
        self.request_pub.publish(request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwistBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
