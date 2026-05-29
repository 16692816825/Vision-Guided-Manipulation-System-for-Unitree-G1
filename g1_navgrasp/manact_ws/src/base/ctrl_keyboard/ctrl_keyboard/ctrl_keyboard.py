import sys
import threading
import json
import time

from unitree_api.msg import Request
import rclpy
from rclpy.node import Node

# G1运动控制API ID
ROBOT_LOCO_API_IDS = {
    "SET_VELOCITY": 7105,
}

class G1AutoWalkNode(Node):
    def __init__(self):
        super().__init__('g1_auto_walk')
        self.sport_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.sport_req = Request()

    def publish_sport_request(self, api_id, params):
        self.sport_req.header.identity.api_id = api_id
        self.sport_req.parameter = json.dumps(params)
        self.sport_pub.publish(self.sport_req)
        time.sleep(0.1)

    def auto_move_sequence(self):
        print("=== 直接开始自动运动（已在走跑模式）===")

        # ====================== 直接执行动作 ======================
        print("\n【1】向前走 3 秒")
        self.publish_sport_request(ROBOT_LOCO_API_IDS["SET_VELOCITY"], 
            {"velocity": [0.6, 0, 0], "duration": 4})
        time.sleep(3)

        print("\n【2】左转 90 度")
        self.publish_sport_request(ROBOT_LOCO_API_IDS["SET_VELOCITY"], 
            {"velocity": [0, 0, 1.2], "duration": 2})
        time.sleep(1.5)

        print("\n【3】向前走 6 秒")
        self.publish_sport_request(ROBOT_LOCO_API_IDS["SET_VELOCITY"], 
            {"velocity": [0.6, 0, 0], "duration": 7})
        time.sleep(6)
        # ==========================================================

        print("\n=== 动作完成，停止 ===")
        self.publish_sport_request(ROBOT_LOCO_API_IDS["SET_VELOCITY"], 
            {"velocity": [0, 0, 0], "duration": 1})

def main():
    rclpy.init()
    node = G1AutoWalkNode()
    
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    try:
        node.auto_move_sequence()
        time.sleep(1)
        
    except KeyboardInterrupt:
        print("\n手动停止")
        
    finally:
        node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_VELOCITY"], 
            {"velocity": [0, 0, 0], "duration": 1})
        
        rclpy.shutdown()
        spinner.join()

if __name__ == '__main__':
    main()

