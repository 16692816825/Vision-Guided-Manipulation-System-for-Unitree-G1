import sys
import threading
import json
import time

from unitree_api.msg import Request
import rclpy
from rclpy.node import Node

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

# G1运动控制API ID
ROBOT_LOCO_API_IDS = {
    "GET_FSM_ID": 7001,
    "GET_FSM_MODE": 7002,
    "GET_BALANCE_MODE": 7003,
    "GET_SWING_HEIGHT": 7004,
    "GET_STAND_HEIGHT": 7005,
    "GET_PHASE": 7006,
    
    "SET_FSM_ID": 7101,
    "SET_BALANCE_MODE": 7102,
    "SET_SWING_HEIGHT": 7103,
    "SET_STAND_HEIGHT": 7104,
    "SET_VELOCITY": 7105,
    "SET_SPEED_MODE": 7107,
}

# 状态机ID映射
FSM_ID = {
    "ZERO_TORQUE": 0,
    "DAMP": 1,
    "SQUAT": 2,
    "SIT": 3,
    "STAND_UP": 4,
    "START": 500,
    "WALK_RUN": 801
}

# 平衡模式
BALANCE_MODE = {
    "BALANCE_STAND": 0,
    "CONTINUOUS_GAIT": 1
}

class G1AutoWalkNode(Node):
    def __init__(self):
        super().__init__('g1_auto_walk')
        self.sport_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.sport_req = Request()
        self.current_mode = "ZERO_TORQUE"

    def publish_sport_request(self, api_id, params):
        """发布运动控制指令"""
        self.sport_req.header.identity.api_id = api_id
        self.sport_req.parameter = json.dumps(params)
        self.sport_pub.publish(self.sport_req)
        time.sleep(0.1)

    def auto_move_sequence(self):
        """自动行走主流程"""
        print("=== 开始自动行走任务 ===")
        print("步骤1：切换到阻尼模式")
        self.publish_sport_request(ROBOT_LOCO_API_IDS["SET_FSM_ID"], {"data": FSM_ID["DAMP"]})
        self.current_mode = "DAMP"
        time.sleep(2)

        print("步骤2：站立")
        self.publish_sport_request(ROBOT_LOCO_API_IDS["SET_FSM_ID"], {"data": FSM_ID["STAND_UP"]})
        self.current_mode = "STAND_UP"
        time.sleep(3)

        print("步骤3：进入走跑模式")
        self.publish_sport_request(ROBOT_LOCO_API_IDS["SET_FSM_ID"], {"data": FSM_ID["WALK_RUN"]})
        self.current_mode = "WALK_RUN"
        time.sleep(1)

        print("步骤4：开启连续步态")
        self.publish_sport_request(ROBOT_LOCO_API_IDS["SET_BALANCE_MODE"], {"data": BALANCE_MODE["CONTINUOUS_GAIT"]})
        time.sleep(1)

        # ====================== 核心自动动作 ======================
        print("\n【动作1】向前行走 3 秒")
        self.publish_sport_request(ROBOT_LOCO_API_IDS["SET_VELOCITY"], 
            {"velocity": [0.6, 0, 0], "duration": 5})
        time.sleep(3)

        print("\n【动作2】左转 90 度")
        self.publish_sport_request(ROBOT_LOCO_API_IDS["SET_VELOCITY"], 
            {"velocity": [0, 0, 1.2], "duration": 2})
        time.sleep(1.5)

        print("\n【动作3】向前行走 6 秒")
        self.publish_sport_request(ROBOT_LOCO_API_IDS["SET_VELOCITY"], 
            {"velocity": [0.6, 0, 0], "duration": 8})
        time.sleep(6)
        # ==========================================================

        print("\n=== 任务完成，停止运动 ===")
        self.publish_sport_request(ROBOT_LOCO_API_IDS["SET_VELOCITY"], 
            {"velocity": [0, 0, 0], "duration": 1})
        time.sleep(1)

        print("坐下")
        self.publish_sport_request(ROBOT_LOCO_API_IDS["SET_FSM_ID"], {"data": FSM_ID["SIT"]})

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = saveTerminalSettings()
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
        node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_FSM_ID"], {"data": FSM_ID["SIT"]})
        
        rclpy.shutdown()
        spinner.join()
        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()

