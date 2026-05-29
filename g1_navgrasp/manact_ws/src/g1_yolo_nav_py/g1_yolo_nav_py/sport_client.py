"""G1 运动控制客户端 — Loco API 封装（参考 ctrl_keyboard 已验证可用方案）。

提供统一的运动控制接口，所有需要控制 G1 机器人运动的节点
都应使用本模块，避免重复定义 API 常量和 Request 发布逻辑。

全部使用 Loco API（ctrl_keyboard 已验证可用）：
    - 状态机控制: SET_FSM_ID(7101) — 参数 {"data": fsm_id}
    - 速度控制:   SET_VELOCITY(7105) — 参数 {"velocity": [vx, vy, vyaw], "duration": t}
    - 平衡模式:   SET_BALANCE_MODE(7102) — 参数 {"data": mode}
    - 速度模式:   SET_SPEED_MODE(7107) — 参数 {"data": mode}

使用方式：
    from g1_yolo_nav_py.sport_client import SportClient

    class MyNode(Node):
        def __init__(self):
            ...
            self._sport = SportClient(self)
            self._sport.auto_init_if_needed()  # 智能检测：已在走跑模式则跳过

        def move_forward(self):
            if self._sport.ready:
                self._sport.move(vx=0.3)

        def stop(self):
            self._sport.stop()
"""

class LocoAPI:
    """Loco Mode API ID 常量"""
    GET_FSM_ID = 7001
    GET_FSM_MODE = 7002
    GET_BALANCE_MODE = 7003
    GET_SWING_HEIGHT = 7004
    GET_STAND_HEIGHT = 7005
    GET_PHASE = 7006

    SET_FSM_ID = 7101         # 状态机控制
    SET_BALANCE_MODE = 7102   # 平衡模式
    SET_SWING_HEIGHT = 7103   # 摆动高度
    SET_STAND_HEIGHT = 7104   # 站立高度
    SET_VELOCITY = 7105       # 速度控制
    SET_SPEED_MODE = 7107     # 速度模式

class FSM_ID:
    """状态机 ID 映射"""
    ZERO_TORQUE = 0    # 零力矩模式（初始坐姿）
    DAMP = 1           # 阻尼控制模式（从坐姿准备站立的过渡）
    SQUAT = 2          # 蹲下
    SIT = 3            # 坐下（稳定坐姿）
    STAND_UP = 4       # 锁定站立
    START = 500        # 常规运控
    WALK_RUN = 801     # 走跑运控

class BalanceMode:
    """平衡模式常量。"""
    BALANCE_STAND = 0     # 平衡站立（速度为0时停止踏步）
    CONTINUOUS_GAIT = 1   # 连续步态（持续踏步）

# SportClient — Loco API 运动控制客户端
import json
import time
import threading
from typing import Optional, Callable

from rclpy.node import Node
from unitree_api.msg import Request

class SportClient:
    """G1 运动控制客户端 — Loco API 封装（参考 ctrl_keyboard 已验证方案）。

    功能：
        - 统一 Request 发布（每次创建新对象，避免 DDS 缓冲区复用问题）
        - FSM 状态机管理（DAMP → STAND_UP → WALK_RUN → CONTINUOUS_GAIT）
        - 速度控制（SET_VELOCITY，带 duration 参数）
        - 停止 / 坐下 / 站立

    使用：
        sport = SportClient(node)
        sport.init_fsm()       # 后台线程初始化 FSM
        sport.move(vx=0.3)     # 前进
        sport.stop()           # 停止
    """

    def __init__(self, node: Node) -> None:
        """初始化运动控制客户端。

        Args:
            node: ROS2 节点实例（用于创建 publisher 和日志）
        """
        self._node = node
        self._pub = node.create_publisher(
            Request, '/api/sport/request', 10
        )
        self._ready: bool = False
        self._sub_warned: bool = False  # 是否已警告过无订阅者
        # 默认 MOVE 持续时间（秒），参考 ctrl_keyboard: 连续=1.0，单次=0.2
        self._move_duration: float = 1.0
        self._stop_duration: float = 0.5
        # GET_FSM_ID 查询用：订阅响应话题，缓存最新 FSM ID
        self._current_fsm_id: Optional[int] = None
        self._fsm_id_event = threading.Event()
        node.create_subscription(
            Request, '/api/sport/response', self._on_sport_response, 10
        )

    @property
    def ready(self) -> bool:
        """FSM 是否已初始化完成，可以接受运动指令。"""
        return self._ready

    def _on_sport_response(self, msg: Request) -> None:
        """处理 /api/sport/response 话题的响应消息。"""
        try:
            api_id = msg.header.identity.api_id
            if api_id == LocoAPI.GET_FSM_ID and msg.parameter:
                data = json.loads(msg.parameter)
                fsm_id = data.get("data")
                if fsm_id is not None:
                    self._current_fsm_id = int(fsm_id)
                    self._fsm_id_event.set()
        except Exception:
            pass

    def get_fsm_id(self, timeout: float = 2.0) -> Optional[int]:
        """查询机器人当前 FSM 状态 ID。

        通过 GET_FSM_ID(7001) 请求查询，等待 /api/sport/response 响应。

        Args:
            timeout: 等待响应的超时时间（秒）

        Returns:
            当前 FSM ID（如 801=WALK_RUN），查询失败返回 None
        """
        self._fsm_id_event.clear()
        self._current_fsm_id = None
        self.publish(LocoAPI.GET_FSM_ID)
        self._fsm_id_event.wait(timeout=timeout)
        return self._current_fsm_id

    def is_walk_run_mode(self, timeout: float = 2.0) -> bool:
        """检测机器人当前是否已处于走跑模式（WALK_RUN=801）。

        Args:
            timeout: 查询超时时间（秒）

        Returns:
            True 表示已处于 WALK_RUN 模式
        """
        fsm_id = self.get_fsm_id(timeout=timeout)
        if fsm_id == FSM_ID.WALK_RUN:
            return True
        return False

    def auto_init_if_needed(self, auto_balance_stand: bool = True,
                            callback: Optional[Callable[[], None]] = None) -> None:
        """智能初始化：检测当前模式，已处于走跑模式则跳过，否则执行完整初始化。

        Args:
            auto_balance_stand: 初始化时是否自动切换到走跑模式并开启连续步态
            callback: FSM 初始化完成后的回调
        """
        def _check_and_init():
            logger = self._node.get_logger
            logger().info("[FSM] 等待 /api/sport/request 订阅者...")
            for _ in range(50):
                if self.has_subscribers():
                    logger().info("[FSM] 检测到订阅者")
                    break
                time.sleep(0.1)
            else:
                logger().warn("[FSM] 等待 5 秒后仍无订阅者!")

            logger().info("[FSM] 查询当前 FSM 状态...")
            fsm_id = self.get_fsm_id(timeout=3.0)

            if fsm_id == FSM_ID.WALK_RUN:
                logger().info(
                    f"[FSM] 当前已处于走跑模式 (FSM_ID={fsm_id})，跳过初始化"
                )
                self._ready = True
            else:
                detected = f"FSM_ID={fsm_id}" if fsm_id is not None else "查询超时"
                logger().info(
                    f"[FSM] 当前不在走跑模式 ({detected})，执行完整初始化..."
                )
                # 复用 init_fsm 的逻辑（不启动新线程，因为已在后台线程中）
                self._do_full_init(auto_balance_stand)

            if callback is not None:
                callback()

        t = threading.Thread(target=_check_and_init, daemon=True)
        t.start()

    def _do_full_init(self, auto_balance_stand: bool = True) -> None:
        """执行完整的 FSM 初始化流程（在当前线程中同步执行）。"""
        logger = self._node.get_logger

        logger().info("[FSM] 步骤1: 切换到 DAMP 模式 (SET_FSM_ID=1)...")
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.DAMP})
        time.sleep(3)

        logger().info("[FSM] 步骤2: 站立 (SET_FSM_ID STAND_UP=4)...")
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.STAND_UP})
        time.sleep(5)

        if auto_balance_stand:
            logger().info("[FSM] 步骤3: 进入常规运控 (SET_FSM_ID START=500)...")
            self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.START})
            time.sleep(2)

            logger().info("[FSM] 步骤4: 进入走跑模式 (SET_FSM_ID WALK_RUN=801)...")
            self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.WALK_RUN})
            time.sleep(1)

            logger().info("[FSM] 步骤5: 开启连续步态 (SET_BALANCE_MODE CONTINUOUS_GAIT=1)...")
            self.publish(LocoAPI.SET_BALANCE_MODE, {"data": BalanceMode.CONTINUOUS_GAIT})
            time.sleep(1)

        self._ready = True
        logger().info("[FSM] 初始化完成，就绪")


    def has_subscribers(self) -> bool:
        """检查 /api/sport/request 是否有订阅者。"""
        try:
            count = self._node.count_subscribers('/api/sport/request')
            return count > 0
        except Exception:
            return True  # 无法判断时假设正常

    def publish(self, api_id: int, params: Optional[dict] = None) -> None:
        """发布运动控制请求。

        每次调用都创建新的 Request 对象，避免 DDS 缓冲区复用导致的数据损坏。
        发布后等待 0.1 秒，确保 DDS 消息被发出（与 ctrl_keyboard 一致）。

        Args:
            api_id: API ID（如 LocoAPI.SET_VELOCITY, LocoAPI.SET_FSM_ID）
            params: 参数字典，会被 JSON 序列化
        """
        req = Request()
        req.header.identity.api_id = api_id
        if params is not None:
            req.parameter = json.dumps(params)
        self._pub.publish(req)
        time.sleep(0.1)  # 与 ctrl_keyboard 一致：确保 DDS 消息被发出

    #  FSM 状态机初始化（Loco API，参考 ctrl_keyboard 已验证方案）
    def init_fsm(self, auto_balance_stand: bool = True,
                 callback: Optional[Callable[[], None]] = None) -> None:
        """在后台线程执行 FSM 初始化（Loco API 方式，参考 ctrl_keyboard）。

        流程（与 ctrl_keyboard.auto_ctrl 一致）：
            SET_FSM_ID(DAMP) → SET_FSM_ID(STAND_UP) → SET_FSM_ID(WALK_RUN) → SET_BALANCE_MODE(CONTINUOUS_GAIT)

        Args:
            auto_balance_stand: 是否自动切换到走跑模式并开启连续步态
            callback: FSM 初始化完成后的回调（在后台线程中调用）
        """
        def _init_thread():
            logger = self._node.get_logger

            logger().info("[FSM] 等待 /api/sport/request 订阅者...")
            for i in range(50):
                if self.has_subscribers():
                    logger().info("[FSM] 检测到订阅者，开始初始化")
                    break
                time.sleep(0.1)
            else:
                logger().warn(
                    "[FSM] 等待 5 秒后仍无订阅者! "
                    "FSM 指令可能丢失，请确认 unitree SDK bridge 已启动。"
                )

            self._do_full_init(auto_balance_stand)

            if callback is not None:
                callback()

        t = threading.Thread(target=_init_thread, daemon=True)
        t.start()

    #  Loco API 速度控制（参考 ctrl_keyboard 已验证方案）
    def move(self, vx: float = 0.0, vy: float = 0.0,
             vyaw: float = 0.0, duration: Optional[float] = None) -> None:
        """通过 Loco API SET_VELOCITY 控制机器人运动。

        需要机器人已在 WALK_RUN 模式下。

        Args:
            vx: 前进速度 (m/s)，正=前进，负=后退
            vy: 侧移速度 (m/s)，正=左，负=右
            vyaw: 偏航角速度 (rad/s)，正=左转，负=右转
            duration: 运动持续时间（秒），None 则使用默认值
        """
        if not self._sub_warned and not self.has_subscribers():
            self._sub_warned = True
            self._node.get_logger().warn(
                "[SportClient] /api/sport/request 无订阅者! "
                "运动指令不会被机器人执行。"
                "请确认 unitree SDK bridge 已启动 "
                "(检查: ros2 topic info /api/sport/request)"
            )
        dur = duration if duration is not None else self._move_duration
        self.publish(LocoAPI.SET_VELOCITY,
                     {"velocity": [vx, vy, vyaw], "duration": dur})

    def stop(self, duration: Optional[float] = None) -> None:
        """发送零速度指令停止运动（SET_VELOCITY velocity=[0,0,0]）。

        Args:
            duration: 停止持续时间，None 则使用默认值
        """
        dur = duration if duration is not None else self._stop_duration
        self.publish(LocoAPI.SET_VELOCITY,
                     {"velocity": [0.0, 0.0, 0.0], "duration": dur})

    def skip_init(self) -> None:
        """跳过 FSM 初始化，手动标记为就绪状态。

        用于机器人已处于 WALK_RUN + CONTINUOUS_GAIT 模式时，
        避免重复执行 DAMP → STAND_UP → WALK_RUN 流程。
        """
        self._ready = True

    #  FSM 状态切换（全部通过 SET_FSM_ID）
    def sit(self) -> None:
        """发送 SIT 指令（SET_FSM_ID SIT=3）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.SIT})

    def stand_up(self) -> None:
        """发送 STAND_UP 指令（SET_FSM_ID STAND_UP=4）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.STAND_UP})

    def stand_down(self) -> None:
        """发送 SQUAT 指令（SET_FSM_ID SQUAT=2）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.SQUAT})

    def damp(self) -> None:
        """发送 DAMP 指令（SET_FSM_ID DAMP=1）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.DAMP})

    def walk_run(self) -> None:
        """发送 WALK_RUN 指令（SET_FSM_ID WALK_RUN=801）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.WALK_RUN})

    def start_mode(self) -> None:
        """发送 START 指令（SET_FSM_ID START=500，常规运控）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.START})

    #  平衡模式切换（SET_BALANCE_MODE）
    def continuous_gait(self) -> None:
        """开启连续步态（SET_BALANCE_MODE CONTINUOUS_GAIT=1）。"""
        self.publish(LocoAPI.SET_BALANCE_MODE, {"data": BalanceMode.CONTINUOUS_GAIT})

    def balance_stand(self) -> None:
        """切换到平衡站立（SET_BALANCE_MODE BALANCE_STAND=0）。"""
        self.publish(LocoAPI.SET_BALANCE_MODE, {"data": BalanceMode.BALANCE_STAND})

    def set_speed_mode(self, mode: int) -> None:
        """设置速度模式（SET_SPEED_MODE）。

        Args:
            mode: 速度模式 0=1.0m/s, 1=2.0m/s, 2=2.7m/s, 3=3.0m/s
        """
        self.publish(LocoAPI.SET_SPEED_MODE, {"data": mode})

    def set_stand_height(self, height: float) -> None:
        """设置站立高度（SET_STAND_HEIGHT）。

        Args:
            height: 站立高度（0.4~0.7m）
        """
        self.publish(LocoAPI.SET_STAND_HEIGHT, {"data": height})
