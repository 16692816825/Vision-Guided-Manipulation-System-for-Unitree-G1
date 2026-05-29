# ui/main_window.py
import datetime
import os
import cv2
import numpy as np
# [修复] 在这里添加了 QPushButton
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QProgressBar, QTextEdit, QFrame, QSizePolicy, QTabWidget, 
                             QDesktopWidget, QMessageBox, QDialog, QPushButton)
from PyQt5.QtCore import Qt, pyqtSlot, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QColor

# === 核心依赖 ===
from core.worker import ControllerWorker 
from config import G1_JOINT_MAP
from core.settings_manager import settings
# === 视觉与AI ===
# 懒加载检测器
try:
    from core.vision_detector import ObjectDetector
except ImportError:
    ObjectDetector = None
    print("[WARN] core.vision_detector not found.")

# DeepSeek 大脑
try:
    from core.deepseek_brain import G1DeepSeekBrain
except ImportError:
    G1DeepSeekBrain = None

# SDK 音频
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

# === UI 组件 ===
from ui.styles import Styles
from ui.widgets import (HoldButton, SpeedControlWidget, MuJoCoWidget, DashCircle, 
                        OscilloscopeWidget, CommStatusWidget, HealthMatrixWidget, FsmSwitchWidget)

# === 拆分后的 Tabs ===
from ui.tabs.monitor_tab import MonitorTab
from ui.tabs.joints_tab import JointsTab
from ui.tabs.teach_tab import TeachTab
from ui.tabs.precision_tab import PrecisionTab
from ui.tabs.nav_tab import NavTab
from ui.tabs.hand_tab import HandTab
from ui.tabs.voice_tab import VoiceTab

# =========================================================================
# 辅助线程: 语音处理
# =========================================================================
class VoiceWorkThread(QThread):
    result_signal = pyqtSignal(str, str)  # (回复文本, 动作指令)

    def __init__(self, brain, text):
        super().__init__()
        self.brain = brain
        self.text = text

    def run(self):
        if self.brain:
            reply, action = self.brain.process(self.text)
            self.result_signal.emit(reply, action)
        else:
            self.result_signal.emit("错误：大脑未加载", None)

# =========================================================================
# 主窗口: 组装厂 (Coordinator)
# =========================================================================
class MainWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.setWindowTitle("G1 混合控制终端 V7.7 [Componentized]")
        self.resize(1600, 950)
        self.setMinimumSize(1280, 800)
        
        # 状态变量
        self.current_fsm_id = -1
        self.is_live_detect_active = False # 实时检测开关状态
        self.detector = None               # YOLO 检测器实例
        
        # 应用样式
        self.setStyleSheet(Styles.GLOBAL)
        self.center_window()

        # 初始化布局
        self._init_ui()
        
        # 初始化后台系统
        self._init_backend()

    # -------------------------------------------------------------------------
    # 1. UI 初始化 (Layout Construction)
    # -------------------------------------------------------------------------
    def _init_ui(self):
        cw = QWidget()
        self.setCentralWidget(cw)
        main_layout = QHBoxLayout(cw)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        # A. 左侧面板 (Left Panel)
        left_panel = self._create_left_panel()
        main_layout.addWidget(left_panel)

        # B. 中间面板 (Center Panel - Simulation)
        center_panel = self._create_center_panel()
        main_layout.addWidget(center_panel, 1) # stretch factor 1

        # C. 右侧面板 (Right Panel - Tabs & Log)
        right_panel = self._create_right_panel()
        main_layout.addWidget(right_panel)

    def _create_left_panel(self):
        frame = QFrame()
        frame.setObjectName("Panel")
        frame.setFixedWidth(340)
        layout = QVBoxLayout(frame)
        layout.setSpacing(15)

        # 标题
        title_box = QVBoxLayout(); title_box.setSpacing(2)
        title_box.addWidget(QLabel("UNITREE G1", styleSheet="font-size:24px; font-weight:900; color:#fff; font-style:italic;"))
        title_box.addWidget(QLabel("HYBRID TERMINAL V7.0", styleSheet="font-size:12px; color:#00e5ff; letter-spacing:2px;"))
        layout.addLayout(title_box)

        # 仪表盘
        self.comm_widget = CommStatusWidget()
        layout.addWidget(self.comm_widget)

        dash_layout = QHBoxLayout()
        self.dash_soc = DashCircle("电量", "%", "#00e676")
        self.dash_temp = DashCircle("温度", "°C", "#ff5252")
        self.dash_volt = DashCircle("电压", "V", "#ffeb3b")
        dash_layout.addWidget(self.dash_soc); dash_layout.addWidget(self.dash_temp); dash_layout.addWidget(self.dash_volt)
        layout.addLayout(dash_layout)

        # 诊断
        layout.addWidget(QLabel("系统诊断 (DIAGNOSTICS)", objectName="SubTitle"))
        self.health_matrix = HealthMatrixWidget()
        layout.addWidget(self.health_matrix)

        # 主控开关
        layout.addWidget(QLabel("主控开关 (MASTER CONTROL)", objectName="SubTitle"))
        self.lbl_state = QLabel("STATE: STANDBY")
        self.lbl_state.setAlignment(Qt.AlignCenter)
        self.lbl_state.setStyleSheet("background:#111; padding:8px; font-weight:bold; color:#aaa; border:1px solid #333; border-radius:4px;")
        layout.addWidget(self.lbl_state)
        
        btn_layout = QHBoxLayout()
        self.btn_start = HoldButton("长按启动")
        self.btn_stop = HoldButton("长按关机", color_fill="#ff5252")
        self.btn_stop.setEnabled(False)
        
        # 信号连接 (Start/Stop)
        self.btn_start.triggered.connect(self.on_start_btn_triggered)
        self.btn_stop.triggered.connect(self.on_stop_btn_triggered)
        
        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_stop)
        layout.addLayout(btn_layout)

        # 速度 & FSM
        layout.addWidget(QLabel("速度限制 (VELOCITY LIMITS)", objectName="SubTitle"))
        self.spd_arm = SpeedControlWidget("手臂灵敏度", "#00e5ff")
        self.spd_walk = SpeedControlWidget("底盘速度", "#00e676")
        layout.addWidget(self.spd_arm)
        layout.addWidget(self.spd_walk)
        
        layout.addSpacing(10)
        self.fsm_switch = FsmSwitchWidget("运控模式 (LOCO MODE)", "#d500f9")
        self.fsm_switch.set_current_mode(801) # 默认
        layout.addWidget(self.fsm_switch)

        layout.addStretch()
        ver_lbl = QLabel("BUILD: 2025.11.24 | ROS1-NAV")
        ver_lbl.setAlignment(Qt.AlignCenter)
        ver_lbl.setStyleSheet("color:#555; font-size:10px;")
        layout.addWidget(ver_lbl)
        
        return frame

    def _create_center_panel(self):
        frame = QFrame()
        frame.setObjectName("Panel")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(4, 4, 4, 4)

        # 顶部信息条
        hl = QHBoxLayout()
        self.lbl_mode = QLabel("MODE: WALKING")
        self.lbl_reach = QLabel("DIST: N/A")
        hl.addWidget(self.lbl_mode)
        hl.addStretch()
        hl.addWidget(self.lbl_reach)
        layout.addLayout(hl)

        # MuJoCo 窗口
        self.mj = MuJoCoWidget()
        
        # 悬浮视角切换按钮
        self.btn_view = QPushButton("切换视角", self.mj)
        self.btn_view.setCursor(Qt.PointingHandCursor)
        self.btn_view.setStyleSheet("QPushButton { background-color: rgba(0,0,0,150); color: white; border: 1px solid #555; border-radius: 4px; padding: 5px; font-weight: bold; } QPushButton:hover { background-color: rgba(0, 229, 255, 150); color: black; }")
        self.btn_view.resize(80, 30)
        self.btn_view.move(10, 10)
        
        layout.addWidget(self.mj)

        # 权重条
        wl = QHBoxLayout()
        wl.addWidget(QLabel("Authority:"))
        self.bar_w = QProgressBar()
        self.bar_w.setRange(0, 100)
        wl.addWidget(self.bar_w)
        layout.addLayout(wl)
        
        return frame

    def _create_right_panel(self):
        frame = QFrame()
        frame.setObjectName("Panel")
        frame.setFixedWidth(580)
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(5, 5, 5, 5)

        # 1. 实例化各个 Tab (组件化核心)
        self.tabs = QTabWidget()
        self.tabs.setFocusPolicy(Qt.NoFocus)

        self.tab_monitor = MonitorTab()
        self.tab_joints = JointsTab()
        self.tab_teach = TeachTab()
        self.tab_precision = PrecisionTab()
        self.tab_nav = NavTab()
        self.tab_hand = HandTab()
        self.tab_voice = VoiceTab()
        self.tab_scope = OscilloscopeWidget() # 复用 widgets 里的

        self.tabs.addTab(self.tab_monitor, "数据监控")
        self.tabs.addTab(self.tab_joints, "关节详情")
        self.tabs.addTab(self.tab_teach, "示教模式")
        self.tabs.addTab(self.tab_precision, "精确控制")
        self.tabs.addTab(self.tab_nav, "智能导航")
        self.tabs.addTab(self.tab_hand, "灵巧手L10")
        self.tabs.addTab(self.tab_voice, "AI 语音助手")
        self.tabs.addTab(self.tab_scope, "波形")

        layout.addWidget(self.tabs)

        # 2. 日志区
        layout.addWidget(QLabel("System Log", objectName="SubTitle"))
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setFocusPolicy(Qt.NoFocus)
        self.log.setFixedHeight(150)
        layout.addWidget(self.log)

        return frame

    # -------------------------------------------------------------------------
    # 2. 系统后台初始化
    # -------------------------------------------------------------------------
    def _init_backend(self):
        # A. Worker 线程
        self.worker = ControllerWorker(self.args)
        
        # B. 信号连接 (核心路由)
        self._connect_signals()
        
        # C. 启动 Worker
        self.worker.start()

        # D. 额外模块初始化 (AI/Audio)
        self.audio_client = None
        try:
            self.audio_client = AudioClient()
            self.audio_client.SetTimeout(5.0)
            self.audio_client.Init()
            print("[System] AudioClient Init Success")
        except: pass

        self.voice_brain = None
        if G1DeepSeekBrain:
            # 从配置读取 Key
            api_key = settings.get("ai", "deepseek_api_key")
            
            if api_key and len(api_key) > 5:
                self.voice_brain = G1DeepSeekBrain(api_key)
                self.log_msg("[Voice] DeepSeek 大脑已加载 (Key Loaded)")
            else:
                self.log_msg("[Voice] 未配置 DeepSeek API Key，语音功能受限")

    def _connect_signals(self):
        # 1. Worker -> Main Window
        self.worker.log_signal.connect(self.log_msg)
        self.worker.popup_signal.connect(lambda t, m: QMessageBox.information(self, t, m))
        self.worker.finished_signal.connect(self.close)
        self.worker.status_signal.connect(self.dispatch_status_update) # 分发数据
        self.worker.frame_signal.connect(self.process_and_display_frame) # 图像处理

        # 2. Left Panel -> Worker
        self.spd_arm.levelChanged.connect(self.worker.set_arm_speed_level)
        self.spd_walk.levelChanged.connect(self.worker.set_walk_speed_level)
        
        # [修改] 改为连接到本地的拦截函数，进行安全检查
        self.fsm_switch.modeChanged.connect(self.handle_fsm_change)

        # 3. Center Panel -> Worker
        self.mj.camera_control.connect(lambda a, dx, dy: self.worker.move_camera(a, dx, dy))
        self.mj.vision_click_signal.connect(self.worker.send_vision_click)
        self.btn_view.clicked.connect(self.worker.toggle_view_mode)

        # 4. Tab Signals -> Worker / Local Logic
        
        # Tab 3: Teach
        self.tab_teach.sig_start_teach.connect(self.worker.start_teaching)
        self.tab_teach.sig_stop_teach.connect(lambda n: self.worker.stop_teaching(n))
        self.tab_teach.sig_start_replay.connect(lambda n: self.worker.start_replay(n))
        self.tab_teach.sig_exit_teach.connect(self.worker.exit_teaching_mode)

        # Tab 4: Precision
        self.tab_precision.sig_ik_move.connect(lambda x,y,z,r,d,traj: 
            self.worker.send_traj_command(x,y,z,r,d) if traj else self.worker.send_ik_command(x,y,z,0,0,r,d))
        
        self.tab_precision.sig_ik_preview.connect(lambda x,y,z,r,rt:
            self.worker.send_ik_tracking(x,y,z,r) if rt else self.worker.send_ik_preview(x,y,z,r))
            
        # 修正: PrecisionTab 发出的信号是 list，需要直接放入队列，因为 worker.send_joint_sequence 可能期望特定格式
        # worker.send_joint_sequence 实际上就是 put CMD_EXECUTE_JOINT_SEQUENCE
        self.tab_precision.sig_run_sequence.connect(self.worker.send_joint_sequence)
        
        self.tab_precision.sig_arm_init.connect(self.worker.trigger_arm_init_sequence)
        # [修改] sig_arm_reset -> 执行收回序列
        self.tab_precision.sig_arm_reset.connect(self.worker.trigger_arm_exit_sequence)
        
        # [新增] sig_full_exit -> 退出示教模式 (CMD_EXIT_TEACH)
        self.tab_precision.sig_full_exit.connect(self.worker.exit_teaching_mode)
        self.tab_precision.sig_vis_clear.connect(lambda: self.worker.cmd_queue.put(('CMD_VIS_CLEAR', None)))
        
        # 视觉检测相关
        self.tab_precision.sig_live_detect_toggled.connect(self._on_live_detect_toggled)
        self.tab_precision.sig_req_auto_grasp.connect(self._perform_auto_grasp_check)

        # Tab 5: Nav
        self.tab_nav.sig_start_mapping.connect(self.worker.start_mapping_mode)
        self.tab_nav.sig_save_map.connect(self.worker.save_ros_map)
        self.tab_nav.sig_start_nav.connect(self.worker.start_nav_mode)
        self.tab_nav.sig_stop_system.connect(self.worker.stop_nav_system)
        self.tab_nav.sig_pub_goal.connect(self.worker.send_nav_goal)
        self.tab_nav.sig_pub_pose.connect(self.worker.send_initial_pose)
        # [在这里添加] 连接 NavTab 的测试动作信号 -> Worker 的回放功能
        self.tab_nav.sig_play_action.connect(self._on_nav_play_action)
        # [新增] 连接 NavTab 的任务执行信号
        self.tab_nav.sig_req_task_exec.connect(self.worker.send_task_chain)
        # Tab 6: Hand
        self.tab_hand.sig_hand_cmd.connect(lambda a, mm: self.worker.cmd_queue.put(('CMD_HAND_ACT', {'action': a, 'diameter': mm})))

        # Tab 7: Voice
        self.tab_voice.sig_user_input.connect(self._handle_voice_input)

        # Tab Scope (Data handled in dispatch)
    def handle_fsm_change(self, target_id):
        """
        [新增] FSM 切换请求拦截器
        防止在导航过程中误切回 801 导致车辆失控/静止
        """
        # 1. 获取当前控制模式 (从 worker 的缓存状态中取)
        # 这里的 last_status 是由 worker 线程实时更新的
        current_status = self.worker.last_status
        current_mode = current_status.get('mode', 'WALKING')
        
        # 2. 检查冲突条件：正在导航 且 目标不是 500
        if current_mode == 'NAVIGATING' and target_id != 500:
            # A. 弹出警告
            QMessageBox.critical(self, "禁止操作", 
                                 f"🚫 导航模式下禁止切换运控状态！\n\n"
                                 f"导航强依赖于 [FSM 500]。\n"
                                 f"切换至 {target_id} 会导致机器人忽略导航指令而停止移动。\n\n"
                                 "请先停止导航，再进行模式切换。")
            
            # B. [关键] 将界面上的按钮强制按回 500 (因为用户刚才点击了801)
            # 我们需要临时断开信号，防止死循环，或者依赖 FsmSwitchWidget 的 set_current_mode 实现
            self.fsm_switch.set_current_mode(500)
            return

        # 3. 如果通过检查，才真正发送指令给后台
        self.worker.switch_fsm_mode(target_id)
    # -------------------------------------------------------------------------
    # 3. 核心循环回调 (Core Loop Callbacks)
    # -------------------------------------------------------------------------
    @pyqtSlot(dict)
    def dispatch_status_update(self, data):
        """
        接收 Worker 发来的全量数据，分发给各个 UI 组件
        """
        # 1. 更新主窗口状态 (Left Panel, Center Panel)
        self._update_global_status(data)
        
        # 2. 分发给当前激活的 Tab (节省性能)
        idx = self.tabs.currentIndex()
        if idx == 0: self.tab_monitor.update_data(data)
        elif idx == 1: self.tab_joints.update_data(data)
        elif idx == 2: self.tab_teach.update_data(data)
        elif idx == 3: self.tab_precision.update_data(data)
        elif idx == 4:  # NavTab
            nav_data = data.get('nav_data')
            if nav_data:
                # 把控制模式塞进去 (用于显示 CTRL: 自动/手动)
                nav_data['control_mode'] = data.get('mode', 'WALKING')
            
            self.tab_nav.update_data(nav_data)
            
            # 【重点检查这一行！！！】
            # 必须手动把 FSM ID 传给 NavTab，否则它永远是 -1
            self.tab_nav.set_current_fsm(self.current_fsm_id) 
        elif idx == 5: self.tab_hand.update_data(data)
        # Tab 6 (Voice) 不需要高频数据
        elif idx == 7: 
            pg, ph = data.get('p_goal', [0,0,0]), data.get('p_hand', [0,0,0])
            self.tab_scope.update_data(pg, ph)

        # 3. 处理特殊一次性消息
        msg_type = data.get('type')
        if msg_type == 'NAV_SAVED_AND_STOPPED':
            self.tab_nav.handle_saved_and_stopped()
        elif msg_type == 'NAV_STOPPED':
            self.tab_nav.handle_stopped()

    def _update_global_status(self, s):
        # FSM ID
        self.current_fsm_id = s.get('fsm_id', -1)
        if self.current_fsm_id in [801, 500]:
            self.fsm_switch.set_current_mode(self.current_fsm_id)

        # App State
        st = s.get('app_state', 'IDLE')
        c = "#00e676" if st == 'RUNNING' else "#aaa"
        if st == 'STOPPING': c = "#ff5252"
        self.lbl_state.setText(st)
        self.lbl_state.setStyleSheet(f"background:#111; padding:10px; font-weight:bold; color:{c}; border:1px solid {c}; border-radius:5px;")

        # Buttons Logic
        if st == 'IDLE': 
            self.btn_start.setEnabled(True); self.btn_stop.setEnabled(False)
            self.mj.set_overlay_text("OFFLINE")
        elif st in ['RUNNING', 'STOPPING']: 
            self.btn_start.setEnabled(False); self.btn_stop.setEnabled(True)
            self.mj.set_overlay_text("")
            
            # Mode Display
            mode = s.get('mode', 'WALKING')
            self.lbl_mode.setText(f"MODE: {mode}")
            
            # Visual Mode
            visual_type = 'WALK'
            if mode in ['ARM_CONTROL', 'IK_MOVING', 'IK_HOLD', 'IK_TRACKING']: visual_type = 'ARM'
            elif mode in ['TEACHING', 'REPLAYING', 'TRANSITION']: visual_type = 'TEACH'
            elif mode == 'NAVIGATING': visual_type = 'NAV'
            self.mj.set_visual_mode(visual_type)

        self.bar_w.setValue(int(s.get('weight', 0) * 100))

                # Hardware Vitals
        vitals = s.get('vitals', {})
        joints = s.get('joints', {})
        
        if vitals:
            self.dash_soc.set_val(vitals.get('soc', 0))
            self.dash_temp.set_val(vitals.get('max_temp', 0))
            self.dash_volt.set_val(vitals.get('voltage', 0))
            
            # --- Health Matrix 状态更新逻辑 ---
            
            # 1. COMM (通信): 只要收到 vitals 数据就亮绿灯
            self.health_matrix.set_status("COMM", 1)
            
            # 2. TEMP (温度): >80度红灯，否则绿灯
            temp = vitals.get('max_temp', 0)
            self.health_matrix.set_status("TEMP", 2 if temp > 80 else 1)
            
            # 3. LEGS (电源): 电压过低(且非0)红灯，否则绿灯
            volt = vitals.get('voltage', 0)
            self.health_matrix.set_status("LEGS", 2 if (volt > 0 and volt < 40) else 1)

            # 4. [新增] IMU (姿态传感器)
            # 检查四元数是否全为0 (全0代表数据无效)
            quat = vitals.get('quat', [0,0,0,0])
            if quat and sum(map(abs, quat)) > 0.001:
                self.health_matrix.set_status("IMU", 1) # 绿灯
            else:
                self.health_matrix.set_status("IMU", 0) # 灰灯

            # 5. [新增] L.ARM / R.ARM (左右臂)
            # 检查是否收到了对应关节的数据
            # 将所有键转为 int，防止可能是字符串 key
            try:
                joint_keys = [int(k) for k in joints.keys()]
                
                # 左肩 Pitch ID = 15
                left_active = 15 in joint_keys
                self.health_matrix.set_status("L.ARM", 1 if left_active else 0)
                
                # 右肩 Pitch ID = 22
                right_active = 22 in joint_keys
                self.health_matrix.set_status("R.ARM", 1 if right_active else 0)
            except:
                pass

            # Update latency
            import time
            self.comm_widget.update_stats(str(self.args.iface), 5.0, int(time.time()) % 1000)
        else:
            # 如果没有 vitals 数据，说明底层连接断开，全部熄灭或变红
            self.health_matrix.set_status("COMM", 0)
            self.health_matrix.set_status("IMU", 0)
            self.health_matrix.set_status("L.ARM", 0)
            self.health_matrix.set_status("R.ARM", 0)

    # -------------------------------------------------------------------------
    # 4. 图像处理 (Vision Pipeline)
    # -------------------------------------------------------------------------
    def _on_nav_play_action(self, action_name):
        status = self.worker.last_status
        app_state = status.get('app_state', 'IDLE')
        mode = status.get('mode', 'WALKING')

        if app_state != 'RUNNING':
            QMessageBox.warning(self, "错误", "机器人未运行，无法回放动作。")
            return
        if mode not in ['WALKING', 'TEACH_STANDBY']:
            QMessageBox.warning(self, "提示", f"当前模式为 {mode}，请先退出其他模式再回放。")
            return

        self.worker.start_replay(action_name)

    def _on_live_detect_toggled(self, active):
        self.is_live_detect_active = active

    def process_and_display_frame(self, rgb_img):
        if rgb_img is None:
            # 可选：self.mj.set_overlay_text("VIDEO DISABLED")
            return
        """Worker 传来的图像 -> YOLO (可选) -> MuJoCo 显示"""
        final_img = rgb_img

        if self.is_live_detect_active:
            if self.detector is None:
                if ObjectDetector: self.detector = ObjectDetector()
                else: 
                    self.is_live_detect_active = False # 自动关闭
                    return

            try:
                bgr_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)
                detections = self.detector.detect(bgr_img, conf_thres=0.5)
                annotated_bgr = self.detector.draw_results(bgr_img, detections)
                final_img = cv2.cvtColor(annotated_bgr, cv2.COLOR_BGR2RGB)
            except Exception as e:
                print(f"[Vision Error] {e}")

        self.mj.update_frame(final_img)

    def _perform_auto_grasp_check(self):
        """响应 PrecisionTab 的抓取请求"""
        # 1. 获取当前图像
        rgb_img = self.mj.image
        if rgb_img is None or rgb_img.isNull():
            QMessageBox.warning(self, "错误", "无摄像头图像")
            return
        
        # 转换 QImage -> numpy
        w, h = rgb_img.width(), rgb_img.height()
        ptr = rgb_img.bits()
        ptr.setsize(rgb_img.byteCount())
        arr = np.array(ptr).reshape(h, w, 3) # 假设 RGB888
        
        # 2. 运行检测
        if self.detector is None: 
            if ObjectDetector: self.detector = ObjectDetector()
            else: return

        bgr_img = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        detections = self.detector.detect(bgr_img)
        
        # 3. 寻找最佳目标 (Bottle / Cup)
        best_target = None
        candidates = [d for d in detections if d['class_name'] in ['bottle', 'cup']]
        if candidates: best_target = max(candidates, key=lambda x: x['conf'])
        elif detections: best_target = max(detections, key=lambda x: x['conf'])

        if best_target:
            u, v = best_target['center']
            # 修正抓取点
            if best_target['class_name'] == 'bottle':
                x1, y1, x2, y2 = best_target['box']
                v = int(y1 + (y2 - y1) * 0.45)

            self.log_msg(f"[Vision] 锁定目标: {best_target['class_name']} @ ({u}, {v})")
            # 发送点击信号给后端计算 3D 坐标
            self.worker.send_vision_click(u, v)
            
            # 延迟弹出确认框 (等待后端计算完 vision_pos)
            QTimer.singleShot(300, self.tab_precision.open_grasp_confirm_dialog)
        else:
            QMessageBox.information(self, "提示", "未检测到目标。")

    # -------------------------------------------------------------------------
    # 5. 语音处理
    # -------------------------------------------------------------------------
    def _handle_voice_input(self, text):
        # 启动线程处理
        self.voice_thread = VoiceWorkThread(self.voice_brain, text)
        self.voice_thread.result_signal.connect(self._on_voice_reply)
        self.voice_thread.start()

    def _on_voice_reply(self, reply_text, action):
        # 1. 更新 UI
        self.tab_voice.append_message("G1", reply_text, role="robot")
        self.tab_voice.reset_input_state()
        
        # 2. 播放语音
        if self.audio_client:
            try:
                self.audio_client.SetVolume(100)
                self.audio_client.TtsMaker(reply_text, 0)
            except Exception as e: self.log_msg(f"TTS Error: {e}")

        # 3. 执行动作
        if action:
            self.tab_voice.append_message("System", f"执行指令: {action}", role="system")
            action = action.upper()
            if action == "WAVE": self.worker.cmd_queue.put(('CMD_HAND_ACT', {'action': 'OPEN'}))
            elif action == "SQUAT": self.worker.trigger_start_squat()

    # -------------------------------------------------------------------------
    # 6. 通用工具 (Startup/Stop Dialogs)
    # -------------------------------------------------------------------------
    def on_start_btn_triggered(self):
        safe_ids = [200, 500, 801]
        if self.current_fsm_id in safe_ids:
            self.log_msg(f"检测到机器人已在运行状态 (FSM {self.current_fsm_id})，跳过启动流程。")
            self.worker.trigger_force_running()
            return

        choice = self._show_custom_selection_dialog(
            "选择启动方式", "请根据当前环境选择机器人的启动模式：",
            "悬挂启动 (Hanger)\n适用于: 调试架/吊装", "#e65100",
            "蹲姿启动 (Squat)\n适用于: 地面/平地", "#2e7d32"
        )
        if choice == 1: self._confirm_action("悬挂启动确认", "请确保已挂好安全绳并调整高度。", self.worker.trigger_start_hanger)
        elif choice == 2: self._confirm_action("蹲姿启动确认", "请确保机器人处于蹲姿且无障碍物。", self.worker.trigger_start_squat)

    def on_stop_btn_triggered(self):
        choice = self._show_custom_selection_dialog(
            "选择关机方式", "⚠️ 警告：关机操作将切断运控力矩",
            "悬挂关机 (Damp)\n直接阻尼 / 需挂绳", "#b71c1c",
            "蹲姿关机 (Squat)\n缓慢趴下 / 地面用", "#1565c0"
        )
        if choice == 1: self._confirm_action("悬挂关机确认", "⚠️ 机器人将直接泄力！确认已挂绳？", self.worker.trigger_stop_hanger)
        elif choice == 2: self._confirm_action("蹲姿关机确认", "机器人将执行趴下动作。", self.worker.trigger_stop_squat)

    def _confirm_action(self, title, msg, func):
        reply = QMessageBox.information(self, title, msg + "\n\n点击 Yes 执行。", QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes: func()

    def _show_custom_selection_dialog(self, title, desc, opt1_text, opt1_color, opt2_text, opt2_color):
        dlg = QDialog(self); dlg.setWindowTitle(title); dlg.setFixedSize(450, 280)
        dlg.setStyleSheet("QDialog { background-color: #222; border: 1px solid #444; } QLabel { color: #eee; }")
        l = QVBoxLayout(dlg); l.setSpacing(15); l.setContentsMargins(20,20,20,20)
        
        lbl = QLabel(desc); lbl.setStyleSheet("font-size: 14px; font-weight: bold; color: #ccc;"); lbl.setWordWrap(True); lbl.setAlignment(Qt.AlignCenter)
        l.addWidget(lbl)

        btn_style = "QPushButton {{ background-color: {color}; color: white; border-radius: 8px; font-size: 16px; font-weight: bold; padding: 10px; border: 1px solid #aaa; }} QPushButton:hover {{ border: 2px solid white; }}"
        
        b1 = QPushButton(opt1_text); b1.setFixedHeight(70); b1.setStyleSheet(btn_style.format(color=opt1_color)); b1.clicked.connect(lambda: dlg.done(1))
        b2 = QPushButton(opt2_text); b2.setFixedHeight(70); b2.setStyleSheet(btn_style.format(color=opt2_color)); b2.clicked.connect(lambda: dlg.done(2))
        bc = QPushButton("取消"); bc.setStyleSheet("background-color: #444; color: #aaa; border: none; padding: 5px;"); bc.clicked.connect(lambda: dlg.done(0))
        
        l.addWidget(b1); l.addWidget(b2); l.addWidget(bc)
        return dlg.exec_()

    def center_window(self):
        qr = self.frameGeometry(); cp = QDesktopWidget().availableGeometry().center(); qr.moveCenter(cp); self.move(qr.topLeft())

    def log_msg(self, t):
        self.log.append(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] {t}")
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def keyPressEvent(self, e): self.worker.handle_key_event(e.key(), True)
    def keyReleaseEvent(self, e): self.worker.handle_key_event(e.key(), False)
    def closeEvent(self, e):
        state = self.worker.last_status.get('app_state', 'IDLE')
        if state != 'IDLE': 
            reply = QMessageBox.warning(self, "警告", "机器人仍在运行！\n强制退出可能导致机器人摔倒。\n\n是否仍要强制退出？", QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.No: e.ignore(); return
        if self.worker.isRunning(): self.worker.stop()
        import sys; sys.exit(0)
