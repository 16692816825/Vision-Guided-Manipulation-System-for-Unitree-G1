#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 NavGrasp 控制面板 (tkinter)
================================
基于 tkinter 的前端控制界面，集成 grasp_task 全部能力：
  - 左侧：原始相机图像
  - 右侧：YOLO 检测标注图像
  - 下方：状态信息 + 操作按钮 + 一键抓取任务

状态机（共享自 _grasp_state.py）：
    IDLE        → 空闲等待
    WORKING     → 搜索 + 步进式对齐 + 接近（连续行为，无状态切换）
    GRABBING    → 执行 armup.py 抓取
    MENU        → 交互菜单（放下/右转放下）

运行：
    ros2 run g1_yolo_nav_py control_panel
    python3 -m g1_yolo_nav_py.control_panel

依赖：
    pip install pillow  (PIL 用于图像缩放显示)
"""

import sys
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

import tkinter as tk
import numpy as np
import cv2
from PIL import ImageTk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

# 注意：control_panel 不导入 unitree_sdk2py！
# unitree_sdk2py 的模块级导入会加载 CycloneDDS 绑定，干扰 ROS2 的 CycloneDDS domain，
# 导致 ROS2 订阅收不到任何数据（原始图像加载不出来）。
# 所有运动控制通过 SportClient 统一封装（Loco API），无需 LocoClient / DDS。

from g1_yolo_nav_py._grasp_state import GraspStateMachineMixin, GraspState
from g1_yolo_nav_py._vis_utils import draw_detections_on_frame, cv2_to_tk

_STATE_LABELS = {
    GraspState.IDLE: "空闲",
    GraspState.WORKING: "执行中",
    GraspState.GRABBING: "抓取中",
    GraspState.MENU: "操作菜单",
}

# 设计系统 — 现代清新浅色 (Modern Light Teal)
_BG_DEEP       = "#F0FDFA"   # 窗口底色 (Teal-50)
_BG_PRIMARY     = "#FFFFFF"   # 卡片背景 (White)
_BG_SECONDARY   = "#0891B2"   # 标题栏 (Teal-600 渐变起点)
_BG_TERTIARY    = "#E0F2FE"   # 次级区域 (Sky-100)
_BORDER_COLOR   = "#D1E7E4"   # 边框 (Teal-muted)
_BORDER_ACTIVE  = "#0891B2"   # 激活边框 (Teal-600)

_ACCENT_CYAN    = "#0891B2"   # 主强调色 (Teal-600 信息/对齐)
_ACCENT_GREEN   = "#10B981"   # 正向/成功/CTA (Emerald-500)
_ACCENT_ORANGE  = "#F59E0B"   # 搜索/警告 (Amber-500)
_ACCENT_RED     = "#EF4444"   # 危险/停止/抓取 (Red-500)
_ACCENT_BLUE    = "#3B82F6"   # 前进/导航 (Blue-500)

_TEXT_PRIMARY    = "#134E4A"   # 主文字 (Teal-900 对比 8.8:1)
_TEXT_SECONDARY  = "#5F7A78"   # 次级文字
_TEXT_MUTED      = "#94A3B8"   # 弱化文字 (Slate-400)

_STATE_COLORS = {
    GraspState.IDLE:     _TEXT_SECONDARY,
    GraspState.WORKING:  _ACCENT_CYAN,
    GraspState.GRABBING: _ACCENT_RED,
    GraspState.MENU:     _ACCENT_GREEN,
}

class ControlPanelNode(Node, GraspStateMachineMixin):
    """G1 控制面板节点 — tkinter GUI + ROS2 通信 + 抓取任务状态机。"""

    def __init__(self) -> None:
        Node.__init__(self, "g1_control_panel_node")

        self.declare_parameter("image_topic", "/D455_1/color/image_raw")
        self.declare_parameter("network_interface", "")  # 传递给 arm 脚本子进程
        self.declare_parameter("display_width", 400)
        self.declare_parameter("display_height", 300)

        p = lambda n: self.get_parameter(n).value
        self._img_topic = p("image_topic")
        self._net_iface = p("network_interface")
        self._disp_w = int(p("display_width"))
        self._disp_h = int(p("display_height"))

        self._init_grasp_state(
            self,
            include_idle=True,
            start_state=GraspState.IDLE,
            network_interface=self._net_iface,
        )

        self._bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )

        self._img_sub = self.create_subscription(
            Image, self._img_topic, self._image_cb, sensor_qos
        )

        self._diag_done = False
        self._diag_timer = self.create_timer(2.0, self._diag_check)

        self._raw_image: Optional[np.ndarray] = None
        self._detections: Optional[Detection2DArray] = None
        self._det_count = 0
        self._fps = 0.0
        self._frame_count = 0
        self._fps_time = time.time()
        self._running = True
        self._first_image_received = False
        self._new_frame = False  # ROS2 回调标记有新帧
        self._manual_vx = 0.0   # 手动控制速度（持续发布）
        self._manual_vy = 0.0
        self._manual_vyaw = 0.0   # 偏航角速度，对应 SportClient.move(vyaw=...)
        self._manual_active = False

        self._build_gui()

        self._update_loop()

        self.get_logger().info("=" * 50)
        self.get_logger().info("G1 NavGrasp 控制面板启动")
        self.get_logger().info(f"目标类别: {self._gs_target_class}")
        self.get_logger().info(f"图像: {self._img_topic}, 检测: {self._gs_det_topic}")
        self.get_logger().info(f"armup: {self._gs_armup_script}")
        self.get_logger().info(f"armdown: {self._gs_armdown_script}")
        self.get_logger().info("=" * 50)

    #  日志实现（GraspStateMachineMixin 抽象方法）
    def _log_info(self, msg: str) -> None:
        self._append_log(msg)

    def _log_error(self, msg: str) -> None:
        self._append_log(msg)

    def _on_state_changed(self, old_state: GraspState, new_state: GraspState) -> None:
        """状态变化时更新 GUI。"""
        self._update_state_display()

    def _diag_check(self):
        """启动后延迟检查订阅状态，帮助定位无法接收图像的问题。"""
        if self._diag_done:
            return
        self._diag_done = True
        if hasattr(self, '_diag_timer') and self._diag_timer is not None:
            self._diag_timer.cancel()
        img_pub_count = self.count_publishers(self._img_topic)
        det_pub_count = self.count_publishers(self._gs_det_topic)
        self.get_logger().info(
            f"[诊断] 图像话题 '{self._img_topic}': 发布者数={img_pub_count}"
        )
        self.get_logger().info(
            f"[诊断] 检测话题 '{self._gs_det_topic}': 发布者数={det_pub_count}"
        )
        if img_pub_count == 0:
            self.get_logger().warn(
                f"[诊断] 图像话题无发布者! "
                f"请检查: 1) 相机驱动是否启动 2) 话题名是否正确 "
                f"(用 ros2 topic list 查看)"
            )
            try:
                result = subprocess.run(
                    ["ros2", "topic", "list"],
                    capture_output=True, text=True, timeout=3
                )
                img_topics = [
                    l for l in result.stdout.splitlines()
                    if "image" in l.lower() or "color" in l.lower()
                ]
                if img_topics:
                    self.get_logger().warn(
                        f"[诊断] 发现可能的图像话题: {img_topics}"
                    )
            except Exception:
                pass

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("G1 NavGrasp 控制面板")
        self.root.configure(bg=_BG_DEEP)
        self.root.geometry("920x660")
        self.root.minsize(720, 520)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        title_frame = tk.Frame(self.root, bg=_BG_SECONDARY, height=42)
        title_frame.pack(fill=tk.X, padx=0, pady=0)
        title_frame.pack_propagate(False)

        tk.Label(
            title_frame, text="  G1 NavGrasp",
            font=("Consolas", 14, "bold"), fg="white", bg=_BG_SECONDARY
        ).pack(side=tk.LEFT, padx=(12, 0), pady=8)

        state_container = tk.Frame(title_frame, bg=_BG_SECONDARY)
        state_container.pack(side=tk.LEFT, padx=(16, 0), pady=8)

        self._state_dot = tk.Canvas(
            state_container, width=10, height=10,
            bg=_BG_SECONDARY, highlightthickness=0
        )
        self._state_dot.pack(side=tk.LEFT, padx=(0, 6))
        self._state_dot_dot = self._state_dot.create_oval(1, 1, 9, 9, fill=_TEXT_SECONDARY, outline="")

        self._state_label = tk.Label(
            state_container, text="空闲",
            font=("Consolas", 11, "bold"), fg="white", bg=_BG_SECONDARY
        )
        self._state_label.pack(side=tk.LEFT)

        self._fps_label = tk.Label(
            title_frame, text="FPS --", font=("Consolas", 10),
            fg="#CCEDF5", bg=_BG_SECONDARY
        )
        self._fps_label.pack(side=tk.RIGHT, padx=12, pady=8)

        img_frame = tk.Frame(self.root, bg=_BG_DEEP)
        img_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=(6, 4))

        left_card = tk.Frame(img_frame, bg=_BG_PRIMARY, highlightbackground=_BORDER_COLOR, highlightthickness=1)
        left_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 3))

        left_header = tk.Frame(left_card, bg=_BG_TERTIARY, height=28)
        left_header.pack(fill=tk.X, padx=0, pady=0)
        left_header.pack_propagate(False)
        tk.Label(
            left_header, text="  原始图像", font=("Consolas", 10),
            fg=_TEXT_PRIMARY, bg=_BG_TERTIARY, anchor="w"
        ).pack(side=tk.LEFT, padx=8, fill=tk.X, expand=True)

        self._raw_canvas = tk.Canvas(
            left_card, width=self._disp_w, height=self._disp_h,
            bg="#000000", highlightthickness=0
        )
        self._raw_canvas.pack(fill=tk.BOTH, expand=True, padx=1, pady=(0, 1))

        right_card = tk.Frame(img_frame, bg=_BG_PRIMARY, highlightbackground=_BORDER_COLOR, highlightthickness=1)
        right_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(3, 0))

        right_header = tk.Frame(right_card, bg=_BG_TERTIARY, height=28)
        right_header.pack(fill=tk.X, padx=0, pady=0)
        right_header.pack_propagate(False)
        tk.Label(
            right_header, text="  检测结果", font=("Consolas", 10),
            fg=_TEXT_PRIMARY, bg=_BG_TERTIARY, anchor="w"
        ).pack(side=tk.LEFT, padx=8, fill=tk.X, expand=True)

        self._det_canvas = tk.Canvas(
            right_card, width=self._disp_w, height=self._disp_h,
            bg="#000000", highlightthickness=0
        )
        self._det_canvas.pack(fill=tk.BOTH, expand=True, padx=1, pady=(0, 1))

        info_frame = tk.Frame(self.root, bg=_BG_PRIMARY, height=30, highlightbackground=_BORDER_COLOR, highlightthickness=1)
        info_frame.pack(fill=tk.X, padx=8, pady=(0, 4))
        info_frame.pack_propagate(False)
        self._status_label = tk.Label(
            info_frame, text="  等待图像...",
            font=("Consolas", 10), fg=_TEXT_SECONDARY, bg=_BG_PRIMARY, anchor="w"
        )
        self._status_label.pack(side=tk.LEFT, padx=8, fill=tk.X, expand=True)

        self._det_info_label = tk.Label(
            info_frame, text="检测 --",
            font=("Consolas", 10), fg=_ACCENT_ORANGE, bg=_BG_PRIMARY
        )
        self._det_info_label.pack(side=tk.RIGHT, padx=10)

        btn_frame = tk.Frame(self.root, bg=_BG_DEEP)
        btn_frame.pack(fill=tk.X, padx=8, pady=(0, 4))

        btn_common = {"font": ("Consolas", 10), "width": 8, "height": 1, "bd": 0, "relief": "flat",
                      "cursor": "hand2"}

        self._btn_forward = tk.Button(
            btn_frame, text="  前进", bg=_ACCENT_BLUE, fg="white",
            activebackground="#2563EB", command=lambda: self._manual_cmd(0.2, 0.0, 0.0),
            **btn_common
        )
        self._btn_forward.pack(side=tk.LEFT, padx=2)

        self._btn_backward = tk.Button(
            btn_frame, text="  后退", bg=_ACCENT_BLUE, fg="white",
            activebackground="#2563EB", command=lambda: self._manual_cmd(-0.2, 0.0, 0.0),
            **btn_common
        )
        self._btn_backward.pack(side=tk.LEFT, padx=2)

        self._btn_left = tk.Button(
            btn_frame, text="  左转", bg=_ACCENT_BLUE, fg="white",
            activebackground="#2563EB", command=lambda: self._manual_cmd(0.0, 0.0, 0.4),
            **btn_common
        )
        self._btn_left.pack(side=tk.LEFT, padx=2)

        self._btn_right = tk.Button(
            btn_frame, text="  右转", bg=_ACCENT_BLUE, fg="white",
            activebackground="#2563EB", command=lambda: self._manual_cmd(0.0, 0.0, -0.4),
            **btn_common
        )
        self._btn_right.pack(side=tk.LEFT, padx=2)

        self._btn_stop = tk.Button(
            btn_frame, text="  停止", bg=_ACCENT_RED, fg="white",
            activebackground="#DC2626", command=self._do_stop,
            **btn_common
        )
        self._btn_stop.pack(side=tk.LEFT, padx=2)

        tk.Frame(btn_frame, width=2, bg=_BORDER_COLOR).pack(
            side=tk.LEFT, fill=tk.Y, padx=8, pady=4
        )

        self._btn_search = tk.Button(
            btn_frame, text="  搜索", bg=_ACCENT_ORANGE, fg="white",
            activebackground="#D97706", command=self._do_start_search,
            **btn_common
        )
        self._btn_search.pack(side=tk.LEFT, padx=2)

        self._btn_grab = tk.Button(
            btn_frame, text="  抓取", bg=_ACCENT_RED, fg="white",
            activebackground="#DC2626", command=self._do_grab,
            **btn_common
        )
        self._btn_grab.pack(side=tk.LEFT, padx=2)

        self._btn_putdown = tk.Button(
            btn_frame, text="  放下", bg=_ACCENT_GREEN, fg="white",
            activebackground="#059669", command=self._do_put_down,
            **btn_common
        )
        self._btn_putdown.pack(side=tk.LEFT, padx=2)

        self._btn_turn_putdown = tk.Button(
            btn_frame, text="右转放下", bg=_ACCENT_GREEN, fg="white",
            activebackground="#059669", command=self._do_turn_and_put_down,
            **btn_common
        )
        self._btn_turn_putdown.pack(side=tk.LEFT, padx=2)

        speed_frame = tk.Frame(btn_frame, bg=_BG_DEEP)
        speed_frame.pack(side=tk.RIGHT, padx=6)
        tk.Label(speed_frame, text="速度", font=("Consolas", 9),
                 fg=_TEXT_MUTED, bg=_BG_DEEP).pack(side=tk.LEFT, padx=(0, 4))
        self._speed_var = tk.DoubleVar(value=0.2)
        speed_scale = tk.Scale(
            speed_frame, from_=0.05, to=0.5, resolution=0.05,
            orient=tk.HORIZONTAL, variable=self._speed_var,
            length=90, bg=_BG_DEEP, fg=_TEXT_PRIMARY,
            highlightthickness=0, troughcolor=_BG_TERTIARY,
            activebackground=_ACCENT_CYAN, sliderrelief="flat"
        )
        speed_scale.pack(side=tk.LEFT)

        log_card = tk.Frame(self.root, bg=_BG_PRIMARY, highlightbackground=_BORDER_COLOR, highlightthickness=1)
        log_card.pack(fill=tk.X, padx=8, pady=(0, 6))

        log_header = tk.Frame(log_card, bg=_BG_TERTIARY, height=24)
        log_header.pack(fill=tk.X, padx=0, pady=0)
        log_header.pack_propagate(False)
        tk.Label(
            log_header, text="  日志", font=("Consolas", 9),
            fg=_TEXT_PRIMARY, bg=_BG_TERTIARY, anchor="w"
        ).pack(side=tk.LEFT, padx=8, fill=tk.X, expand=True)

        log_inner = tk.Frame(log_card, bg=_BG_PRIMARY)
        log_inner.pack(fill=tk.BOTH, expand=False, padx=0, pady=0)
        log_inner.pack_propagate(False)
        log_inner.configure(height=72)

        self._log_textbox = tk.Text(
            log_inner, height=4, bg=_BG_TERTIARY, fg="#059669",
            font=("Consolas", 9), bd=0, wrap=tk.WORD,
            state=tk.DISABLED, insertbackground=_ACCENT_GREEN,
            selectbackground=_BG_SECONDARY, selectforeground="white",
            padx=8, pady=4
        )
        scrollbar = tk.Scrollbar(log_inner, command=self._log_textbox.yview,
                                 bg=_BG_PRIMARY, troughcolor=_BG_TERTIARY,
                                 activebackground=_BORDER_ACTIVE)
        self._log_textbox.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 0))
        self._log_textbox.pack(fill=tk.BOTH, expand=True, padx=0, pady=0)

    def _append_log(self, text: str):
        """向日志栏追加一行文本。"""
        def _update():
            self._log_textbox.config(state=tk.NORMAL)
            self._log_textbox.insert(tk.END, text + "\n")
            self._log_textbox.see(tk.END)
            lines = int(self._log_textbox.index("end-1c").split(".")[0])
            if lines > 200:
                self._log_textbox.delete("1.0", f"{lines - 200}.0")
            self._log_textbox.config(state=tk.DISABLED)

        if threading.current_thread() is threading.main_thread():
            _update()
        else:
            self.root.after(0, _update)

    def _update_state_display(self):
        """更新顶部状态显示。"""
        label = _STATE_LABELS.get(self._gs_state, "未知")
        color = _STATE_COLORS.get(self._gs_state, _TEXT_SECONDARY)
        self._state_label.config(text=label, fg=color)
        self._state_dot.itemconfigure(self._state_dot_dot, fill=color)

    def _image_cb(self, msg: Image):
        try:
            self._raw_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._frame_count += 1
            self._new_frame = True
            if not self._first_image_received:
                self._first_image_received = True
                self.get_logger().info(
                    f"[图像] 首次收到图像: {msg.width}x{msg.height}, "
                    f"encoding={msg.encoding}"
                )
        except Exception as e:
            self.get_logger().warn(f"[图像] 转换失败: {e}")

    def _gs_on_detection(self, msg: Detection2DArray) -> None:
        """检测回调 — 存储原始检测结果用于 GUI 绘制，同时转发给基类。"""
        self._detections = msg
        self._det_count = len(msg.detections)
        super()._gs_on_detection(msg)

    def _draw_detections(self, frame: np.ndarray) -> np.ndarray:
        """在图像上绘制检测框（返回新图像，不修改传入的 frame）。"""
        if self._detections is None:
            return frame.copy()
        out = draw_detections_on_frame(frame, self._detections)
        h, w = out.shape[:2]

        if self._gs_target_u is not None and self._gs_state == GraspState.WORKING:
            cx = int(self._gs_target_u * w)
            cy = h // 2
            cv2.line(out, (cx - 15, cy), (cx + 15, cy), (0, 255, 255), 1)
            cv2.line(out, (cx, cy - 15), (cx, cy + 15), (0, 255, 255), 1)

        return out

    def _cv2_to_tk(self, frame: np.ndarray, canvas: tk.Canvas) -> Optional[ImageTk.PhotoImage]:
        """将 OpenCV 图像转为 tkinter 可显示格式并缩放适配画布。"""
        cw = canvas.winfo_width() or self._disp_w
        ch = canvas.winfo_height() or self._disp_h
        if cw < 2 or ch < 2:
            cw, ch = self._disp_w, self._disp_h
        return cv2_to_tk(frame, cw, ch)

    def _manual_cmd(self, vx: float, vy: float, vyaw: float):
        """手动方向控制（持续发布模式，点击后持续运动直到按停止）。"""
        if self._gs_state not in (GraspState.IDLE, GraspState.MENU):
            self._append_log("[提示] 任务执行中，请先停止")
            return
        self._manual_vx = vx
        self._manual_vy = vy
        self._manual_vyaw = vyaw
        self._manual_active = True
        self._append_log(f"[手动] 持续运动: vx={vx}, vyaw={vyaw}（按停止结束）")

    def _do_stop(self):
        """停止所有运动并回到 IDLE。"""
        self._manual_active = False
        self._manual_vx = 0.0
        self._manual_vy = 0.0
        self._manual_vyaw = 0.0

        prev = self._gs_state
        self.gs_state = GraspState.IDLE
        self._sport.stop()
        self._gs_reset_detection()
        self._gs_aligner.reset()
        self._gs_approach.reset()
        self._append_log(f"[状态] {prev.name} → IDLE: 手动停止")
        self._update_state_display()

    def _do_init_fsm(self):
        """手动触发 FSM 初始化。"""
        if self._sport.ready:
            self._append_log("[FSM] 已就绪，无需重复初始化")
            return
        self._append_log("[FSM] 开始初始化状态机...")
        self._sport.init_fsm()

    def _do_start_search(self):
        """开始搜索目标。"""
        if self._gs_state not in (GraspState.IDLE, GraspState.MENU):
            self._append_log("[提示] 请先停止当前任务")
            return
        self.gs_state = GraspState.WORKING
        self._gs_reset_detection()
        self._gs_aligned = False
        self._gs_aligner.reset()
        self._gs_approach.reset()
        self._append_log(f"[状态] → WORKING: 开始搜索 '{self._gs_target_class}'")
        self._update_state_display()

    def _do_grab(self):
        """手动触发抓取（执行 armup.py）。"""
        self._append_log(f"[调试] 抓取按钮: state={self._gs_state.name}, armup={self._gs_armup_script}")
        if self._gs_state not in (GraspState.IDLE, GraspState.MENU):
            self._append_log("[提示] 请先停止当前任务")
            return
        self.gs_state = GraspState.GRABBING
        self._sport.stop()
        self._gs_run_grab()
        self._update_state_display()

    def _do_put_down(self):
        """放下目标物（执行 armdown.py）。"""
        self._append_log(f"[调试] 放下按钮: state={self._gs_state.name}, armdown={self._gs_armdown_script}")
        if self._gs_state not in (GraspState.IDLE, GraspState.MENU):
            self._append_log("[提示] 请先停止当前任务")
            return
        self._gs_run_armdown()

    def _do_turn_and_put_down(self):
        """右转 90° 后放下目标物。"""
        self._append_log(f"[调试] 右转放下按钮: state={self._gs_state.name}")
        if self._gs_state not in (GraspState.IDLE, GraspState.MENU):
            self._append_log("[提示] 请先停止当前任务")
            return
        self._append_log("[右转] 开始右转 90° ...")

        def _worker():
            self._sport.move(vyaw=-self._gs_turn_speed)
            time.sleep(self._gs_turn_duration)
            self._append_log("[右转] 右转完成")
            self._gs_run_armdown()

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

    #  覆盖基类的抓取/放下 — 添加 GUI 状态更新
    def _gs_run_grab(self) -> None:
        """执行 armup.py — 覆盖基类以添加 GUI 更新。"""
        script = str(self._gs_armup_script)
        if not Path(script).exists():
            self._append_log(f"[错误] armup.py 不存在: {script}")
            self.gs_state = GraspState.IDLE
            return

        self._append_log(f"[抓取] 执行 armup.py ... (python={sys.executable}, iface={self._net_iface})")

        def _worker():
            try:
                args = [sys.executable, script]
                if self._net_iface:
                    args.append(self._net_iface)
                self._append_log(f"[抓取] 命令: {' '.join(args)}")
                proc = subprocess.run(
                    args, check=True, input=b"\n",
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    timeout=120,
                )
                if proc.stdout:
                    for line in proc.stdout.decode(errors="replace").splitlines():
                        self._append_log(f"[armup] {line}")
                self._append_log("[抓取] armup.py 完成")
            except subprocess.TimeoutExpired:
                self._append_log("[错误] armup.py 超时（120秒）")
            except subprocess.CalledProcessError as e:
                self._append_log(f"[错误] armup.py 失败: 返回码={e.returncode}")
                if e.stdout:
                    for line in e.stdout.decode(errors="replace").splitlines()[:10]:
                        self._append_log(f"[armup] {line}")
            except Exception as e:
                self._append_log(f"[错误] armup.py 异常: {e}")
            finally:
                self.gs_state = GraspState.MENU
                self._append_log("[状态] → MENU: 抓取完成，可选择放下")
                self.root.after(0, self._update_state_display)

        t = threading.Thread(target=_worker, daemon=True)
        t.start()
        self._append_log(f"[抓取] 子线程已启动 tid={t.ident}")

    def _gs_run_armdown(self) -> None:
        """执行 armdown.py — 覆盖基类以添加 GUI 更新。"""
        script = str(self._gs_armdown_script)
        if not Path(script).exists():
            self._append_log(f"[错误] armdown.py 不存在: {script}")
            return
        self._append_log(f"[放下] 执行 armdown.py ... (python={sys.executable}, iface={self._net_iface})")

        def _worker():
            try:
                args = [sys.executable, script]
                if self._net_iface:
                    args.append(self._net_iface)
                self._append_log(f"[放下] 命令: {' '.join(args)}")
                proc = subprocess.run(
                    args, check=True, input=b"\n",
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    timeout=120,
                )
                if proc.stdout:
                    for line in proc.stdout.decode(errors="replace").splitlines():
                        self._append_log(f"[armdown] {line}")
                self._append_log("[放下] armdown.py 完成")
            except subprocess.TimeoutExpired:
                self._append_log("[错误] armdown.py 超时（120秒）")
            except subprocess.CalledProcessError as e:
                self._append_log(f"[错误] armdown.py 失败: 返回码={e.returncode}")
                if e.stdout:
                    for line in e.stdout.decode(errors="replace").splitlines()[:10]:
                        self._append_log(f"[armdown] {line}")
            except Exception as e:
                self._append_log(f"[错误] armdown.py 异常: {e}")
            finally:
                self.gs_state = GraspState.IDLE
                self._append_log("[状态] → IDLE: 放下完成")
                self.root.after(0, self._update_state_display)

        t = threading.Thread(target=_worker, daemon=True)
        t.start()
        self._append_log(f"[放下] 子线程已启动 tid={t.ident}")

    #  状态机 tick（转发到基类 + 手动控制）
    def _tick(self) -> None:
        # 手动控制持续发布（SET_VELOCITY duration=1.0 需要持续发送保持运动）
        if self._manual_active and self._gs_state in (GraspState.IDLE, GraspState.MENU):
            self._sport.move(vx=self._manual_vx, vy=self._manual_vy, vyaw=self._manual_vyaw)

        self._gs_tick()

    def _update_loop(self):
        """定时刷新 GUI + 状态机 tick（约 60Hz+）。"""
        if not self._running:
            return

        try:
            # 状态机 tick（10Hz，每 100ms 执行一次）
            now = time.time()
            if not hasattr(self, '_last_tick_time'):
                self._last_tick_time = 0.0
            if now - self._last_tick_time >= 0.1:
                self._last_tick_time = now
                self._tick()

            self._do_update()
        except tk.TclError:
            self._running = False
            return
        except Exception as e:
            self.get_logger().warn(f"[GUI] 刷新异常: {e}")

        if self._running:
            self.root.after(16, self._update_loop)

    def _do_update(self):
        """实际刷新逻辑。"""
        now = time.time()
        elapsed = now - self._fps_time
        if elapsed >= 1.0:
            self._fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_time = now
            self._fps_label.config(text=f"FPS: {self._fps:.0f}")

        self._update_status_text()

        if not self._new_frame:
            return
        self._new_frame = False

        if self._raw_image is not None:
            frame_copy = self._raw_image.copy()

            self._raw_photo = self._cv2_to_tk(frame_copy, self._raw_canvas)
            if self._raw_photo:
                self._raw_canvas.delete("all")
                self._raw_canvas.create_image(0, 0, anchor=tk.NW, image=self._raw_photo)

            det_frame = self._draw_detections(frame_copy)
            self._det_photo = self._cv2_to_tk(det_frame, self._det_canvas)
            if self._det_photo:
                self._det_canvas.delete("all")
                self._det_canvas.create_image(0, 0, anchor=tk.NW, image=self._det_photo)

            if self._detections and self._det_count > 0:
                best = max(
                    self._detections.detections,
                    key=lambda d: d.results[0].score if d.results else 0
                )
                if best.results:
                    name = best.results[0].id
                    score = best.results[0].score
                    bbox_info = f"bbox={max(self._gs_bbox_size_x, self._gs_bbox_size_y):.2f}" if self._gs_target_u is not None else ""
                    self._det_info_label.config(
                        text=f"检测: {name} ({score:.0%}) x{self._det_count} {bbox_info}"
                    )
            else:
                self._det_info_label.config(text="检测: 无目标")
        else:
            self._status_label.config(text="状态: 等待图像...", fg="#aaaaaa")

    def _update_status_text(self):
        """更新底部状态栏文字。"""
        if self._raw_image is None:
            self._status_label.config(text="  等待图像...", fg=_TEXT_MUTED)
            return
        if self._gs_state == GraspState.IDLE:
            self._status_label.config(text="  空闲", fg=_TEXT_MUTED)
        elif self._gs_state == GraspState.WORKING:
            if self._gs_target_u is not None:
                err = abs(self._gs_target_u - 0.5)
                bbox_max = max(self._gs_bbox_size_x, self._gs_bbox_size_y)
                self._status_label.config(
                    text=f"  工作中 err={err:.3f} bbox={bbox_max:.2f}/{self._gs_arrive_ratio:.2f}",
                    fg=_ACCENT_CYAN
                )
            else:
                self._status_label.config(text="  搜索目标中...", fg=_ACCENT_ORANGE)
        elif self._gs_state == GraspState.GRABBING:
            self._status_label.config(text="  抓取中...", fg=_ACCENT_RED)
        elif self._gs_state == GraspState.MENU:
            self._status_label.config(text="  抓取完成，可放下", fg=_ACCENT_GREEN)

    def _on_close(self):
        self._running = False
        self._gs_state = GraspState.IDLE
        try:
            self._sport.stop()
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self):
        self.root.mainloop()

    def destroy_node(self) -> None:
        self._gs_destroy()
        super().destroy_node()

def main(args=None):
    """
    控制面板主入口函数

    初始化 ROS2 节点并在后台线程中启动 ROS2 spin，然后运行 tkinter 主循环。
    control_panel 不使用 unitree SDK 的 ChannelFactoryInitialize，
    因此不需要 DDS 兼容层（不调用 _dds_compat.py）。
    _dds_compat 会设置 ROS_DOMAIN_ID=1，导致 ROS2 收不到 Domain 0 的相机数据。

    Args:
        args (list, optional): ROS2 命令行参数，默认为 None
    """
    rclpy.init(args=args)
    node = ControlPanelNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
