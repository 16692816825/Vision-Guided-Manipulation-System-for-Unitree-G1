#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
检测框可视化节点 (tkinter)
==========================
订阅相机图像 + 检测结果，绘制检测框：
  - 左侧：原始相机图像
  - 右侧：YOLO 检测标注图像
  - 同时发布标注图像到 ROS 话题（兼容 rviz2/rqt）

相比 cv2.imshow 方案，tkinter 无需 DISPLAY 环境变量，SSH 下也能正常显示。

运行：
    ros2 run g1_yolo_nav_py detection_visualizer

依赖：
    pip install pillow
"""

import threading
import time
from typing import Optional

import tkinter as tk
import numpy as np
import cv2
from PIL import Image as PILImage, ImageTk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

from g1_yolo_nav_py._vis_utils import draw_detections_on_frame, cv2_to_tk  # 共享绘制和转换

# 设计系统 — 现代清新浅色 (Modern Light Teal)
_BG_DEEP       = "#F0FDFA"
_BG_PRIMARY     = "#FFFFFF"
_BG_SECONDARY   = "#0891B2"
_BG_TERTIARY    = "#E0F2FE"
_BORDER_COLOR   = "#D1E7E4"
_ACCENT_CYAN    = "#0891B2"
_ACCENT_GREEN   = "#10B981"
_ACCENT_ORANGE  = "#F59E0B"
_ACCENT_RED     = "#EF4444"
_ACCENT_BLUE    = "#3B82F6"
_TEXT_PRIMARY    = "#134E4A"
_TEXT_SECONDARY  = "#5F7A78"
_TEXT_MUTED      = "#94A3B8"

class DetectionVisualizerNode(Node):
    """订阅图像和检测结果，叠加检测框后 tkinter 显示 + 话题发布。"""

    def __init__(self) -> None:
        super().__init__("g1_detection_visualizer_node")

        self.declare_parameter("image_topic", "/D455_1/color/image_raw")
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("annotated_topic", "/g1/vision/annotated_image")
        self.declare_parameter("display_width", 400)
        self.declare_parameter("display_height", 300)

        image_topic = self.get_parameter("image_topic").value
        det_topic = self.get_parameter("detection_topic").value
        annotated_topic = self.get_parameter("annotated_topic").value
        self._disp_w = int(self.get_parameter("display_width").value)
        self._disp_h = int(self.get_parameter("display_height").value)

        self._bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )

        self.create_subscription(Image, image_topic, self._image_cb, sensor_qos)
        self.create_subscription(Detection2DArray, det_topic, self._detection_cb, 10)

        self._pub = self.create_publisher(Image, annotated_topic, pub_qos)

        self._cv_image: Optional[np.ndarray] = None
        self._detections: Optional[Detection2DArray] = None
        self._pub_count = 0
        self._running = True
        self._fps = 0.0
        self._frame_count = 0
        self._fps_time = time.time()
        self._new_frame = False  # ROS2 回调标记有新帧
        self._last_header = None  # 保存 header 用于发布

        self._build_gui()
        self._update_loop()

        self.get_logger().info(
            f"可视化节点启动: 图像={image_topic}, 检测={det_topic}, "
            f"输出={annotated_topic}"
        )

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("G1 YOLO Detection Visualizer")
        self.root.configure(bg=_BG_DEEP)
        self.root.geometry("880x460")
        self.root.minsize(640, 360)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        title_frame = tk.Frame(self.root, bg=_BG_SECONDARY, height=42)
        title_frame.pack(fill=tk.X)
        title_frame.pack_propagate(False)
        tk.Label(
            title_frame, text="  G1 YOLO 检测可视化",
            font=("Consolas", 13, "bold"), fg="white", bg=_BG_SECONDARY
        ).pack(side=tk.LEFT, padx=(12, 0), pady=8)

        self._fps_label = tk.Label(
            title_frame, text="FPS --", font=("Consolas", 10),
            fg="#CCEDF5", bg=_BG_SECONDARY
        )
        self._fps_label.pack(side=tk.RIGHT, padx=12, pady=8)

        self._det_label = tk.Label(
            title_frame, text="检测 --", font=("Consolas", 10),
            fg="#CCEDF5", bg=_BG_SECONDARY
        )
        self._det_label.pack(side=tk.RIGHT, padx=10, pady=8)

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

        status_frame = tk.Frame(self.root, bg=_BG_PRIMARY, height=28, highlightbackground=_BORDER_COLOR, highlightthickness=1)
        status_frame.pack(fill=tk.X, padx=8, pady=(0, 6))
        status_frame.pack_propagate(False)
        self._status_label = tk.Label(
            status_frame, text="  等待图像...",
            font=("Consolas", 9), fg=_TEXT_SECONDARY, bg=_BG_PRIMARY, anchor="w"
        )
        self._status_label.pack(side=tk.LEFT, padx=8, fill=tk.X, expand=True)

    def _image_cb(self, msg: Image):
        try:
            self._cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._frame_count += 1
            self._new_frame = True
            self._last_header = msg.header
        except Exception as e:
            self.get_logger().warn(f"图像转换失败: {e}")

    def _detection_cb(self, msg: Detection2DArray):
        self._detections = msg

    def _draw_on_frame(self, frame: np.ndarray) -> np.ndarray:
        """在图像上绘制检测框（返回新图像，不修改传入的 frame）。"""
        if self._detections is None:
            return frame.copy()
        return draw_detections_on_frame(frame, self._detections)

    def _cv2_to_tk(self, frame: np.ndarray, canvas: tk.Canvas) -> Optional[ImageTk.PhotoImage]:
        """将 OpenCV 图像转为 tkinter 可显示格式并缩放适配画布。"""
        cw = canvas.winfo_width() or self._disp_w
        ch = canvas.winfo_height() or self._disp_h
        if cw < 2 or ch < 2:
            cw, ch = self._disp_w, self._disp_h
        return cv2_to_tk(frame, cw, ch)

    # 标注图像发布（在主线程中执行，避免阻塞 ROS2 回调）
    def _publish_annotated(self) -> None:
        """绘制检测框并发布标注图像到话题。"""
        if self._cv_image is None or self._last_header is None:
            return
        try:
            frame = self._cv_image.copy()
            frame = self._draw_on_frame(frame)
            annotated_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            annotated_msg.header = self._last_header
            self._pub.publish(annotated_msg)
            self._pub_count += 1
        except Exception as e:
            self.get_logger().warn(f"发布标注图像失败: {e}")

    def _update_loop(self):
        """定时刷新 GUI（约 60Hz）。"""
        if not self._running:
            return

        try:
            self._do_update()
        except tk.TclError:
            self._running = False
            return
        except Exception:
            pass

        if self._running:
            self.root.after(16, self._update_loop)

    def _do_update(self):
        now = time.time()
        elapsed = now - self._fps_time
        if elapsed >= 1.0:
            self._fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_time = now
            self._fps_label.config(text=f"FPS: {self._fps:.0f}")

        if not self._new_frame:
            return
        self._new_frame = False

        if self._cv_image is not None:
            frame_copy = self._cv_image.copy()

            self._raw_photo = self._cv2_to_tk(frame_copy, self._raw_canvas)
            if self._raw_photo:
                self._raw_canvas.delete("all")
                self._raw_canvas.create_image(0, 0, anchor=tk.NW, image=self._raw_photo)

            det_frame = self._draw_on_frame(frame_copy)
            self._det_photo = self._cv2_to_tk(det_frame, self._det_canvas)
            if self._det_photo:
                self._det_canvas.delete("all")
                self._det_canvas.create_image(0, 0, anchor=tk.NW, image=self._det_photo)

            # 发布标注图像（移到主线程，避免阻塞 ROS2 回调）
            self._publish_annotated()

            det_count = len(self._detections.detections) if self._detections else 0
            if det_count > 0:
                best = max(
                    self._detections.detections,
                    key=lambda d: d.results[0].score if d.results else 0
                )
                if best.results:
                    self._det_label.config(
                        text=f"检测: {best.results[0].id} ({best.results[0].score:.0%}) x{det_count}"
                    )
            else:
                self._det_label.config(text="检测: 无目标")

            self._status_label.config(
                text=f"  已发布 {self._pub_count} 帧 | {self._cv_image.shape[1]}x{self._cv_image.shape[0]}",
                fg=_ACCENT_CYAN
            )
        else:
            self._status_label.config(text="  等待图像...", fg=_TEXT_MUTED)

    def _on_close(self):
        self._running = False
        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self):
        self.root.mainloop()

    def destroy_node(self) -> None:
        self.get_logger().info("[清理] 可视化节点已停止")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DetectionVisualizerNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()  # tkinter 主循环（阻塞）
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()