# ui/tabs/monitor_tab.py
import math
import numpy as np
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QFrame, QGridLayout, QProgressBar)
from PyQt5.QtCore import Qt

# 引入样式表
from ui.styles import Styles

class MonitorTab(QWidget):
    """
    Tab 1: 数据监控面板
    职责：显示机器人基座坐标、手臂IK追踪误差、视觉目标坐标、综合误差条
    """
    def __init__(self):
        super().__init__()
        self.monitor_labels = {} # 用于存储手臂数据的 Label 引用
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # 应用样式
        self.setStyleSheet(Styles.MONITOR_CARD + Styles.MONITOR_PROGRESS_BAR)

        # =============================================================
        # 模块 1: 机器人基座状态 (底盘定位)
        # =============================================================
        frame_odom = QFrame()
        frame_odom.setObjectName("Card")
        lo_odom = QVBoxLayout(frame_odom)
        
        # 标题栏
        header_odom = QHBoxLayout()
        header_odom.addWidget(QLabel("机器人定位状态 (世界坐标系)", objectName="CardTitle"))
        header_odom.addStretch()
        lbl_status_led = QLabel("● 系统在线")
        lbl_status_led.setStyleSheet("color: #00e676; font-size: 10px; font-weight: bold;")
        header_odom.addWidget(lbl_status_led)
        lo_odom.addLayout(header_odom)

        # 数据网格
        grid_odom = QGridLayout()
        grid_odom.setSpacing(10)

        # 创建 X, Y, Yaw 显示
        box_x, self.lbl_odom_x = self._create_stat_block("X 轴坐标 (米)", "#d500f9")
        box_y, self.lbl_odom_y = self._create_stat_block("Y 轴坐标 (米)", "#d500f9")
        box_yaw, self.lbl_odom_yaw = self._create_stat_block("航向角 / Yaw (度)", "#ffffff")

        grid_odom.addWidget(box_x, 0, 0)
        grid_odom.addWidget(self._create_v_line(), 0, 1)
        grid_odom.addWidget(box_y, 0, 2)
        grid_odom.addWidget(self._create_v_line(), 0, 3)
        grid_odom.addWidget(box_yaw, 0, 4)

        lo_odom.addLayout(grid_odom)
        layout.addWidget(frame_odom)

        # =============================================================
        # 模块 2: 机械臂追踪状态 (目标 vs 实际)
        # =============================================================
        frame_arm = QFrame()
        frame_arm.setObjectName("Card")
        lo_arm = QVBoxLayout(frame_arm)
        lo_arm.addWidget(QLabel("末端执行器追踪数据 (基座相对系)", objectName="CardTitle"))

        grid_arm = QGridLayout()
        grid_arm.setVerticalSpacing(15)
        grid_arm.setHorizontalSpacing(20)

        # 表头
        headers = ["轴向", "目标设定值 (Goal)", "实际反馈值 (Real)", "偏差 (Error)"]
        header_colors = ["#888", "#00e5ff", "#ffeb3b", "#ff5252"] 
        
        for c, (txt, col) in enumerate(zip(headers, header_colors)):
            l = QLabel(txt)
            l.setStyleSheet(f"color: {col}; font-weight: bold; font-size: 11px; border-bottom: 2px solid {col}; padding-bottom: 5px; font-family: 'Microsoft YaHei UI';")
            l.setAlignment(Qt.AlignCenter if c > 0 else Qt.AlignLeft)
            grid_arm.addWidget(l, 0, c)

        # 数据行 (X, Y, Z)
        axes = ["X", "Y", "Z"]
        for r, axis in enumerate(axes):
            row_idx = r + 1
            
            # 轴标签
            lbl_axis = QLabel(axis)
            lbl_axis.setStyleSheet("color: #aaa; font-weight: bold; font-size: 16px; background: #252525; border-radius: 4px; padding: 2px 8px;")
            lbl_axis.setFixedWidth(40)
            lbl_axis.setAlignment(Qt.AlignCenter)
            grid_arm.addWidget(lbl_axis, row_idx, 0)

            # 目标值
            lbl_goal = QLabel("0.000"); lbl_goal.setObjectName("ValNorm")
            lbl_goal.setStyleSheet("color: #00e5ff;")
            lbl_goal.setAlignment(Qt.AlignCenter)
            grid_arm.addWidget(lbl_goal, row_idx, 1)

            # 实际值
            lbl_hand = QLabel("0.000"); lbl_hand.setObjectName("ValNorm")
            lbl_hand.setStyleSheet("color: #ffeb3b;")
            lbl_hand.setAlignment(Qt.AlignCenter)
            grid_arm.addWidget(lbl_hand, row_idx, 2)

            # 偏差值
            lbl_diff = QLabel("0.000"); lbl_diff.setObjectName("ValNorm")
            lbl_diff.setStyleSheet("color: #666;") 
            lbl_diff.setAlignment(Qt.AlignCenter)
            grid_arm.addWidget(lbl_diff, row_idx, 3)

            # 存入字典: gX, hX, dX...
            self.monitor_labels[f"g{axis}"] = lbl_goal
            self.monitor_labels[f"h{axis}"] = lbl_hand
            self.monitor_labels[f"d{axis}"] = lbl_diff

            # 行间分割线
            if r < 2:
                line = QFrame()
                line.setFrameShape(QFrame.HLine)
                line.setStyleSheet("color: #222; margin-top: 5px;")
                grid_arm.addWidget(line, row_idx, 0, 1, 4, alignment=Qt.AlignBottom)

        lo_arm.addLayout(grid_arm)
        layout.addWidget(frame_arm)

        # =============================================================
        # 模块 3: 视觉锁定目标 (Visual Target)
        # =============================================================
        frame_vis = QFrame()
        frame_vis.setObjectName("Card")
        lo_vis = QVBoxLayout(frame_vis)
        
        header_vis = QHBoxLayout()
        header_vis.addWidget(QLabel("视觉锁定坐标 (Visual Target)", objectName="CardTitle"))
        header_vis.addStretch()
        lbl_cam_icon = QLabel("📷 D435i")
        lbl_cam_icon.setStyleSheet("color: #ff9800; font-size: 10px; font-weight: bold;")
        header_vis.addWidget(lbl_cam_icon)
        lo_vis.addLayout(header_vis)

        grid_vis = QGridLayout()
        grid_vis.setSpacing(10)

        vb_x, self.lbl_vis_x = self._create_stat_block("视觉 X (前)", "#ff9800")
        vb_y, self.lbl_vis_y = self._create_stat_block("视觉 Y (左)", "#ff9800")
        vb_z, self.lbl_vis_z = self._create_stat_block("视觉 Z (上)", "#ff9800")

        grid_vis.addWidget(vb_x, 0, 0)
        grid_vis.addWidget(self._create_v_line(), 0, 1)
        grid_vis.addWidget(vb_y, 0, 2)
        grid_vis.addWidget(self._create_v_line(), 0, 3)
        grid_vis.addWidget(vb_z, 0, 4)

        lo_vis.addLayout(grid_vis)
        layout.addWidget(frame_vis)

        # =============================================================
        # 模块 4: 综合误差条
        # =============================================================
        frame_err = QFrame()
        frame_err.setStyleSheet("background: transparent; border: none;")
        lo_err = QHBoxLayout(frame_err)
        lo_err.setContentsMargins(0, 5, 0, 0)

        lo_err.addWidget(QLabel("综合追踪精度:", styleSheet="color: #bbb; font-size: 11px; font-weight: bold;"))
        
        self.bar_err = QProgressBar()
        self.bar_err.setRange(0, 100) # 0 ~ 10cm
        self.bar_err.setTextVisible(False)
        self.bar_err.setFixedHeight(8)
        lo_err.addWidget(self.bar_err)
        
        self.lbl_total_err = QLabel("0.0 mm")
        self.lbl_total_err.setStyleSheet("color: #aaa; font-family: Consolas; font-weight: bold;")
        self.lbl_total_err.setFixedWidth(80)
        self.lbl_total_err.setAlignment(Qt.AlignRight)
        lo_err.addWidget(self.lbl_total_err)

        layout.addWidget(frame_err)
        layout.addStretch()

    # --- 辅助方法 ---
    def _create_stat_block(self, label, color):
        w = QWidget()
        v_layout = QVBoxLayout(w)
        v_layout.setContentsMargins(0,0,0,0)
        v_layout.setSpacing(2)
        
        lbl_title = QLabel(label)
        lbl_title.setStyleSheet("color: #aaa; font-size: 11px;")
        lbl_val = QLabel("0.00")
        lbl_val.setObjectName("ValBig")
        lbl_val.setStyleSheet(f"color: {color};")
        
        v_layout.addWidget(lbl_title)
        v_layout.addWidget(lbl_val)
        return w, lbl_val

    def _create_v_line(self):
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setStyleSheet("color: #333;")
        return line

    # =============================================================
    # 数据更新接口 (核心逻辑)
    # =============================================================
    def update_data(self, status_data):
        """
        接收来自主窗口的全量状态包，解析并更新界面
        """
        if not status_data: return

        # 1. 更新机器人基座 (Odom)
        robot_x, robot_y, robot_yaw = 0.0, 0.0, 0.0
        
        # 优先使用导航定位数据，如果没有则使用仿真基座数据
        nav_data = status_data.get('nav_data', {})
        if nav_data and 'robot_pose' in nav_data and nav_data['robot_pose'] != [0,0,0]:
            robot_x, robot_y, robot_yaw = nav_data['robot_pose']
        else:
            base_pos = status_data.get('robot_base_pos', np.array([0., 0., 0.]))
            base_yaw = status_data.get('robot_base_yaw', 0.0)
            robot_x, robot_y = base_pos[0], base_pos[1]
            robot_yaw = base_yaw
        
        self.lbl_odom_x.setText(f"{robot_x:.3f}")
        self.lbl_odom_y.setText(f"{robot_y:.3f}")
        self.lbl_odom_yaw.setText(f"{math.degrees(robot_yaw):.1f}°")

        # 2. 更新手臂追踪 (Arm Tracking)
        p_goal_world = np.array(status_data.get('p_goal', [0., 0., 0.]))
        p_hand_world = np.array(status_data.get('p_hand', [0., 0., 0.]))
        
        # 计算相对坐标 (相对于基座)
        base_pos_arr = np.array([robot_x, robot_y, 0.0]) # 简化 Z
        rel_goal = p_goal_world - base_pos_arr
        rel_hand = p_hand_world - base_pos_arr
        
        axes = ['X', 'Y', 'Z']
        for i, ax in enumerate(axes):
            # 注意：这里的数据可能只是示意，如果需要精确的相对基座变换，需后端支持
            # 这里沿用原逻辑：直接做减法显示
            g_val = rel_goal[i]
            h_val = rel_hand[i]
            d_val = g_val - h_val
            
            if f'g{ax}' in self.monitor_labels:
                self.monitor_labels[f'g{ax}'].setText(f"{g_val:.3f}")
                self.monitor_labels[f'h{ax}'].setText(f"{h_val:.3f}")
                
                lbl_diff = self.monitor_labels[f'd{ax}']
                lbl_diff.setText(f"{d_val:+.3f}")
                
                # 动态颜色
                if abs(d_val) < 0.01: lbl_diff.setStyleSheet("color: #444;") # 极小误差变暗
                elif abs(d_val) > 0.1: lbl_diff.setStyleSheet("color: #ff5252;") # 大误差变红
                else: lbl_diff.setStyleSheet("color: #bbb;") 
        
        # 3. 更新误差条
        diff_vec = p_goal_world - p_hand_world
        total_err = np.linalg.norm(diff_vec)
        err_mm = total_err * 1000
        self.bar_err.setValue(int(min(100, err_mm)))
        self.lbl_total_err.setText(f"{err_mm:.1f} mm")

        # 4. 更新视觉坐标
        vis_pos = status_data.get('vision_pos')
        if vis_pos is not None:
            self.lbl_vis_x.setText(f"{vis_pos[0]:.3f}")
            self.lbl_vis_y.setText(f"{vis_pos[1]:.3f}")
            self.lbl_vis_z.setText(f"{vis_pos[2]:.3f}")
        else:
            self.lbl_vis_x.setText("N/A")
            self.lbl_vis_y.setText("N/A")
            self.lbl_vis_z.setText("N/A")
