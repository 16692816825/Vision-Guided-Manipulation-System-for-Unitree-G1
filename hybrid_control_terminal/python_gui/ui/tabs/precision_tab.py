# ui/tabs/precision_tab.py
import math
import json
import pathlib
import os
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QFrame, QGroupBox, QComboBox, 
                             QTableWidget, QTableWidgetItem, QHeaderView, 
                             QAbstractItemView, QCheckBox, QDialog, 
                             QFormLayout, QLineEdit, QDialogButtonBox, 
                             QFileDialog, QMessageBox)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QColor

from config import ACT_NONE, ACT_GRASP, ACT_OPEN
from ui.styles import Styles
from ui.widgets import SmartLineEdit

class PrecisionTab(QWidget):
    """
    Tab 4: 精确控制与任务序列面板
    职责：IK滑块控制、航点列表编辑(增删改查)、智能抓取参数配置
    """
    # === 对外信号 ===
    sig_ik_move = pyqtSignal(float, float, float, float, float, bool)
    sig_ik_preview = pyqtSignal(float, float, float, float, bool)
    sig_run_sequence = pyqtSignal(list)
    sig_arm_init = pyqtSignal()
    sig_arm_reset = pyqtSignal()
    sig_full_exit = pyqtSignal() # [新增] 用于完全退出 (To Walking)
    sig_vis_clear = pyqtSignal()
    sig_req_auto_grasp = pyqtSignal()
    sig_live_detect_toggled = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.waypoints = [] 
        self.last_vis_pos = None 
        self.app_state = 'IDLE'
        # [新增] 安全互锁标志：True 表示手臂已伸出/未归位，禁止直接退出
        self.is_arm_extended = False 
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        self.setStyleSheet(Styles.PRECISION_GROUP)

        # Group 1: 目标姿态设定
        grp_pose = QGroupBox("目标姿态设定 (TARGET POSE)")
        layout_pose = QVBoxLayout(grp_pose)
        layout_pose.setSpacing(12)
        layout_pose.setContentsMargins(15, 20, 15, 15)

        from PyQt5.QtWidgets import QGridLayout
        grid_layout = QGridLayout()
        grid_layout.setVerticalSpacing(10)
        grid_layout.setHorizontalSpacing(15)

        
        # X (修改默认值为 35 / 0.35)
        grid_layout.addWidget(QLabel("X (m)"), 0, 0)
        # 参数含义: (最小值, 最大值, 默认值, 颜色)
        self.sl_x = self._make_slider(20, 40, 35, "#00e5ff") 
        self.inp_x = SmartLineEdit("0.35"); self.inp_x.setFixedWidth(60) 
        self._bind_slider_input(self.sl_x, self.inp_x, 0.01)
        grid_layout.addWidget(self.sl_x, 0, 1); grid_layout.addWidget(self.inp_x, 0, 2)

        # Y (保持 0)
        grid_layout.addWidget(QLabel("Y (m)"), 1, 0)
        self.sl_y = self._make_slider(0, 35, 0, "#00e676") # <--- 建议检查范围是否够用
        self.inp_y = SmartLineEdit("0.00"); self.inp_y.setFixedWidth(60)
        self._bind_slider_input(self.sl_y, self.inp_y, 0.01)
        grid_layout.addWidget(self.sl_y, 1, 1); grid_layout.addWidget(self.inp_y, 1, 2)

        # Z (修改默认值为 15 / 0.15)
        grid_layout.addWidget(QLabel("Z (m)"), 2, 0)
        self.sl_z = self._make_slider(5, 30, 15, "#ff9800") # 
        self.inp_z = SmartLineEdit("0.15"); self.inp_z.setFixedWidth(60) 
        self._bind_slider_input(self.sl_z, self.inp_z, 0.01)
        grid_layout.addWidget(self.sl_z, 2, 1); grid_layout.addWidget(self.inp_z, 2, 2)
        # Splitter
        line_sep = QFrame(); line_sep.setFrameShape(QFrame.HLine); line_sep.setStyleSheet("color:#333")
        grid_layout.addWidget(line_sep, 3, 0, 1, 3)

        # Roll
        grid_layout.addWidget(QLabel("Gnd Angle (deg)"), 4, 0)
        self.sl_roll = self._make_slider(0, 180, 0, "#d500f9")
        self.inp_roll = SmartLineEdit("0"); self.inp_roll.setFixedWidth(60)
        self._bind_slider_input(self.sl_roll, self.inp_roll, 1.0)
        grid_layout.addWidget(self.sl_roll, 4, 1); grid_layout.addWidget(self.inp_roll, 4, 2)
        
        layout_pose.addLayout(grid_layout)

        row_param = QHBoxLayout()
        row_param.addWidget(QLabel("时长(s):"))
        self.inp_time = SmartLineEdit("0.8"); self.inp_time.setFixedWidth(50)
        row_param.addWidget(self.inp_time)
        row_param.addSpacing(20)
        
        self.chk_realtime = QCheckBox("⚡ 实时跟随")
        self.chk_realtime.setStyleSheet("color: #00e5ff; font-weight: bold;")
        self.chk_realtime.setFocusPolicy(Qt.NoFocus)
        row_param.addWidget(self.chk_realtime)
        
        self.chk_trajectory = QCheckBox("📏 直线轨迹")
        self.chk_trajectory.setStyleSheet("color: #e040fb; font-weight: bold;")
        self.chk_trajectory.setChecked(True)
        self.chk_trajectory.setFocusPolicy(Qt.NoFocus)
        row_param.addWidget(self.chk_trajectory)
        row_param.addStretch()
        layout_pose.addLayout(row_param)

        # =======================================================
        # [修改] 动作：把“执行手动设定移动”按钮移到这里
        # =======================================================
        self.btn_ik_move = QPushButton("执行手动设定移动 (MANUAL EXECUTE)")
        self.btn_ik_move.setFixedHeight(35) # 稍微加高一点点，显眼
        self.btn_ik_move.setCursor(Qt.PointingHandCursor)
        self.btn_ik_move.setStyleSheet("QPushButton { background-color: #1565c0; color: #bbdefb; font-weight: bold; border-radius: 4px; font-size: 13px; margin-top: 5px; } QPushButton:hover { background-color: #1976d2; color: white; }")
        self.btn_ik_move.clicked.connect(self.on_ik_execute)
        layout_pose.addWidget(self.btn_ik_move)

        # =======================================================
        # 分割线 (把手动控制和下面的自动抓取隔开)
        # =======================================================
        line_auto = QFrame(); line_auto.setFrameShape(QFrame.HLine); line_auto.setStyleSheet("color:#444; margin: 10px 0;")
        layout_pose.addWidget(line_auto)

        # --- 原有代码：智能抓取部分 ---
        self.chk_live_detect = QCheckBox("🔴 开启实时目标检测 (Real-time Detection)")
        self.chk_live_detect.setStyleSheet("QCheckBox { color: #00e5ff; font-weight: bold; font-size: 14px; } QCheckBox::indicator { width: 18px; height: 18px; }")
        self.chk_live_detect.toggled.connect(self.sig_live_detect_toggled.emit)
        layout_pose.addWidget(self.chk_live_detect)

        self.btn_auto_grasp = QPushButton("👁️ 智能识别并抓取 (AUTO DETECT & GRASP)")
        self.btn_auto_grasp.setFixedHeight(30)
        self.btn_auto_grasp.setCursor(Qt.PointingHandCursor)
        self.btn_auto_grasp.setStyleSheet(Styles.BTN_AUTO_GRASP)
        self.btn_auto_grasp.clicked.connect(self.on_btn_auto_grasp_clicked)
        layout_pose.addWidget(self.btn_auto_grasp)

        # Init & Exit
        hbox_init = QHBoxLayout(); hbox_init.setSpacing(10)
        self.btn_safe_init = QPushButton("🛡️ 手臂任务初始化 (SAFE INIT)")
        self.btn_safe_init.setFixedHeight(30); self.btn_safe_init.setCursor(Qt.PointingHandCursor)
        self.btn_safe_init.setStyleSheet("QPushButton { background-color: #7b1fa2; color: white; font-weight: bold; border-radius: 4px; font-size: 13px; } QPushButton:hover { background-color: #9c27b0; }")
        self.btn_safe_init.clicked.connect(self.on_arm_safe_init)
        
        self.btn_exit_arm = QPushButton("🔙 收回机械臂 (RETRACT)") 
        self.btn_exit_arm.setFixedHeight(30); self.btn_exit_arm.setCursor(Qt.PointingHandCursor)
        self.btn_exit_arm.setStyleSheet("QPushButton { background-color: #455a64; color: #eceff1; font-weight: bold; border-radius: 4px; font-size: 13px; } QPushButton:hover { background-color: #546e7a; }")
        self.btn_exit_arm.clicked.connect(self.on_arm_retract) # 绑定到收回函数
        
        hbox_init.addWidget(self.btn_safe_init)
        hbox_init.addWidget(self.btn_exit_arm)
        layout_pose.addLayout(hbox_init)

        layout.addWidget(grp_pose)

        # Group 2: 航点序列
        grp_seq = QGroupBox("航点序列任务 (MISSION SEQUENCE)")
        layout_seq = QVBoxLayout(grp_seq)
        layout_seq.setContentsMargins(10, 20, 10, 10)
        layout_seq.setSpacing(10)

        tb_layout = QHBoxLayout()
        tb_layout.addWidget(QLabel("到达后动作:"))
        self.combo_act = QComboBox()
        self.combo_act.addItems(["无动作", "✊ 抓取", "🖐 张开"])
        self.combo_act.setFixedWidth(90)
        self.combo_act.setStyleSheet("background:#1a1a1a; color:white; border:1px solid #555; padding: 2px;")
        tb_layout.addWidget(self.combo_act)
        
        btn_add_wp = QPushButton("➕ 标记当前点")
        btn_add_wp.setCursor(Qt.PointingHandCursor)
        btn_add_wp.setStyleSheet("background-color:#2e7d32; color:white; border-radius:3px; padding: 5px 10px; font-weight:bold;")
        btn_add_wp.clicked.connect(self.on_add_waypoint)
        tb_layout.addWidget(btn_add_wp)
        
        btn_del_wp = QPushButton("❌ 删除选中")
        btn_del_wp.setCursor(Qt.PointingHandCursor)
        btn_del_wp.setStyleSheet("background-color:#c62828; color:white; border-radius:3px; padding: 5px 10px;")
        btn_del_wp.clicked.connect(self.on_del_waypoint)
        tb_layout.addWidget(btn_del_wp)
        tb_layout.addStretch()
        layout_seq.addLayout(tb_layout)

        self.wp_table = QTableWidget()
        self.wp_table.setColumnCount(5)
        self.wp_table.setHorizontalHeaderLabels(["ID", "XYZ坐标", "ROLL", "动作", "时长"])
        self.wp_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.wp_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.wp_table.verticalHeader().setVisible(False)
        self.wp_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.wp_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.wp_table.setFixedHeight(150)
        self.wp_table.setStyleSheet("QTableWidget { background:#1a1a1a; border:1px solid #444; gridline-color:#333; } QHeaderView::section { background:#2d2d2d; border:none; color:#aaa; }")
        layout_seq.addWidget(self.wp_table)

        action_layout = QHBoxLayout()
        btn_save = QPushButton("💾 保存"); btn_save.setFixedWidth(70); btn_save.setFixedHeight(35)
        btn_save.setStyleSheet("background:#37474f; color:#eceff1; border-radius:4px;")
        btn_save.clicked.connect(self.on_save_sequence)
        
        btn_load = QPushButton("📂 加载"); btn_load.setFixedWidth(70); btn_load.setFixedHeight(35)
        btn_load.setStyleSheet("background:#37474f; color:#eceff1; border-radius:4px;")
        btn_load.clicked.connect(self.on_load_sequence)
        
        btn_run = QPushButton("▶ 执行序列 (RUN)")
        btn_run.setFixedHeight(35)
        btn_run.setCursor(Qt.PointingHandCursor)
        btn_run.setStyleSheet(Styles.BTN_IK_EXECUTE)
        btn_run.clicked.connect(self.on_run_sequence)
        
        btn_clr = QPushButton("清空"); btn_clr.setFixedWidth(60); btn_clr.setFixedHeight(35)
        btn_clr.setStyleSheet("background:#37474f; color:#ff5252; border-radius:4px;")
        btn_clr.clicked.connect(self.on_clear_sequence)

        action_layout.addWidget(btn_save); action_layout.addWidget(btn_load)
        action_layout.addSpacing(10)
        action_layout.addWidget(btn_run)
        action_layout.addSpacing(10)
        action_layout.addWidget(btn_clr)
        layout_seq.addLayout(action_layout)
        layout.addWidget(grp_seq)

        layout.addStretch()

        # Group 3: 底部系统按钮
        sys_layout = QHBoxLayout()
        self.btn_exit_ik = QPushButton("🚪 退出控制模式 (BACK TO WALK)")
        self.btn_exit_ik.setFixedHeight(35)
        self.btn_exit_ik.setCursor(Qt.PointingHandCursor)
        self.btn_exit_ik.setStyleSheet("background-color:#b71c1c; color:white; border:1px solid #e57373; border-radius:5px; font-weight:bold;")
        self.btn_exit_ik.clicked.connect(self.on_full_exit) # 绑定到完全退出函数
        
        sys_layout.addWidget(self.btn_exit_ik)
        layout.addLayout(sys_layout)

    # ... Helper Methods ...
    from PyQt5.QtWidgets import QSlider
    def _make_slider(self, min_val, max_val, init_val, color_hex):
        from PyQt5.QtWidgets import QSlider
        sl = QSlider(Qt.Horizontal)
        sl.setRange(min_val, max_val)
        sl.setValue(init_val)
        sl.setFocusPolicy(Qt.NoFocus)
        sl.setStyleSheet(Styles.get_slider_style(color_hex))
        return sl

    def _bind_slider_input(self, slider, line_edit, scale_factor):
        def on_value_change(v):
            val = v * scale_factor
            line_edit.setText(f"{val:.2f}".rstrip('0').rstrip('.'))
            self._trigger_ik_preview()
        slider.valueChanged.connect(on_value_change)
        
        def on_text_edit():
            try: 
                val = float(line_edit.text() or 0)
                slider.setValue(int(val / scale_factor))
                self._trigger_ik_preview()
            except ValueError: pass
        line_edit.editingFinished.connect(on_text_edit)

    def _trigger_ik_preview(self):
        if not self.isVisible(): return
        try:
            x = float(self.inp_x.text())
            y = float(self.inp_y.text())
            z = float(self.inp_z.text())
            roll_rad = math.radians(float(self.inp_roll.text()))
            is_realtime = self.chk_realtime.isChecked()
            self.sig_ik_preview.emit(x, y, z, roll_rad, is_realtime)
        except: pass

    def update_data(self, status_data):
        self.app_state = status_data.get('app_state', 'IDLE')
        self.last_vis_pos = status_data.get('vision_pos')
        # 滑块回显逻辑 (省略，防止跟手打架)

    def on_ik_execute(self):
        if self.app_state != 'RUNNING': 
            QMessageBox.warning(self, "错误", "请先启动机器人！"); return
        try:
            x = float(self.inp_x.text())
            y = float(self.inp_y.text())
            z = float(self.inp_z.text())
            roll = math.radians(float(self.inp_roll.text()))
            dur = float(self.inp_time.text())
            use_traj = self.chk_trajectory.isChecked()
            
            self.sig_ik_move.emit(x, y, z, roll, dur, use_traj)
            # [新增] 只要动了，就标记为伸出
            self.is_arm_extended = True
        except ValueError: 
            QMessageBox.warning(self, "错误", "数据格式无效")

    def on_arm_safe_init(self):
        if self.app_state != 'RUNNING': 
            QMessageBox.warning(self, "错误", "请先启动机器人！"); return
        reply = QMessageBox.question(self, "关节初始化", "即将执行【固定角度】初始化序列...\n确认执行？", QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.sig_arm_init.emit()
             # [新增] 标记手臂已伸出
            self.is_arm_extended = True
            # [修改] 这里设置你想要的默认复位值
            self.inp_x.setText("0.35")
            self.sl_x.setValue(35)
            self.inp_y.setText("0.00")
            self.sl_y.setValue(0)
            self.inp_z.setText("0.15")
            self.sl_z.setValue(15)
            self.inp_roll.setText("0") # Roll 通常复位为 0
            self.sl_roll.setValue(0)

    def on_arm_retract(self):
        """逻辑A: 仅收回手臂，保持 IK_HOLD"""
        if self.app_state != 'RUNNING': return
        
        reply = QMessageBox.question(self, "收回机械臂", 
                                     "确认执行收回序列 (LIFT -> RETRACT)？\n\n"
                                     "注意：手臂将收回体侧并保持锁定状态。", 
                                     QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            self.sig_arm_reset.emit() # 发送收回信号
             # [新增] 解除锁定，标记为已安全收回
            self.is_arm_extended = False
            # [修改] 这里设置你想要的默认复位值
            self.inp_x.setText("0.35")
            self.sl_x.setValue(35)
            self.inp_y.setText("0.00")
            self.sl_y.setValue(0)
            self.inp_z.setText("0.15")
            self.sl_z.setValue(15)
            self.inp_roll.setText("0") # Roll 通常复位为 0
            self.sl_roll.setValue(0)
        
    def on_full_exit(self):
        """逻辑B: 完全退出控制，切回 Walking，手臂泄力"""
        if self.app_state != 'RUNNING': return

        # [新增] 安全拦截检查
        if self.is_arm_extended:
            QMessageBox.critical(self, "安全拦截", 
                                 "🚫 禁止直接退出！\n\n"
                                 "检测到手臂处于【伸出状态】。直接退出会导致手臂砸向地面或机身。\n\n"
                                 "请先点击上方的【🔙 收回机械臂】按钮，待手臂归位后再退出。")
            return

        reply = QMessageBox.warning(self, "退出控制", 
                                    "⚠️ 确认退出手臂控制？\n\n"
                                    "1. 机器人将切回 WALKING 行走模式。\n"
                                    "2. 手臂将失去刚性（柔性泄力）。", 
                                    QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.sig_full_exit.emit()
            
            # UI 归零 (可选)
            self.inp_x.setText("0.35") # 归位到默认起始值或许更好，或者 0.00
            self.sl_x.setValue(35)
            self.inp_y.setText("0.00")
            self.sl_y.setValue(0)
            self.inp_z.setText("0.15")
            self.sl_z.setValue(15)
            self.inp_roll.setText("0") 
            self.sl_roll.setValue(0)


    # ... (序列操作逻辑保持不变) ...
    def on_add_waypoint(self):
        try:
            x = float(self.inp_x.text()); y = float(self.inp_y.text()); z = float(self.inp_z.text())
            roll = float(self.inp_roll.text()); dur = float(self.inp_time.text())
            act_idx = self.combo_act.currentIndex(); act_str = self.combo_act.currentText()
            wp = {'x': x, 'y': y, 'z': z, 'roll': math.radians(roll), 'duration': dur, 'action': act_idx, 'act_str': act_str}
            self.waypoints.append(wp)
            self._refresh_wp_table()
        except ValueError: QMessageBox.warning(self, "错误", "坐标数据无效")

    def on_del_waypoint(self):
        row = self.wp_table.currentRow()
        if row >= 0: self.waypoints.pop(row); self._refresh_wp_table(); self.sig_vis_clear.emit()

    def on_clear_sequence(self):
        self.waypoints = []; self._refresh_wp_table(); self.sig_vis_clear.emit()

    def _refresh_wp_table(self):
        self.wp_table.setRowCount(len(self.waypoints))
        for i, wp in enumerate(self.waypoints):
            self.wp_table.setItem(i, 0, QTableWidgetItem(str(i+1)))
            self.wp_table.setItem(i, 1, QTableWidgetItem(f"({wp['x']:.2f}, {wp['y']:.2f}, {wp['z']:.2f})"))
            self.wp_table.setItem(i, 2, QTableWidgetItem(f"{math.degrees(wp['roll']):.0f}°"))
            item_act = QTableWidgetItem(wp['act_str'])
            if wp['action'] == ACT_GRASP: item_act.setForeground(QColor("#ff9800"))
            elif wp['action'] == ACT_OPEN: item_act.setForeground(QColor("#00e676"))
            self.wp_table.setItem(i, 3, item_act)
            self.wp_table.setItem(i, 4, QTableWidgetItem(f"{wp['duration']}s"))
        self.wp_table.scrollToBottom()

    def on_save_sequence(self):
        if not self.waypoints: return
        fname, _ = QFileDialog.getSaveFileName(self, '保存航点序列', 'waypoints.json', "JSON Files (*.json)")
        if fname:
            try:
                with open(fname, 'w', encoding='utf-8') as f: json.dump(self.waypoints, f, indent=4, ensure_ascii=False)
                QMessageBox.information(self, "成功", f"序列已保存至: {pathlib.Path(fname).name}")
            except Exception as e: QMessageBox.critical(self, "错误", f"保存失败:\n{str(e)}")

    def on_load_sequence(self):
        fname, _ = QFileDialog.getOpenFileName(self, '加载航点序列', '', "JSON Files (*.json)")
        if fname:
            try:
                with open(fname, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                if isinstance(data, list) and len(data) > 0 and 'x' in data[0]:
                    self.waypoints = data
                    self._refresh_wp_table()
                    QMessageBox.information(self, "成功", f"已加载 {len(data)} 个航点")
                else: QMessageBox.warning(self, "错误", "文件格式不正确！")
            except Exception as e: QMessageBox.critical(self, "错误", f"加载失败:\n{str(e)}")

    def on_run_sequence(self):
        if not self.waypoints: QMessageBox.warning(self, "提示", "航点列表为空"); return
        if self.app_state != 'RUNNING': QMessageBox.warning(self, "错误", "请先启动机器人！"); return
        self.sig_run_sequence.emit(self.waypoints)
         # [新增] 标记手臂已伸出
        self.is_arm_extended = True


    # ... (智能抓取逻辑保持不变) ...
    def on_btn_auto_grasp_clicked(self):
        if self.app_state != 'RUNNING': QMessageBox.warning(self, "错误", "请先启动机器人！"); return
        self.sig_req_auto_grasp.emit()

    def open_grasp_confirm_dialog(self):
        if self.last_vis_pos is None: QMessageBox.warning(self, "警告", "目标深度无效"); return
        x, y, z = self.last_vis_pos
        dlg = QDialog(self); dlg.setWindowTitle("智能抓取参数确认"); dlg.setFixedSize(350, 320)
        dlg.setStyleSheet("QDialog { background-color: #222; color: #eee; } QLineEdit { background: #333; color: #00e5ff; font-weight: bold; border: 1px solid #555; } QLabel { font-size: 13px; }")
        layout = QFormLayout(dlg); layout.setSpacing(10)
        inp_x = QLineEdit(f"{x:.3f}"); layout.addRow("目标 X (前后):", inp_x)
        inp_y = QLineEdit(f"{y:.3f}"); layout.addRow("目标 Y (左右):", inp_y)
        inp_z = QLineEdit(f"{z:.3f}"); layout.addRow("目标 Z (高度):", inp_z)
        auto_roll_rad = math.atan2(y, x)
        inp_roll = QLineEdit(f"{math.degrees(auto_roll_rad):.1f}"); layout.addRow("手腕 Roll (°):", inp_roll)
        layout.addRow(QLabel("-" * 40))
        inp_offset = QLineEdit("0.12"); layout.addRow("预抓取距离 (m):", inp_offset)
        inp_lift = QLineEdit("0.15"); layout.addRow("抓后抬起 (m):", inp_lift)
        layout.addRow(QLabel("提示: 请确认 Z 轴高度是否在瓶身中部。"))
        btn_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btn_box.button(QDialogButtonBox.Ok).setText("执行抓取")
        btn_box.button(QDialogButtonBox.Cancel).setText("取消")
        btn_box.accepted.connect(dlg.accept); btn_box.rejected.connect(dlg.reject)
        layout.addWidget(btn_box)
        if dlg.exec_() == QDialog.Accepted:
            try:
                tx, ty, tz = float(inp_x.text()), float(inp_y.text()), float(inp_z.text())
                roll_rad = math.radians(float(inp_roll.text()))
                lift_h = float(inp_lift.text())
                self._generate_and_run_grasp(tx, ty, tz, roll_rad, lift_h)
            except ValueError: QMessageBox.warning(self, "错误", "参数输入无效！")

    def _generate_and_run_grasp(self, tx, ty, tz, roll, lift_h):
        safe_retract_dist = 0.15; safe_x = tx - safe_retract_dist
        target_grasp_x = tx - 0.06; y_align = ty + 0.1; z_align = tz - 0.03
        seq = []
        seq.append({'x': safe_x, 'y': y_align, 'z': tz, 'roll': roll, 'duration': 1.5, 'action': ACT_OPEN, 'act_str': '1.安全撤回'})
        seq.append({'x': safe_x, 'y': y_align, 'z': z_align, 'roll': roll, 'duration': 1.0, 'action': ACT_OPEN, 'act_str': '2.侧向对齐'})
        seq.append({'x': target_grasp_x, 'y': y_align, 'z': z_align, 'roll': roll, 'duration': 1.2, 'action': ACT_GRASP, 'act_str': '3.✊平推抓取'})
        seq.append({'x': target_grasp_x, 'y': y_align, 'z': z_align + lift_h, 'roll': roll, 'duration': 1.0, 'action': ACT_NONE, 'act_str': '4.抬起物体'})
        seq.append({'x': safe_x, 'y': y_align, 'z': z_align + lift_h, 'roll': 0.0, 'duration': 1.5, 'action': ACT_NONE, 'act_str': '5.安全回收'})
        self.waypoints = seq; self._refresh_wp_table(); self.sig_run_sequence.emit(self.waypoints)
