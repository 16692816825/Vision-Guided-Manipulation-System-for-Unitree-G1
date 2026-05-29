# ui/tabs/teach_tab.py
import os
import pathlib
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                             QPushButton, QFrame, QGroupBox, QListWidget,
                             QInputDialog, QMessageBox, QDialog, QLineEdit)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal

from config import DATA_LOG_DIR
from ui.styles import Styles

class TeachTab(QWidget):
    """
    Tab 3: 动作示教面板
    职责：管理动作文件的录制(带倒计时)、列表展示、删除、回放控制
    """
    # === 定义对外信号 ===
    sig_start_teach = pyqtSignal()          # 开始录制 (倒计时结束后触发)
    sig_stop_teach = pyqtSignal(str)        # 停止录制并保存 (参数: 文件名)
    sig_start_replay = pyqtSignal(str)      # 开始回放 (参数: 文件名)
    sig_exit_teach = pyqtSignal()           # 退出示教模式

    def __init__(self):
        super().__init__()
        self.last_app_state = 'IDLE' # 缓存机器人状态，用于按钮禁用检查
        self.last_mode = 'WALKING'
        self.is_replaying = False    # 回放状态标记
        self.has_replay_started = False 
        
        self._init_ui()
        self.refresh_file_list()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        
        # 应用分组样式
        self.setStyleSheet(Styles.TEACH_GROUP)

        # 1. 顶部：操作指引卡片
        frame_guide = QFrame()
        frame_guide.setStyleSheet("background-color: #263238; border: 1px solid #37474f; border-radius: 6px;")
        frame_guide.setFixedHeight(80)
        guide_layout = QHBoxLayout(frame_guide)
        
        lbl_icon = QLabel("ℹ️")
        lbl_icon.setStyleSheet("font-size: 30px; border: none; background: transparent;")
        
        lbl_text = QLabel(
            "<b>操作流程指南:</b><br>"
            "<span style='color:#b0bec5'>1. 点击 [开始录制] -> 拖拽手臂 -> 再次点击结束并保存<br>"
            "2. 在下方列表选择文件 -> 点击 [回放动作] -> 观察效果</span>"
        )
        lbl_text.setStyleSheet("font-size: 13px; border: none; background: transparent;")
        
        guide_layout.addWidget(lbl_icon)
        guide_layout.addWidget(lbl_text, 1)
        layout.addWidget(frame_guide)

        # 2. 中部：动作文件库
        grp_lib = QGroupBox("动作文件库 (MOTION LIBRARY)")
        lib_layout = QVBoxLayout(grp_lib)
        lib_layout.setContentsMargins(10, 20, 10, 10)

        # 2.1 工具栏
        tool_layout = QHBoxLayout()
        tool_layout.addStretch()
        
        btn_refresh = QPushButton("⟳ 刷新列表")
        btn_refresh.setFixedWidth(90)
        btn_refresh.setCursor(Qt.PointingHandCursor)
        btn_refresh.setStyleSheet("background-color:#2e7d32; color:white; border-radius:3px; padding:4px;")
        btn_refresh.clicked.connect(self.refresh_file_list)
        
        btn_del = QPushButton("🗑 删除选中")
        btn_del.setFixedWidth(90)
        btn_del.setCursor(Qt.PointingHandCursor)
        btn_del.setStyleSheet("background-color:#c62828; color:white; border-radius:3px; padding:4px;")
        btn_del.clicked.connect(self.on_del_file)
        
        tool_layout.addWidget(btn_refresh)
        tool_layout.addWidget(btn_del)
        lib_layout.addLayout(tool_layout)

        # 2.2 列表控件
        self.list_files = QListWidget()
        self.list_files.setFocusPolicy(Qt.NoFocus)
        self.list_files.setStyleSheet(Styles.TEACH_LIST)
        lib_layout.addWidget(self.list_files, 1)
        layout.addWidget(grp_lib, 1)

        # 3. 底部：操作控制台
        grp_ctrl = QGroupBox("操作控制台 (CONTROL DECK)")
        ctrl_layout = QVBoxLayout(grp_ctrl)
        ctrl_layout.setContentsMargins(15, 25, 15, 15)
        ctrl_layout.setSpacing(15)

        # 3.1 状态显示
        self.lbl_status = QLabel("当前状态: 就绪 (Ready)")
        self.lbl_status.setAlignment(Qt.AlignCenter)
        self.lbl_status.setStyleSheet("color: #00e5ff; font-weight: bold; font-size: 14px; border-bottom: 1px solid #333; padding-bottom: 5px;")
        ctrl_layout.addWidget(self.lbl_status)

        # 3.2 录制与回放按钮
        act_layout = QHBoxLayout()
        
        self.btn_record = QPushButton("🔴 开始录制 (RECORD)")
        self.btn_record.setFixedHeight(60)
        self.btn_record.setCursor(Qt.PointingHandCursor)
        self.btn_record.setStyleSheet(Styles.BTN_RECORD_ORANGE)
        self.btn_record.clicked.connect(self.on_record_click)
        
        self.btn_replay = QPushButton("▶ 回放动作 (PLAY)")
        self.btn_replay.setFixedHeight(60)
        self.btn_replay.setCursor(Qt.PointingHandCursor)
        self.btn_replay.setStyleSheet(Styles.BTN_REPLAY_BLUE)
        self.btn_replay.clicked.connect(self.on_replay_click)
        
        act_layout.addWidget(self.btn_record)
        act_layout.addWidget(self.btn_replay)
        ctrl_layout.addLayout(act_layout)

        # 3.3 退出按钮
        self.btn_exit = QPushButton("🛑 退出示教模式 (恢复行走控制)")
        self.btn_exit.setFixedHeight(50)
        self.btn_exit.setCursor(Qt.PointingHandCursor)
        self.btn_exit.setEnabled(False)
        self.btn_exit.setStyleSheet(Styles.BTN_EXIT_TEACH)
        self.btn_exit.clicked.connect(self.on_exit_click)
        ctrl_layout.addWidget(self.btn_exit)

        layout.addWidget(grp_ctrl)

    # =============================================================
    # 逻辑处理
    # =============================================================
    def refresh_file_list(self, checked=False):
        """刷新文件列表"""
        self.list_files.clear()
        if DATA_LOG_DIR.exists():
            for f in sorted(list(DATA_LOG_DIR.glob("*.json"))): 
                self.list_files.addItem(f.stem)

    def on_del_file(self, checked=False):
        """删除文件逻辑"""
        item = self.list_files.currentItem()
        if not item:
            QMessageBox.warning(self, "提示", "请先选择一个要删除的文件")
            return
            
        filename = item.text()
        file_path = DATA_LOG_DIR / f"{filename}.json"
        
        reply = QMessageBox.question(self, "确认删除", 
                                     f"确定要永久删除文件 '{filename}' 吗？",
                                     QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            try:
                if file_path.exists():
                    os.remove(file_path)
                    self.refresh_file_list()
                    QMessageBox.information(self, "成功", "文件已删除")
                else:
                    QMessageBox.warning(self, "错误", "文件不存在")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"删除失败:\n{str(e)}")

    def on_record_click(self, checked=False):
        """点击录制按钮"""
        if self.last_app_state != 'RUNNING': 
            QMessageBox.warning(self, "错误", "机器人未运行，无法录制！")
            return
        if self.last_mode not in ['WALKING', 'TEACH_STANDBY']:
            QMessageBox.warning(self, "提示", f"当前模式为 {self.last_mode}，请先退出其他模式再录制。")
            return

        btn_text = self.btn_record.text()
        # 1. 如果正在录制 -> 停止并保存
        if "结束" in btn_text or "STOP" in btn_text:
            name, ok = QInputDialog.getText(self, "保存", "输入动作名称:", QLineEdit.Normal, "motion_new")
            fname = name if (ok and name) else "temp_discard"
            
            # 发送停止信号
            self.sig_stop_teach.emit(fname)
            
            # 恢复 UI
            self.btn_record.setText("🔴 开始录制 (RECORD)")
            self.btn_record.setStyleSheet(Styles.BTN_RECORD_ORANGE)
            self.btn_replay.setEnabled(True)
            self.lbl_status.setText(f"当前状态: 录制完成，已保存为 {fname}")
            self.lbl_status.setStyleSheet("color: #00e676; font-weight: bold; font-size: 14px; border-bottom: 1px solid #333; padding-bottom: 5px;")
            
            QTimer.singleShot(500, self.refresh_file_list)

        # 2. 如果未录制 -> 开始倒计时
        else:
            reply = QMessageBox.question(self, "开始录制", "点击 Yes 将开始 5秒 倒计时。\n\n请准备好手动拖拽机器人手臂。", QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.btn_record.setEnabled(False)
                self.btn_replay.setEnabled(False)
                self.lbl_status.setText("当前状态: ⏳ 倒计时中 (准备松开)...")
                self.lbl_status.setStyleSheet("color: #ffeb3b; font-weight: bold; font-size: 14px; border-bottom: 1px solid #333; padding-bottom: 5px;")
                
                self.count_down_val = 5
                self.timer_record = QTimer(self)
                self.timer_record.timeout.connect(self._tick_countdown)
                self.timer_record.start(1000)
                
                # 倒计时弹窗
                self.progress = QDialog(self)
                self.progress.setWindowTitle("准备中...")
                self.progress.setModal(True)
                self.progress.setFixedSize(300, 100)
                self.progress.setStyleSheet("QDialog { background-color: #263238; } QLabel { color: #e0e0e0; }")
                self.progress_lbl = QLabel("5", self.progress)
                self.progress_lbl.setAlignment(Qt.AlignCenter)
                self.progress_lbl.setStyleSheet("font-size: 40px; font-weight: bold; color: #00e5ff;")
                l = QVBoxLayout(self.progress)
                l.addWidget(QLabel("松开倒计时:"))
                l.addWidget(self.progress_lbl)
                self.progress.show()

    def _tick_countdown(self):
        self.count_down_val -= 1
        if hasattr(self, 'progress_lbl'): 
            self.progress_lbl.setText(str(self.count_down_val))
        
        if self.count_down_val <= 0:
            self.timer_record.stop()
            if hasattr(self, 'progress'): self.progress.close()
            
            self.btn_record.setEnabled(True)
            
            # 发送开始录制信号
            self.sig_start_teach.emit()
            
            # 更新 UI 为“录制中”
            self.btn_record.setText("⬛ 结束录制 (STOP)")
            self.btn_record.setStyleSheet("background-color: #d50000; color: white; font-weight: bold; font-size: 16px; border-radius: 8px; border: 2px solid white;")
            self.lbl_status.setText("当前状态: 🔴 正在录制... (请拖拽手臂)")
            self.lbl_status.setStyleSheet("color: #ff5252; font-weight: bold; font-size: 14px; border-bottom: 1px solid #333; padding-bottom: 5px;")

    def on_replay_click(self, checked=False):
        # [修复] 1. 防止重复点击或信号双重触发
        if self.is_replaying:
            print("[UI] 正在回放中，忽略重复点击")
            return
        """点击回放按钮"""
        item = self.list_files.currentItem()
        if not item: 
            QMessageBox.warning(self, "提示", "请先选择一个动作文件")
            return
        
        if self.last_app_state != 'RUNNING': 
            QMessageBox.warning(self, "错误", "机器人未运行"); return
        if self.last_mode not in ['WALKING', 'TEACH_STANDBY']:
            QMessageBox.warning(self, "提示", f"当前模式为 {self.last_mode}，请先退出其他模式再回放。")
            return
            
        reply = QMessageBox.question(self, "回放", f"确认回放动作: {item.text()}？\n请确保机器人周围无障碍物。", QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes: 
            # [修复] 2. 先锁定状态，再发信号
            self.is_replaying = True
            self.has_replay_started = False
        
            
            # 锁定 UI
            self.btn_record.setEnabled(False)
            self.btn_replay.setEnabled(False)
            self.btn_replay.setText("▶ 回放中...")
            self.lbl_status.setText(f"当前状态: 🔵 正在回放 [{item.text()}]...")
            self.lbl_status.setStyleSheet("color: #00e5ff; font-weight: bold; font-size: 14px; border-bottom: 1px solid #333; padding-bottom: 5px;")
            
            # 发送回放信号
            self.sig_start_replay.emit(item.text())

    def on_exit_click(self, checked=False):
        reply = QMessageBox.warning(self, "退出", "确定恢复行走模式吗？", QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes: 
            self.sig_exit_teach.emit()

    # =============================================================
    # 数据更新接口
    # =============================================================
    def update_data(self, status_data):
        """接收后端状态，管理按钮可用性和回放逻辑"""
        app_state = status_data.get('app_state', 'IDLE')
        mode = status_data.get('mode', 'WALKING')
        self.last_app_state = app_state
        self.last_mode = mode

        # 1. 按钮启用/禁用管理
        is_teach_mode = mode in ['TEACH_STANDBY', 'TEACHING', 'REPLAYING', 'TRANSITION']
        is_nav = (mode == 'NAVIGATING')
        
        # 退出按钮仅在示教相关模式下可用
        self.btn_exit.setEnabled(is_teach_mode)
        
        # 录制/回放在导航模式下禁用，且仅在运行时可用
        can_operate = (app_state == 'RUNNING' and not is_nav and mode in ['WALKING', 'TEACH_STANDBY'])
        
        # 如果正在录制或回放，不要随意 enable 按钮，防止状态打架
        if "结束" not in self.btn_record.text() and not self.is_replaying:
            self.btn_record.setEnabled(can_operate)
            self.btn_replay.setEnabled(can_operate)

        # 2. 回放状态自动复位逻辑 (移植自 MainWindow)
        if self.is_replaying:
            # 阶段一：等待后端进入回放/过渡状态
            if mode in ['REPLAYING', 'TRANSITION']:
                self.has_replay_started = True

            # 阶段二：后端已执行完，切回了 STANDBY
            if self.has_replay_started and mode not in ['REPLAYING', 'TRANSITION']:
                # 结束回放
                self.is_replaying = False
                self.has_replay_started = False
                
                # 恢复 UI
                self.btn_replay.setEnabled(True)
                self.btn_replay.setText("▶ 回放动作 (PLAY)")
                self.btn_replay.setStyleSheet(Styles.BTN_REPLAY_BLUE)
                
                self.lbl_status.setText("当前状态: ✅ 演示完毕 (Ready)")
                self.lbl_status.setStyleSheet("color: #00e676; font-weight: bold; font-size: 14px; border-bottom: 1px solid #333; padding-bottom: 5px;")
                
                # 延时恢复默认提示
                QTimer.singleShot(3000, lambda: self.lbl_status.setText("当前状态: 就绪 (Ready)") if not self.is_replaying else None)
                QTimer.singleShot(3000, lambda: self.lbl_status.setStyleSheet("color: #00e5ff; font-weight: bold; font-size: 14px; border-bottom: 1px solid #333; padding-bottom: 5px;") if not self.is_replaying else None)
