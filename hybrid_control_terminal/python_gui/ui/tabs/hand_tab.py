# ui/tabs/hand_tab.py
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QFrame, QGroupBox, QGridLayout, QPushButton)
from PyQt5.QtCore import Qt, pyqtSignal

from ui.styles import Styles
from ui.widgets import SmartLineEdit, FingerHeatmap

class HandTab(QWidget):
    """
    Tab 6: 灵巧手 L10 控制面板
    职责：发送抓取/张开指令，显示指尖压力热力图
    """
    # 对外信号：(动作名称, 直径mm)
    sig_hand_cmd = pyqtSignal(str, int)

    def __init__(self):
        super().__init__()
        self.heatmaps = [] # 存储 (HeatmapWidget, Label) 元组
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        
        # --- 顶部：状态与配置 ---
        top_frame = QFrame()
        top_frame.setStyleSheet("background-color: #151922; border-radius: 6px;")
        top_layout = QHBoxLayout(top_frame)
        
        self.lbl_hand_status = QLabel("L10 链接状态: 未知")
        self.lbl_hand_status.setStyleSheet("color: #aaa; font-weight: bold; font-size: 14px;")
        top_layout.addWidget(self.lbl_hand_status)
        top_layout.addStretch()
        layout.addWidget(top_frame)

        # --- 中部：动作指令控制区 ---
        ctrl_group = QGroupBox("动作指令")
        ctrl_group.setStyleSheet(Styles.HAND_CTRL_GROUP)
        cg_layout = QGridLayout(ctrl_group)
        cg_layout.setSpacing(15)

        cg_layout.addWidget(QLabel("物体直径 (mm):"), 0, 0)
        self.inp_hand_mm = SmartLineEdit("30")
        self.inp_hand_mm.setFixedHeight(40)
        self.inp_hand_mm.setStyleSheet("font-size: 18px; color: yellow; background: #333; border: 1px solid #555;")
        cg_layout.addWidget(self.inp_hand_mm, 0, 1)

        # 创建三个大按钮
        btn_open = self._make_btn("🖐 全手张开", "#2e7d32", lambda: self._on_btn_click('OPEN'))
        btn_fist = self._make_btn("✊ 强力握拳", "#4527a0", lambda: self._on_btn_click('FIST'))
        btn_grasp = self._make_btn("🤏 智能抓取", "#e65100", lambda: self._on_btn_click('GRASP'))
        
        cg_layout.addWidget(btn_open, 1, 0)
        cg_layout.addWidget(btn_fist, 1, 1)
        cg_layout.addWidget(btn_grasp, 0, 2, 2, 1) # 跨两行
        layout.addWidget(ctrl_group)

        # --- 底部：压力反馈可视化 (Heatmap) ---
        feed_group = QGroupBox("指尖矩阵压感 (Matrix Tactile)")
        feed_group.setStyleSheet(Styles.HAND_FEED_GROUP)
        fg_layout = QHBoxLayout(feed_group)
        fg_layout.setSpacing(20)

        fingers = ["拇指", "食指", "中指", "无名", "小指"]
        for name in fingers:
            vbox = QVBoxLayout()
            vbox.setSpacing(5)
            
            val_lbl = QLabel("0")
            val_lbl.setAlignment(Qt.AlignCenter)
            val_lbl.setStyleSheet("color: #00e676; font-weight: bold; font-family: Consolas;")
            
            heatmap = FingerHeatmap()
            
            name_lbl = QLabel(name)
            name_lbl.setAlignment(Qt.AlignCenter)
            name_lbl.setStyleSheet("color: #aaa; font-size: 12px;")
            
            vbox.addWidget(val_lbl)
            vbox.addWidget(heatmap)
            vbox.addWidget(name_lbl)
            
            fg_layout.addLayout(vbox)
            self.heatmaps.append((heatmap, val_lbl))

        layout.addWidget(feed_group)
        layout.addStretch()

    def _make_btn(self, text, color, slot):
        b = QPushButton(text)
        b.setFixedHeight(60)
        b.setCursor(Qt.PointingHandCursor)
        b.setStyleSheet(Styles.get_hand_btn_style(color))
        b.clicked.connect(slot)
        return b

    def _on_btn_click(self, action):
        try:
            mm = int(self.inp_hand_mm.text())
        except:
            mm = 30
            self.inp_hand_mm.setText("30")
        self.sig_hand_cmd.emit(action, mm)

    # =============================================================
    # 数据更新接口
    # =============================================================
    def update_data(self, status_data):
        forces = status_data.get('hand_force', [])
        matrices = status_data.get('hand_matrix', [])
        
        # 更新连接状态灯
        if forces and len(forces) >= 5:
            self.lbl_hand_status.setText("L10 链接状态: ● 在线 (Online)")
            self.lbl_hand_status.setStyleSheet("color: #00e676; font-weight: bold; font-size: 14px;")
            
            # 更新五个手指的数据
            for i in range(5):
                if i < len(self.heatmaps):
                    hm_widget, val_lbl = self.heatmaps[i]
                    
                    # 1. 更新数值 (简单的牛顿转换假设)
                    raw_val = int(forces[i])
                    # 这里的转换系数沿用之前的逻辑
                    force_newton = (raw_val / 2.0) * 0.02 
                    
                    val_lbl.setText(f"{force_newton:.1f} N")
                    if force_newton > 3.0: 
                        val_lbl.setStyleSheet("color: #ff5252; font-weight: bold; font-size: 16px;") 
                    else: 
                        val_lbl.setStyleSheet("color: #00e676; font-weight: bold; font-family: Consolas;") 
                    
                    # 2. 更新热力图矩阵
                    if matrices and i < len(matrices): 
                        hm_widget.update_data(matrices[i])
        else:
            self.lbl_hand_status.setText("L10 链接状态: ○ 离线")
            self.lbl_hand_status.setStyleSheet("color: #555; font-weight: bold; font-size: 14px;")
