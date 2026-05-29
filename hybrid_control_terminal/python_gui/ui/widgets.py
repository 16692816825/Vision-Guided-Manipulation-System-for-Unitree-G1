# ui/widgets.py
# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, 
                             QButtonGroup, QSizePolicy, QLineEdit, QFrame, QGridLayout)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPoint, QRectF 
from PyQt5.QtGui import QImage, QPainter, QColor, QPen, QFont

import numpy as np

try:
    import pyqtgraph as pg
except ImportError:
    pg = None

# --- [类 0] 智能输入框 (屏蔽方向键，防止误触机器人移动) ---
class SmartLineEdit(QLineEdit):
    def keyPressEvent(self, event):
        # 如果按下方向键，忽略输入框的光标移动，
        # 让事件传递给主窗口去控制机器人
        if event.key() in [Qt.Key_Up, Qt.Key_Down, Qt.Key_Left, Qt.Key_Right, 
                           Qt.Key_W, Qt.Key_A, Qt.Key_S, Qt.Key_D]:
            event.ignore() 
        else:
            super().keyPressEvent(event)

# --- [类 1] 长按按钮 (防止误触) ---
class HoldButton(QPushButton):
    triggered = pyqtSignal()
    
    def __init__(self, text, color_base="#444", color_fill="#00e676", parent=None):
        super().__init__(text, parent)
        self.setCursor(Qt.PointingHandCursor)
        self.setFixedSize(120, 50)
        self.setFocusPolicy(Qt.NoFocus)
        self.color_base = color_base
        self.color_fill = color_fill
        self.progress = 0.0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.is_holding = False
        self._update_style()

    def _update_style(self):
        p = min(1.0, max(0.001, self.progress))
        # 使用 CSS 渐变模拟进度条效果
        self.setStyleSheet(f"""
            QPushButton {{
                background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, 
                    stop:0 {self.color_fill}, stop:{p} {self.color_fill}, 
                    stop:{p+0.001} {self.color_base}, stop:1 {self.color_base});
                border: 1px solid #666; border-radius: 5px; color: white; font-weight: bold; font-size: 14px;
            }}
            QPushButton:pressed {{ border: 1px solid white; }}
            QPushButton:disabled {{ background-color: #222; color: #555; border: 1px solid #333; }}
        """)

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.is_holding = True
            self.progress = 0.0
            self.timer.start(30) # 30ms 刷新一次
        super().mousePressEvent(e)

    def mouseReleaseEvent(self, e):
        self.is_holding = False
        self.timer.stop()
        self.progress = 0.0
        self._update_style()
        super().mouseReleaseEvent(e)

    def _on_tick(self):
        if self.is_holding:
            self.progress += 0.02 # 约 1.5秒 充满
            if self.progress >= 1.0:
                self.progress = 0.0
                self.is_holding = False
                self.timer.stop()
                self._update_style()
                self.triggered.emit() # 触发信号
            else:
                self._update_style()

# --- [类 2] 速度控制条 ---
class SpeedControlWidget(QWidget):
    levelChanged = pyqtSignal(int)
    def __init__(self, title, active_color="#00e5ff"):
        super().__init__()
        l = QVBoxLayout(self)
        l.setContentsMargins(0,0,0,0); l.setSpacing(5)
        l.addWidget(QLabel(title))
        bl = QHBoxLayout()
        self.group = QButtonGroup(self)
        self.buttons = []
        for i, txt in enumerate(["慢速", "标准", "快速"]):
            b = QPushButton(txt)
            b.setCheckable(True)
            b.setFocusPolicy(Qt.NoFocus)
            b.setFixedHeight(30)
            b.setStyleSheet(f"""
                QPushButton {{ background-color: #333; border: 1px solid #555; color: #aaa; }}
                QPushButton:checked {{ background-color: {active_color}; color: #000; font-weight: bold; border: 1px solid {active_color}; }}
                QPushButton:hover:!checked {{ background-color: #444; }}
            """)
            self.group.addButton(b, i)
            self.buttons.append(b)
            bl.addWidget(b)
        self.group.buttonClicked[int].connect(self.levelChanged.emit)
        l.addLayout(bl)
    def set_level(self, l): 
        if 0 <= l < 3: self.buttons[l].setChecked(True)

# --- [类 3] 圆形仪表盘 ---
class DashCircle(QWidget):
    def __init__(self, title, suffix="%", color="#00e676"):
        super().__init__()
        self.title = title; self.suffix = suffix; self.color = color; self.value = 0
        self.setFixedSize(80, 80)
    def set_val(self, v): self.value = v; self.update()
    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        rect = self.rect().adjusted(5,5,-5,-5)
        p.setPen(QPen(QColor("#333"), 4)); p.drawEllipse(rect)
        p.setPen(QPen(QColor(self.color), 4, Qt.SolidLine, Qt.RoundCap))
        span = int(-self.value * 3.6 * 16)
        p.drawArc(rect, 90*16, span)
        p.setPen(QColor("#fff")); font = QFont("Consolas", 14, QFont.Bold); p.setFont(font)
        p.drawText(rect, Qt.AlignCenter, f"{int(self.value)}")
        font.setPointSize(8); p.setFont(font)
        p.drawText(rect.adjusted(0, 25, 0, 0), Qt.AlignCenter, self.suffix)

# --- [类 4] MuJoCo 窗口 (视觉增强版) ---
class MuJoCoWidget(QWidget):
    # 原有的相机控制信号
    camera_control = pyqtSignal(int, float, float)
    
    # [新增] 视觉点击信号: 发送图像像素坐标 (u, v)
    vision_click_signal = pyqtSignal(int, int) 

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 300)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.image = None
        self.overlay_text = "WAITING FOR STREAM..."
        
        self.last_pos = QPoint(); self.is_dragging = False
        self.setFocusPolicy(Qt.ClickFocus)

        # === 视觉增强状态 ===
        self.border_color = QColor("#444")
        self.mode_label = "OFFLINE"
        self.is_flashing = False
        self.flash_on = True
        self.flash_timer = QTimer(self)
        self.flash_timer.setInterval(600)
        self.flash_timer.timeout.connect(self._toggle_flash)

    def _toggle_flash(self):
        self.flash_on = not self.flash_on
        self.update()

    def set_visual_mode(self, mode_type):
        self.mode_label = mode_type
        if mode_type == 'WALK':
            self.border_color = QColor("#00e676") # 绿
            self.is_flashing = False
        elif mode_type == 'ARM':
            self.border_color = QColor("#00e5ff") # 蓝
            self.is_flashing = False
        elif mode_type == 'NAV':
            self.border_color = QColor("#e040fb") # 紫
            self.is_flashing = False
        elif mode_type == 'TEACH':
            self.border_color = QColor("#ffea00") # 黄 (警示)
            self.is_flashing = True
        elif mode_type == 'STOP':
            self.border_color = QColor("#ff1744") # 红 (危险)
            self.is_flashing = True
        else:
            self.border_color = QColor("#444")
            self.is_flashing = False

        if self.is_flashing and not self.flash_timer.isActive():
            self.flash_timer.start()
        elif not self.is_flashing:
            self.flash_timer.stop()
            self.flash_on = True
        self.update()

    def update_frame(self, np_img):
        if np_img is None: return
        h, w, c = np_img.shape
        self.image = QImage(np_img.data, w, h, 3*w, QImage.Format_RGB888)
        self.update()

    def set_overlay_text(self, t): self.overlay_text = t; self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#000"))
        
        if self.image and not self.image.isNull():
            # 保持比例缩放绘制
            scaled = self.image.scaled(self.size(), Qt.KeepAspectRatio, Qt.FastTransformation)
            x = (self.width()-scaled.width())//2
            y = (self.height()-scaled.height())//2
            p.drawImage(x, y, scaled)
        else:
            p.setPen(QPen(QColor("#222"), 1, Qt.DotLine))
            for i in range(0, self.width(), 40): p.drawLine(i, 0, i, self.height())
            for i in range(0, self.height(), 40): p.drawLine(0, i, self.width(), i)
        
        if self.overlay_text:
            p.setPen(QColor("#fff")); font = self.font(); font.setBold(True); font.setPointSize(14); p.setFont(font)
            p.drawText(self.rect(), Qt.AlignCenter, self.overlay_text)

        border_width = 6
        display_color = self.border_color
        if self.is_flashing and not self.flash_on:
            display_color = display_color.darker(300)

        p.setPen(QPen(display_color, border_width))
        p.drawRect(border_width//2, border_width//2, self.width()-border_width, self.height()-border_width)

        if self.mode_label != "IDLE":
            p.fillRect(border_width, border_width, 160, 30, QColor(0,0,0,150))
            p.setPen(display_color)
            font = QFont("Consolas", 12, QFont.Bold); p.setFont(font)
            p.drawText(border_width+10, border_width+20, f"MODE: {self.mode_label}")

    # [核心修改] 鼠标点击事件处理
    def mousePressEvent(self, event):
        # 1. 检查是否是 Ctrl + 左键
        if (event.modifiers() & Qt.ControlModifier) and (event.button() == Qt.LeftButton):
            if self.image and not self.image.isNull():
                # 计算图像在控件中的实际显示区域
                w_widget, h_widget = self.width(), self.height()
                w_img, h_img = self.image.width(), self.image.height()
                
                # 计算缩放比例 (与 paintEvent 逻辑一致)
                scale = min(w_widget / w_img, h_widget / h_img)
                
                # 计算图像在控件中的偏移量 (居中)
                dx = (w_widget - w_img * scale) / 2
                dy = (h_widget - h_img * scale) / 2
                
                # 获取鼠标点击坐标
                mouse_x, mouse_y = event.x(), event.y()
                
                # 逆向计算：(屏幕坐标 - 偏移) / 缩放 = 图像坐标
                img_x = int((mouse_x - dx) / scale)
                img_y = int((mouse_y - dy) / scale)
                
                # 边界检查，确保点在图像内
                if 0 <= img_x < w_img and 0 <= img_y < h_img:
                    print(f"[UI] 视觉点击: ({img_x}, {img_y})")
                    # 发送信号给 Main Window
                    self.vision_click_signal.emit(img_x, img_y)
            return # 拦截事件，不触发后续拖拽逻辑

        # 2. 原有的相机旋转逻辑
        if event.button() in [Qt.LeftButton, Qt.RightButton]: 
            self.is_dragging = True; self.last_pos = event.pos()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event): self.is_dragging = False; super().mouseReleaseEvent(event)
    
    def mouseMoveEvent(self, event):
        if self.is_dragging:
            dx = event.x() - self.last_pos.x(); dy = event.y() - self.last_pos.y()
            action = 2 if (event.buttons() & Qt.LeftButton) else 3
            self.camera_control.emit(action, dx * 0.02, dy * 0.02)
            self.last_pos = event.pos()
        super().mouseMoveEvent(event)
        
    def wheelEvent(self, event):
        self.camera_control.emit(4, 0.0, -event.angleDelta().y() * 0.002)
        super().wheelEvent(event)

# --- [类 5] 示波器 ---
class OscilloscopeWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self); self.layout.setContentsMargins(0, 0, 0, 0)
        if pg is None:
            self.layout.addWidget(QLabel("Please install pyqtgraph"))
            return
        pg.setConfigOption('background', '#111'); pg.setConfigOption('foreground', '#aaa')
        self.win = pg.GraphicsLayoutWidget(); self.layout.addWidget(self.win)
        self.ptr = 0; self.max_len = 400
        self.data_goal = np.zeros((3, self.max_len)); self.data_real = np.zeros((3, self.max_len))
        self.curves_goal = []; self.curves_real = []
        labels = ['X (m)', 'Y (m)', 'Z (m)']
        for i in range(3):
            p = self.win.addPlot(row=i, col=0); p.showGrid(x=False, y=True, alpha=0.3); p.setLabel('left', labels[i])
            if i==0: p.addLegend(offset=(10, 10))
            self.curves_goal.append(p.plot(pen=pg.mkPen(color='#00e5ff', width=2, style=Qt.DashLine), name='Target' if i==0 else None))
            self.curves_real.append(p.plot(pen=pg.mkPen(color='#ffeb3b', width=2), name='Actual' if i==0 else None))

    def update_data(self, goal_pos, hand_pos):
        if pg is None: return
        self.data_goal[:, :-1] = self.data_goal[:, 1:]; self.data_real[:, :-1] = self.data_real[:, 1:]
        self.data_goal[:, -1] = goal_pos; self.data_real[:, -1] = hand_pos
        for i in range(3):
            self.curves_goal[i].setData(self.data_goal[i]); self.curves_real[i].setData(self.data_real[i])

# --- [类 6] 通信状态面板 (新增) ---
class CommStatusWidget(QFrame):
    """通信状态面板：显示网络延迟、接口信息"""
    def __init__(self):
        super().__init__()
        self.setObjectName("InfoBox")
        self.setStyleSheet("""
            #InfoBox { background-color: #222; border: 1px solid #333; border-radius: 5px; }
            QLabel { color: #aaa; font-size: 11px; }
            QLabel#Val { color: #00e5ff; font-weight: bold; font-family: Consolas; }
        """)
        layout = QGridLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setVerticalSpacing(5)

        self.lbl_iface = QLabel("enpXs0")
        self.lbl_lat = QLabel("0.0 ms")
        self.lbl_pkg = QLabel("0")
        
        layout.addWidget(QLabel("INTERFACE:"), 0, 0)
        layout.addWidget(self.lbl_iface, 0, 1, Qt.AlignRight)
        
        layout.addWidget(QLabel("LATENCY:"), 1, 0)
        layout.addWidget(self.lbl_lat, 1, 1, Qt.AlignRight)
        
        layout.addWidget(QLabel("PACKETS:"), 2, 0)
        layout.addWidget(self.lbl_pkg, 2, 1, Qt.AlignRight)
        
        self.lbl_iface.setObjectName("Val"); self.lbl_lat.setObjectName("Val"); self.lbl_pkg.setObjectName("Val")

    def update_stats(self, iface, latency, pkg_count):
        self.lbl_iface.setText(iface)
        # 根据延迟变色
        c = "#00e676" if latency < 10 else ("#ffeb3b" if latency < 20 else "#ff5252")
        self.lbl_lat.setStyleSheet(f"color: {c}")
        self.lbl_lat.setText(f"{latency:.1f} ms")
        self.lbl_pkg.setText(str(pkg_count))

# --- [类 7] 健康状态矩阵 (新增) ---
class HealthMatrixWidget(QFrame):
    """健康状态矩阵：显示各模块的指示灯"""
    def __init__(self):
        super().__init__()
        self.setObjectName("MatrixBox")
        self.setStyleSheet("""
            #MatrixBox { background-color: #222; border: 1px solid #333; border-radius: 5px; }
            QLabel { font-size: 11px; color: #ccc; font-weight: bold; }
        """)
        self.indicators = {}
        layout = QGridLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        
        items = ["IMU", "L.ARM", "R.ARM", "LEGS", "COMM", "TEMP"]
        for i, name in enumerate(items):
            row = i // 2
            col = i % 2
            
            w = QWidget()
            h = QHBoxLayout(w); h.setContentsMargins(0,2,0,2)
            
            led = QLabel()
            led.setFixedSize(12, 12)
            led.setStyleSheet("background-color: #444; border-radius: 6px;")
            
            lbl = QLabel(name)
            
            h.addWidget(led); h.addWidget(lbl); h.addStretch()
            layout.addWidget(w, row, col)
            self.indicators[name] = led

    def set_status(self, name, status):
        """status: 0=Gray, 1=Green, 2=Red, 3=Yellow"""
        if name not in self.indicators: return
        colors = {0: "#444", 1: "#00e676", 2: "#ff5252", 3: "#ffea00"}
        c = colors.get(status, "#444")
        shadow = f"border: 2px solid {c};" if status != 0 else ""
        self.indicators[name].setStyleSheet(f"background-color: {c}; border-radius: 6px; {shadow}")
# --- [类 8] 手指压力热力图 (新增) ---
class FingerHeatmap(QWidget):
    def __init__(self, title="Finger"):
        super().__init__()
        self.setFixedSize(60, 120) # 设置一个合适的大小 (宽60 高120)
        self.title = title
        self.data_matrix = None # 存储 12x6 数据
        
        # 颜色映射: 0=黑, 小=蓝, 中=绿, 大=红
        self.setStyleSheet("background-color: #111; border: 1px solid #444;")

    def update_data(self, matrix_data):
        """matrix_data: 2D list (rows x cols)"""
        self.data_matrix = matrix_data
        self.update() # 触发重绘

# ui/widgets.py 中的 FingerHeatmap 类

    def paintEvent(self, event):
        painter = QPainter(self)
        # 1. 改为深灰色背景，而不是纯黑，方便区分
        painter.fillRect(self.rect(), QColor("#222")) 
        
        if not self.data_matrix:
            painter.setPen(QColor("#555"))
            painter.drawText(self.rect(), Qt.AlignCenter, "Wait...")
            return

        rows = len(self.data_matrix)
        cols = len(self.data_matrix[0])
        cell_w = self.width() / cols
        cell_h = self.height() / rows

        for r in range(rows):
            for c in range(cols):
                val = self.data_matrix[r][c]
                
                # === [强制显形逻辑] ===
                if val > 5: 
                    # 有压力：正常显示彩色
                    ratio = min(1.0, val / 150.0)
                    hue = int((1.0 - ratio) * 240 * 0.7)
                    color = QColor.fromHsv(hue, 220, 255)
                else:
                    # 无压力：显示浅灰色边框的黑块，证明这里有格子
                    color = QColor("#111") 

                # 绘制方块
                rect = QRectF(c * cell_w, r * cell_h, cell_w, cell_h)
                painter.fillRect(rect, color)
                
                # [新增] 绘制网格线 (白色细线)
                painter.setPen(QPen(QColor("#444"), 1))
                painter.drawRect(rect)

# --- [类 9] FSM 模式切换条 (新增) ---
class FsmSwitchWidget(QWidget):
    modeChanged = pyqtSignal(int) # 发送 FSM ID

    def __init__(self, title="运控模式 (FSM MODE)", active_color="#d500f9"):
        super().__init__()
        l = QVBoxLayout(self)
        l.setContentsMargins(0,0,0,0); l.setSpacing(5)
        
        # 标题样式保持一致
        l.addWidget(QLabel(title))
        
        bl = QHBoxLayout()
        self.group = QButtonGroup(self)
        self.buttons = {}
        
        # 定义模式选项: ID -> 显示名称
        modes = [
            (801, "AI 运控 (801)"),
            (500, "标准运控 (500)")
        ]
        
        for fsm_id, text in modes:
            b = QPushButton(text)
            b.setCheckable(True)
            b.setFocusPolicy(Qt.NoFocus)
            b.setFixedHeight(30)
            # 复用之前的 CSS 样式
            b.setStyleSheet(f"""
                QPushButton {{ background-color: #333; border: 1px solid #555; color: #aaa; }}
                QPushButton:checked {{ background-color: {active_color}; color: #fff; font-weight: bold; border: 1px solid {active_color}; }}
                QPushButton:hover:!checked {{ background-color: #444; }}
            """)
            
            self.group.addButton(b, fsm_id) # 这里的 id 就是 FSM ID
            self.buttons[fsm_id] = b
            bl.addWidget(b)
            
        # 连接信号
        self.group.buttonClicked[int].connect(self.modeChanged.emit)
        l.addLayout(bl)

    def set_current_mode(self, fsm_id):
        """外部设置当前选中状态"""
        if fsm_id in self.buttons:
            self.buttons[fsm_id].setChecked(True)