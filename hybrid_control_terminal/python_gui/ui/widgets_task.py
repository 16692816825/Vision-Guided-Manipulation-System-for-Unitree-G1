# ui/widgets_task.py
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QFrame, QListWidget, QListWidgetItem,
                             QAbstractItemView, QMenu, QAction)
from PyQt5.QtCore import Qt, pyqtSignal, QSize
from PyQt5.QtGui import QColor, QIcon, QDrag

# 定义任务类型常量
TASK_TYPE_MOVE = "MOVE"  # 移动并动作
TASK_TYPE_WAIT = "WAIT"  # 等待外部信号

class TaskCardWidget(QFrame):
    """
    单个任务卡片组件
    """
    # 信号：请求编辑(task_uuid), 请求删除(task_uuid), 请求模拟触发(task_uuid)
    sig_edit = pyqtSignal(str)
    sig_delete = pyqtSignal(str)
    sig_simulate = pyqtSignal(str)

    def __init__(self, task_data):
        super().__init__()
        self.task_data = task_data
        self.uuid = task_data.get('uuid', 'unknown')
        self.task_type = task_data.get('type', TASK_TYPE_MOVE)
        
        self.setObjectName("TaskCard")
        self._init_ui()
        self._update_style()

    def _init_ui(self):
        # 布局：水平 [图标 | 信息区 | 操作区]
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(10)

        # 1. 左侧状态条/图标
        self.lbl_icon = QLabel()
        self.lbl_icon.setFixedSize(30, 30)
        self.lbl_icon.setAlignment(Qt.AlignCenter)
        self.lbl_icon.setStyleSheet("font-size: 18px; border-radius: 15px; background: rgba(255,255,255,0.1);")
        layout.addWidget(self.lbl_icon)

        # 2. 中间信息区
        info_layout = QVBoxLayout()
        info_layout.setSpacing(2)
        
        # 标题 (ID 或 名称)
        self.lbl_title = QLabel(self.task_data.get('name', '未命名任务'))
        self.lbl_title.setStyleSheet("font-weight: bold; font-size: 14px; color: #fff;")
        
        # 副标题 (详情)
        self.lbl_desc = QLabel()
        self.lbl_desc.setStyleSheet("font-size: 11px; color: #aaa;")
        
        info_layout.addWidget(self.lbl_title)
        info_layout.addWidget(self.lbl_desc)
        layout.addLayout(info_layout, 1) # 占据主要空间

        # 3. 右侧状态/操作区
        self.lbl_status = QLabel("待命")
        self.lbl_status.setStyleSheet("color: #666; font-size: 10px; font-weight: bold;")
        layout.addWidget(self.lbl_status)

        # 仅在鼠标悬停时显示的操作按钮 (这里简化为常驻或根据需求)
        # 为了简洁，我们把编辑和删除放在右键菜单里，或者在右侧放一个小按钮
        # 这里演示放一个编辑按钮
        self.btn_edit = QPushButton("⚙")
        self.btn_edit.setFixedSize(24, 24)
        self.btn_edit.setCursor(Qt.PointingHandCursor)
        self.btn_edit.setStyleSheet("""
            QPushButton { background: transparent; color: #888; border: none; font-size: 14px; }
            QPushButton:hover { color: white; }
        """)
        self.btn_edit.clicked.connect(lambda: self.sig_edit.emit(self.uuid))
        layout.addWidget(self.btn_edit)
        
        # 如果是等待类型，增加一个模拟按钮
        if self.task_type == TASK_TYPE_WAIT:
            self.btn_sim = QPushButton("▶")
            self.btn_sim.setToolTip("强制通过 (模拟信号)")
            self.btn_sim.setFixedSize(24, 24)
            self.btn_sim.setCursor(Qt.PointingHandCursor)
            self.btn_sim.setStyleSheet("background: #d500f9; color: white; border-radius: 12px; font-weight: bold;")
            self.btn_sim.clicked.connect(lambda: self.sig_simulate.emit(self.uuid))
            self.btn_sim.hide() # 默认隐藏，执行时显示
            layout.addWidget(self.btn_sim)

    def _update_style(self):
        """根据任务类型设置颜色主题"""
        if self.task_type == TASK_TYPE_MOVE:
            # 蓝色/绿色系
            self.lbl_icon.setText("📍")
            border_col = "#00e5ff"
            bg_col = "#1e2a30"
            
            # 解析副标题
            pose = self.task_data.get('pose', {})
            act = self.task_data.get('action', '无动作')
            if act is None: act = "无动作"
            self.lbl_desc.setText(f"坐标: ({pose.get('x',0):.1f}, {pose.get('y',0):.1f}) | 动作: {act}")
            
        elif self.task_type == TASK_TYPE_WAIT:
            # 紫色/黄色系
            self.lbl_icon.setText("📡")
            border_col = "#d500f9"
            bg_col = "#2a1e30"
            
            # 解析副标题
            cond = self.task_data.get('condition', {})
            key = cond.get('key', '?')
            val = cond.get('val', '?')
            self.lbl_desc.setText(f"等待信号: [{key}] == '{val}'")

        # 设置 QFrame 样式
        self.setStyleSheet(f"""
            #TaskCard {{
                background-color: {bg_col};
                border: 1px solid #333;
                border-left: 4px solid {border_col};
                border-radius: 4px;
            }}
            #TaskCard:hover {{
                background-color: {bg_col}40; /* 稍微亮一点 */
                border: 1px solid {border_col};
            }}
        """)

    def set_active_state(self, state):
        """设置运行状态: IDLE, RUNNING, FINISHED"""
        if state == 'IDLE':
            self.lbl_status.setText("待命")
            self.lbl_status.setStyleSheet("color: #666;")
            if hasattr(self, 'btn_sim'): self.btn_sim.hide()
            
        elif state == 'RUNNING':
            self.lbl_status.setText("执行中...")
            self.lbl_status.setStyleSheet("color: #00e676; font-weight: bold;")
            # 运行时的背景高亮
            self.setStyleSheet(self.styleSheet().replace("border: 1px solid #333;", "border: 1px solid #00e676;"))
            if hasattr(self, 'btn_sim'): self.btn_sim.show()

        elif state == 'FINISHED':
            self.lbl_status.setText("完成")
            self.lbl_status.setStyleSheet("color: #888;")
            self.setGraphicsEffect(None) # 移除特效

class TaskFlowList(QListWidget):
    """
    支持拖拽排序的任务列表容器
    """
    # 列表顺序发生变化的信号
    sig_order_changed = pyqtSignal()
    # 转发内部卡片的信号
    sig_card_edit = pyqtSignal(str)
    sig_card_delete = pyqtSignal(str)
    sig_card_simulate = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        # 开启拖拽
        self.setDragDropMode(QAbstractItemView.InternalMove)
        self.setSelectionMode(QAbstractItemView.SingleSelection)
        self.setSpacing(5)
        # 样式
        self.setStyleSheet("""
            QListWidget {
                background-color: #121212;
                border: 1px solid #333;
                border-radius: 6px;
                padding: 5px;
            }
            QListWidget::item {
                background: transparent;
                margin-bottom: 5px;
            }
            QListWidget::item:selected {
                background: transparent; /* 选中效果交给 Card 自己处理 */
            }
        """)
        # 监听模型变化以触发排序信号
        self.model().rowsMoved.connect(self._on_rows_moved)

    def _on_rows_moved(self, parent, start, end, dest, row):
        self.sig_order_changed.emit()

    def add_task_card(self, task_data):
        """添加一个任务卡片"""
        item = QListWidgetItem()
        # 必须设置 Item 的大小，否则自定义 Widget 无法显示
        item.setSizeHint(QSize(0, 70)) 
        
        # 绑定数据到 item (方便排序后取回)
        item.setData(Qt.UserRole, task_data)
        
        self.addItem(item)
        
        # 创建卡片组件
        card = TaskCardWidget(task_data)
        # 信号转发
        card.sig_edit.connect(self.sig_card_edit)
        card.sig_simulate.connect(self.sig_card_simulate)
        
        self.setItemWidget(item, card)

    def get_all_tasks(self):
        """按照当前视觉顺序，返回所有任务数据"""
        tasks = []
        for i in range(self.count()):
            item = self.item(i)
            # 重新获取数据 (可能需要从 widget 里取最新的，如果支持在线编辑)
            # 这里简单起见，假设 Item 里的 UserRole 数据是最新的
            # 如果你在 Card 里修改了数据，记得同步更新 item.setData
            t_data = item.data(Qt.UserRole)
            tasks.append(t_data)
        return tasks

    def contextMenuEvent(self, event):
        """右键菜单：删除"""
        item = self.itemAt(event.pos())
        if item:
            menu = QMenu(self)
            action_del = QAction("🗑 删除此任务", self)
            action_del.triggered.connect(lambda: self._trigger_delete(item))
            menu.addAction(action_del)
            menu.exec_(event.globalPos())

    def _trigger_delete(self, item):
        data = item.data(Qt.UserRole)
        uuid = data.get('uuid')
        # 从列表移除
        row = self.row(item)
        self.takeItem(row)
        # 发送信号让外部处理数据层
        self.sig_card_delete.emit(uuid)

