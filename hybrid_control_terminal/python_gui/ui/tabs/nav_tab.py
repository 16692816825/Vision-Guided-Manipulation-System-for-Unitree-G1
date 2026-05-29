# ui/tabs/nav_tab.py

import os
import pathlib
import datetime
import math
import glob
import json

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QFrame, QStackedWidget, QFileDialog, 
                             QInputDialog, QMessageBox, QGroupBox, QFormLayout, 
                             QLineEdit, QComboBox, QTableWidget, QTableWidgetItem, 
                             QHeaderView, QAbstractItemView, QSplitter, QGridLayout,
                             QCheckBox) # <--- 确保这里有 QGridLayout

from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QColor, QIcon

from ui.styles import Styles
from ui.nav_widget import NavMapWidget
from core.task_manager import task_manager
from config import DATA_LOG_DIR

import uuid
from ui.widgets_task import TaskFlowList, TASK_TYPE_MOVE, TASK_TYPE_WAIT
class NavTab(QWidget):
    """
    智能导航 Tab 页
    职责：管理导航UI状态机 (IDLE/MAPPING/RELOC/NAV)、处理地图控件交互
    """
    
    # === 定义对外信号 (解耦关键) ===
    # 请求后端执行具体任务的信号
    sig_start_mapping = pyqtSignal()
    sig_save_map = pyqtSignal(str)      # 参数: 完整文件路径(无后缀)
    sig_start_nav = pyqtSignal(str)     # 参数: 地图名(无后缀)
    sig_stop_system = pyqtSignal()
    
    # 导航相关指令信号 (转发给 Worker)
    sig_pub_goal = pyqtSignal(float, float, float) # x, y, yaw
    sig_pub_pose = pyqtSignal(float, float, float) # x, y, yaw
    # [新增] 请求测试动作信号 (参数: 动作文件名)
    sig_play_action = pyqtSignal(str)
    sig_req_task_exec = pyqtSignal(dict) # 参数: 任务数据字典
    def __init__(self):
        super().__init__()
        self.current_nav_state = 'IDLE'
        self.current_fsm_id = -1  # [新增] 用于存储当前运控状态ID
        self._init_ui()
        self._connect_signals()
        self._load_tasks_to_ui()
    def _init_ui(self):
        """初始化布局"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # 1. 顶部状态栏 (保持不变)
        header = QFrame()
        header.setFixedHeight(50)
        hl = QHBoxLayout(header)
        header.setStyleSheet("background: #252526; border-bottom: 1px solid #333;")
        
        self.lbl_nav_state = QLabel("当前状态: 待机 (IDLE)")
        self.lbl_nav_state.setStyleSheet("font-size: 14px; font-weight: bold; color: #aaa;")
        hl.addWidget(self.lbl_nav_state)
        hl.addStretch()
        
        # 紧急停止按钮
        self.btn_stop = QPushButton("🛑 停止系统 / 重置")
        self.btn_stop.setFixedSize(140, 36)
        self.btn_stop.setCursor(Qt.PointingHandCursor)
        self.btn_stop.setStyleSheet("""
            QPushButton { background-color: #b71c1c; color: white; font-weight: bold; border-radius: 4px; border: 1px solid #d32f2f; }
            QPushButton:hover { background-color: #d32f2f; }
        """)
        self.btn_stop.clicked.connect(self._on_stop_clicked)
        hl.addWidget(self.btn_stop)
        
        layout.addWidget(header)

        # 2. 中间操作引导区 (堆叠窗口) - 移除原有的 Page 4
        self.nav_stack = QStackedWidget()
        self.nav_stack.setFixedHeight(60) # 高度减小，节省空间给地图
        
        # --- Page 0: 首页 ---
        p0 = QWidget(); l0 = QHBoxLayout(p0); l0.setContentsMargins(0,5,0,5)
        btn_goto_map = QPushButton("🛠️ 新建地图")
        btn_goto_map.setCursor(Qt.PointingHandCursor); btn_goto_map.setStyleSheet(Styles.NAV_BTN_MAP)
        btn_goto_map.clicked.connect(self._on_btn_start_mapping_clicked)
        
        btn_goto_nav = QPushButton("🚀 启动导航")
        btn_goto_nav.setCursor(Qt.PointingHandCursor); btn_goto_nav.setStyleSheet(Styles.NAV_BTN_RUN)
        btn_goto_nav.clicked.connect(self._on_btn_start_nav_clicked)
        
        l0.addStretch(); l0.addWidget(btn_goto_map); l0.addSpacing(20); l0.addWidget(btn_goto_nav); l0.addStretch()
        
        # --- Page 1: 建图中 ---
        p1 = QWidget(); l1 = QHBoxLayout(p1); l1.setContentsMargins(10,5,10,5)
        lbl_mapping = QLabel("正在建图... (键盘控制移动)")
        lbl_mapping.setStyleSheet("color: #00e676; font-weight: bold;")
        btn_save_map = QPushButton("💾 保存地图"); btn_save_map.clicked.connect(self._on_btn_save_map_clicked)
        l1.addWidget(lbl_mapping); l1.addStretch(); l1.addWidget(btn_save_map)

        # --- Page 2: 重定位 ---
        p2 = QWidget(); l2 = QHBoxLayout(p2); l2.setContentsMargins(10,5,10,5)
        lbl_reloc = QLabel("重定位: 按住 Shift + 拖拽箭头对齐")
        lbl_reloc.setStyleSheet("color: #d500f9; font-weight: bold;")
        btn_confirm_reloc = QPushButton("✅ 完成对齐"); btn_confirm_reloc.clicked.connect(lambda: self.switch_state('NAV'))
        l2.addWidget(lbl_reloc); l2.addStretch(); l2.addWidget(btn_confirm_reloc)

        # --- Page 3: 导航中 ---
        p3 = QWidget(); l3 = QHBoxLayout(p3); l3.setContentsMargins(10,5,10,5)
        lbl_naving = QLabel("导航就绪: Ctrl + 拖拽设置目标 | 键盘微调位置")
        lbl_naving.setStyleSheet("color: #2979ff; font-weight: bold;")
        btn_re_reloc = QPushButton("📍 重新定位"); btn_re_reloc.clicked.connect(lambda: self.switch_state('RELOC'))
        l3.addWidget(lbl_naving); l3.addStretch(); l3.addWidget(btn_re_reloc)

        self.nav_stack.addWidget(p0); self.nav_stack.addWidget(p1)
        self.nav_stack.addWidget(p2); self.nav_stack.addWidget(p3)
        layout.addWidget(self.nav_stack)

        # 3. 主体区域 (上下分割: 地图 + 任务面板)
        splitter = QSplitter(Qt.Vertical)
        splitter.setHandleWidth(1)
        splitter.setStyleSheet("QSplitter::handle { background-color: #444; }")

        # 上半部分: 地图
        self.nav_view = NavMapWidget()
        splitter.addWidget(self.nav_view)

        # 下半部分: 任务管理面板 (新增)
        self.task_panel = QWidget()
        self._init_task_panel(self.task_panel)
        splitter.addWidget(self.task_panel)
        
        # 设置初始比例 (地图 : 面板 = 3 : 1)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)

        layout.addWidget(splitter)

        # 初始状态
        self.switch_state('IDLE')

    def _connect_signals(self):
        """连接内部信号"""
        # 将地图控件的信号转发出去，或者在 Tab 内部处理
        self.nav_view.set_goal_signal.connect(self.sig_pub_goal)
        self.nav_view.set_pose_signal.connect(self.sig_pub_pose)
        
    # =========================================================
    # 状态机逻辑
    # =========================================================
    def switch_state(self, state):
        """切换导航界面状态"""
        self.current_nav_state = state
        
        if state == 'IDLE':
            self.nav_stack.setCurrentIndex(0)
            self.lbl_nav_state.setText("当前状态: 待机 (IDLE)")
            self.lbl_nav_state.setStyleSheet("color: #aaa; font-weight: bold;")
            self.nav_view.set_input_mode('VIEW') 
            self.nav_view.clear_map() # 状态隔离
            
        elif state == 'MAPPING':
            self.nav_stack.setCurrentIndex(1)
            self.lbl_nav_state.setText("当前状态: 建图中 (MAPPING)")
            self.lbl_nav_state.setStyleSheet("color: #e65100; font-weight: bold;")
            self.nav_view.set_input_mode('VIEW') 
            self.nav_view.clear_map()
            
        elif state == 'RELOC':
            self.nav_stack.setCurrentIndex(2)
            self.lbl_nav_state.setText("当前状态: 等待重定位 (RELOC)")
            self.lbl_nav_state.setStyleSheet("color: #d500f9; font-weight: bold;")
            self.nav_view.set_input_mode('RELOC') 
            
        elif state == 'NAV':
            self.nav_stack.setCurrentIndex(3)
            self.lbl_nav_state.setText("当前状态: 导航就绪 (NAV READY)")
            self.lbl_nav_state.setStyleSheet("color: #2979ff; font-weight: bold;")
            self.nav_view.set_input_mode('NAV') 
            self.nav_view.set_task_list(task_manager.get_all_tasks())
        # [新增] 任务编辑模式
        elif state == 'TASK_EDIT':
            self.nav_stack.setCurrentIndex(4)
            self.lbl_nav_state.setText("当前状态: 任务点编辑 (TASK EDIT)")
            self.lbl_nav_state.setStyleSheet("color: #fbc02d; font-weight: bold;")
            
            # 设置地图为 NAV 模式，允许用户点击地图让机器人走过去
            self.nav_view.set_input_mode('NAV')
            
            # 刷新动作文件列表
            self._refresh_action_files()
            
            # 刷新地图上的任务图标
            self.nav_view.set_task_list(task_manager.get_all_tasks())

    # =========================================================
    # 按钮事件处理 (内部逻辑 -> 发送信号)
    # =========================================================
    def _on_stop_clicked(self):
        """点击停止按钮"""
        self.nav_view.clear_map()
        self.sig_stop_system.emit()
        self.switch_state('IDLE')

    def _on_btn_start_mapping_clicked(self):
        """点击开始建图"""
        if not self._check_fsm_ready("建图"):
            return

        # 2. 弹窗确认
        reply = QMessageBox.question(self, "启动建图", 
                                     "即将启动激光雷达建图 (Mapping) 模式。\n\n"
                                     "1. 请确保机器人周围场地开阔。\n"
                                     "2. 建图过程中可以使用键盘方向键控制移动。\n"
                                     "3. 建图完成后请点击【保存地图】。\n\n"
                                     "是否继续？",
                                     QMessageBox.Yes | QMessageBox.No)
        
        if reply != QMessageBox.Yes:
            return
        # ==========================================

        self.nav_view.clear_map()
        self.sig_start_mapping.emit()
        self.switch_state('MAPPING')

    def _on_btn_save_map_clicked(self):
        """点击保存地图"""
        default_name = f"map_{datetime.datetime.now().strftime('%m%d_%H%M')}"
        name, ok = QInputDialog.getText(self, "保存地图", "输入名称 (无需后缀):", QLineEdit.Normal, default_name)
        
        if ok and name:
            # 自动补全路径 logic
            maps_dir = os.path.join(os.getcwd(), "maps")
            if not os.path.exists(maps_dir):
                os.makedirs(maps_dir)
            full_path_base = os.path.join(maps_dir, name)
            
            # 发送信号
            self.sig_save_map.emit(full_path_base)

    def _on_btn_start_nav_clicked(self):
        """点击启动导航"""
        if not self._check_fsm_ready("导航"):
            return

        map_dir = str(pathlib.Path("maps").absolute())
        f, _ = QFileDialog.getOpenFileName(self, "选择地图文件", map_dir, "YAML Config (*.yaml)")
        if not f: return
        
        map_name = pathlib.Path(f).stem
        
        # 启动后端
        self.sig_start_nav.emit(map_name)
        
        # 切换界面到重定位
        self.nav_view.clear_map()
        self.switch_state('RELOC')

    def _check_fsm_ready(self, action_name):
        """检查运控模式是否满足建图/导航要求"""
        if self.current_fsm_id != 500:
            QMessageBox.warning(
                self,
                "模式错误",
                f"🚫 无法启动{action_name}！\n\n"
                f"当前运控模式: FSM {self.current_fsm_id}\n"
                f"{action_name}功能要求必须处于 [标准运控模式 (500)]。\n\n"
                "请先在左侧面板将【运控模式】切换为 500，再重试。"
            )
            return False
        return True

    # =========================================================
    # 数据更新接口 (供 MainWindow 调用)
    # =========================================================
    def update_data(self, nav_data):
        """
        接收来自 Worker 的导航数据包
        """
        if not nav_data: return

        # 1. 更新地图和机器人位置
        scan_data = nav_data.get('scan')
        if 'robot_pose' in nav_data:
            self.nav_view.update_data(
                nav_data.get('map'), 
                nav_data.get('map_info'), 
                nav_data.get('robot_pose'), 
                nav_data.get('global_path'), 
                nav_data.get('local_path'),
                scan_data
            )
            
            # 2. 实时更新底部面板的坐标
            rx, ry, ryaw = nav_data['robot_pose']
            # 注意：self.lbl_realtime_pose 是在 _init_task_panel 中初始化的
            if hasattr(self, 'lbl_realtime_pose'):
                self.lbl_realtime_pose.setText(f"X:{rx:.2f} Y:{ry:.2f} Yaw:{math.degrees(ryaw):.0f}°")

            # ==========================================
            # [这里插入你刚才的代码] 更新控制源状态指示
            # ==========================================
            if hasattr(self, 'lbl_control_source'):
                mode = nav_data.get('control_mode', 'WALKING')
                
                if mode == 'NAVIGATING':
                    self.lbl_control_source.setText("CTRL: 自动导航中 🟢")
                    self.lbl_control_source.setStyleSheet("color: #00e676; font-weight: bold;")
                elif mode == 'WALKING':
                    self.lbl_control_source.setText("CTRL: 手动/微调 🔵")
                    self.lbl_control_source.setStyleSheet("color: #2979ff; font-weight: bold;")
                else:
                    # 其他状态（如原地站立、示教等）
                    self.lbl_control_source.setText(f"CTRL: {mode}")
                    self.lbl_control_source.setStyleSheet("color: #aaa; font-weight: bold;")

    def handle_saved_and_stopped(self):
        """当后端发来 NAV_SAVED_AND_STOPPED 信号时调用"""
        QMessageBox.information(self, "系统提示", "地图保存成功！\n系统已自动停止，请重新选择功能。")
        self.switch_state('IDLE')

    def handle_stopped(self):
        """当后端发来 NAV_STOPPED 信号时调用"""
        if self.current_nav_state != 'IDLE':
            self.switch_state('IDLE')
    def set_current_fsm(self, fsm_id):
        """[新增] 接收当前 FSM ID"""
        self.current_fsm_id = fsm_id
    def _refresh_task_table(self):
        """从 TaskManager 读取数据并刷新表格"""
        tasks = task_manager.get_all_tasks()
        self.tbl_tasks.setRowCount(len(tasks))
        
        for i, t in enumerate(tasks):
            pose = t.get('pose', {})
            # [修改] 显示 X, Y, Yaw (两位小数)
            x = pose.get('x', 0)
            y = pose.get('y', 0)
            yaw_deg = math.degrees(pose.get('yaw', 0))
            pose_str = f"({x:.2f}, {y:.2f}, {yaw_deg:.2f}°)"
            
            self.tbl_tasks.setItem(i, 0, QTableWidgetItem(t.get('trigger_id', '')))
            self.tbl_tasks.setItem(i, 1, QTableWidgetItem(t.get('action_file') or "None"))
            self.tbl_tasks.setItem(i, 2, QTableWidgetItem(pose_str))
            self.tbl_tasks.setItem(i, 3, QTableWidgetItem(t.get('description', '')))

    def _on_table_item_clicked(self, item):
        """点击表格行，填充编辑器"""
        row = item.row()
        tid = self.tbl_tasks.item(row, 0).text()
        task = task_manager.get_task(tid)
        if task:
            self.inp_task_id.setText(task['trigger_id'])
            
            action = task.get('action_file')
            idx = self.combo_task_action.findText(action) if action else 0
            if idx >= 0: self.combo_task_action.setCurrentIndex(idx)
            
            # TODO: 可以让地图高亮这个点 (需要 NavMapWidget 支持)

    def _refresh_action_files(self):
        """刷新下拉框"""
        self.combo_task_action.clear()
        self.combo_task_action.addItem("无动作 (None)", None)
        files = glob.glob(str(DATA_LOG_DIR / "*.json"))
        for f in sorted(files):
            fname = pathlib.Path(f).name
            if fname == "tasks.json": continue
            self.combo_task_action.addItem(fname, fname)

    def _on_save_task_clicked(self):
        tid = self.inp_task_id.text().strip()
        if not tid:
            QMessageBox.warning(self, "错误", "请输入 Trigger ID (唯一标识)")
            return
            
        # [新增] 检查 ID 是否已存在
        existing_task = task_manager.get_task(tid)
        if existing_task:
            reply = QMessageBox.question(
                self, "覆盖确认", 
                f"ID [{tid}] 已存在！\n是否覆盖原有数据？\n\n(如果要新增点位，请修改 ID)",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.No:
                return

        rx, ry, ryaw = self.nav_view.robot_pose
        action = self.combo_task_action.currentData()
        pose = {'x': rx, 'y': ry, 'yaw': ryaw}
        
        if task_manager.add_task(tid, pose, action, description="UI Edited"):
            self._refresh_task_table()
            self.nav_view.set_task_list(task_manager.get_all_tasks())
            QMessageBox.information(self, "成功", f"点位 [{tid}] 已保存")
        else:
            QMessageBox.critical(self, "错误", "保存失败，请检查日志。")

    def _on_del_task_clicked(self):
        tid = self.inp_task_id.text().strip()
        if not tid: return
        task_manager.remove_task(tid)
        self._refresh_task_table()
        self.nav_view.set_task_list(task_manager.get_all_tasks())

    def _on_test_action_clicked(self):
        action = self.combo_task_action.currentData()
        if not action: 
            QMessageBox.warning(self, "提示", "当前没有绑定动作文件")
            return
        
        reply = QMessageBox.question(
            self, "动作测试", 
            f"即将原地播放动作: [{action}]\n\n"
            "注意：这不会移动底盘，仅测试机械臂动作。\n是否继续？",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # 发送信号给 Main Window 处理
            self.sig_play_action.emit(action)

    def _on_resume_nav_clicked(self):
        """[新增] 恢复导航"""
        # 逻辑：重新发送当前的目标点给后端，触发状态机切回 NAVIGATING
        # 我们可以读取地图上当前的 Goal
        if self.nav_view.nav_goal:
            gx, gy = self.nav_view.nav_goal
            # 这里的 yaw 暂时不知道，发 0 也可以，或者上次记录的
            self.sig_pub_goal.emit(gx, gy, 0.0)
        else:
            QMessageBox.warning(self, "提示", "当前没有设置导航目标")

    # 替换 ui/tabs/nav_tab.py 中的 _init_task_panel 方法

    def _init_task_panel(self, parent_widget):
        """初始化底部的任务管理面板 (重构版 - 卡片流)"""
        layout = QHBoxLayout(parent_widget)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(10)

        # === 左侧: 任务流列表 (Task Flow) ===
        self.task_list_widget = TaskFlowList()
        # 连接信号
        self.task_list_widget.sig_card_edit.connect(self._on_card_edit)
        self.task_list_widget.sig_card_delete.connect(self._on_card_delete)
        self.task_list_widget.sig_order_changed.connect(self._on_list_reordered)
        
        layout.addWidget(self.task_list_widget, 2) # 占据 2/3 宽度

        # === 右侧: 编辑与控制区 ===
        # === 右侧: 编辑与控制区 ===
        edit_frame = QGroupBox("任务编辑器")
        
        # [修改] 更新样式表，修复 ComboBox 下拉列表看不见的问题
        edit_frame.setStyleSheet("""
            QGroupBox { 
                border: 1px solid #444; 
                border-radius: 4px; 
                margin-top: 8px; 
                font-weight: bold; 
            }
            QGroupBox::title { 
                subcontrol-origin: margin; 
                left: 10px; 
                color: #00e5ff; 
            }
            QLabel { 
                font-size: 12px; 
                color: #ccc; 
            }
            
            /* 输入框和下拉框的基础样式 */
            QLineEdit, QComboBox { 
                background: #333; 
                color: white; 
                border: 1px solid #555; 
                padding: 4px; 
                border-radius: 3px;
            }
            
            /* [核心修复] 下拉列表的样式 */
            QComboBox QAbstractItemView {
                background-color: #333;  /* 列表背景设为深色 */
                color: white;            /* 列表文字设为白色 */
                selection-background-color: #00e5ff; /* 选中项背景色 */
                selection-color: black;              /* 选中项文字色 */
                border: 1px solid #555;
            }
        """)
        
        edit_layout = QVBoxLayout(edit_frame)
        edit_layout.setSpacing(8)

        # 1. 任务 Tab 页 (普通点位 / 等待信号)
        row_type = QHBoxLayout()
        row_type.addWidget(QLabel("任务类型:"))
        self.combo_task_type = QComboBox()
        self.combo_task_type.addItems(["📍 导航移动 (MOVE)", "📡 等待信号 (WAIT)"])
        self.combo_task_type.currentIndexChanged.connect(self._on_task_type_changed)
        row_type.addWidget(self.combo_task_type)
        edit_layout.addLayout(row_type)

        # 2. 通用字段: ID/名称
        row_id = QHBoxLayout()
        row_id.addWidget(QLabel("任务名称:"))
        self.inp_task_name = QLineEdit()
        self.inp_task_name.setPlaceholderText("例如: Station_A")
        row_id.addWidget(self.inp_task_name)
        edit_layout.addLayout(row_id)

        # 3. 动态内容区 (Stack)
        self.stack_editor = QStackedWidget()
        
        # [Page 0: 移动参数]
        p0 = QWidget(); l0 = QVBoxLayout(p0); l0.setContentsMargins(0,0,0,0)
        
        l0.addWidget(QLabel("绑定动作:"))
        # [修复] 变量名改回 combo_task_action，与旧逻辑兼容
        self.combo_task_action = QComboBox() 
        l0.addWidget(self.combo_task_action)
        # === [新增] 复选框：使用当前机器人位置 ===
        self.chk_use_robot_pose = QCheckBox("📍 使用当前机器人位姿")
        self.chk_use_robot_pose.setStyleSheet("color: #00e676; font-weight: bold;")
        self.chk_use_robot_pose.setToolTip("勾选后，添加任务时将记录机器人此时此刻的位置和朝向，\n而不是地图上鼠标点击的位置。")
        l0.addWidget(self.chk_use_robot_pose)
        # ======================================
        self.lbl_pose_info = QLabel("坐标: (请在地图点击)")
        self.lbl_pose_info.setStyleSheet("color: #aaa; font-size: 11px;")
        l0.addWidget(self.lbl_pose_info)
        
        self.stack_editor.addWidget(p0)

        # [Page 1: 信号参数]
        p1 = QWidget(); l1 = QVBoxLayout(p1); l1.setContentsMargins(0,0,0,0)
        
        row_k = QHBoxLayout()
        row_k.addWidget(QLabel("信号 Key:"))
        self.inp_sig_key = QLineEdit("car_status")
        row_k.addWidget(self.inp_sig_key)
        l1.addLayout(row_k)
        
        row_v = QHBoxLayout()
        row_v.addWidget(QLabel("期望 Value:"))
        self.inp_sig_val = QLineEdit("arrived")
        row_v.addWidget(self.inp_sig_val)
        l1.addLayout(row_v)
        
        l1.addWidget(QLabel("提示: 只有收到该 JSON 信号才会继续"))
        
        self.stack_editor.addWidget(p1)
        edit_layout.addWidget(self.stack_editor)

        # 4. 底部按钮
        grid_btn = QGridLayout()
        
        btn_add = QPushButton("➕ 添加到列表尾部")
        btn_add.setStyleSheet("background-color: #2e7d32; color: white;")
        btn_add.clicked.connect(self._on_add_task_clicked)
        
        btn_update = QPushButton("💾 更新当前选中")
        btn_update.setStyleSheet("background-color: #00838f; color: white;")
        btn_update.clicked.connect(self._on_update_task_clicked)

        # 全局控制
        btn_exec = QPushButton("🚀 执行全流程")
        btn_exec.setStyleSheet("background-color: #6200ea; color: white; font-weight: bold; margin-top: 10px;")
        btn_exec.clicked.connect(self._on_exec_all_clicked)

        grid_btn.addWidget(btn_add, 0, 0)
        grid_btn.addWidget(btn_update, 0, 1)
        grid_btn.addWidget(btn_exec, 1, 0, 1, 2)
        
        edit_layout.addLayout(grid_btn)
        edit_layout.addStretch()

        layout.addWidget(edit_frame, 1)
        
        # 初始化动作列表 (现在 self.combo_task_action 存在了，调用不会报错)
        self._refresh_action_files()

    # 同时修正 _on_add_task_clicked 方法中的引用
    # 替换 ui/tabs/nav_tab.py 中的 _on_add_task_clicked

    def _on_add_task_clicked(self):
        """添加任务到列表 (支持 地图选点 或 机器人当前位姿)"""
        # 1. 获取基本信息
        task_type = TASK_TYPE_MOVE if self.combo_task_type.currentIndex() == 0 else TASK_TYPE_WAIT
        name = self.inp_task_name.text().strip() or "未命名"
        
        # 2. 初始化 new_task 字典 (防止 NameError)
        new_task = {
            "uuid": str(uuid.uuid4())[:8], 
            "type": task_type,
            "name": name
        }
        
        # --- 分支 1: 移动任务 ---
        if task_type == TASK_TYPE_MOVE:
            gx, gy, gyaw = 0.0, 0.0, 0.0
            
            # [关键调试] 打印一下复选框的状态，看看程序是否识别到了
            is_using_robot_pose = self.chk_use_robot_pose.isChecked()
            print(f"[UI] 复选框状态: {is_using_robot_pose}")

            # === 核心逻辑分支 ===
            if is_using_robot_pose:
                # [情况 A] 勾选了复选框 -> 读取机器人位置
                if hasattr(self.nav_view, 'robot_pose') and self.nav_view.robot_pose:
                    gx, gy, gyaw = self.nav_view.robot_pose
                    print(f"[UI] 捕获机器人位姿: ({gx:.2f}, {gy:.2f})")
                else:
                    QMessageBox.warning(self, "错误", "无法获取机器人当前位置！\n请确保导航数据已更新。")
                    return
            else:
                # [情况 B] 没勾选 -> 必须在地图上手动选了点
                # 注意：这里必须是 else，绝对不能让下面的代码在勾选时也执行
                if not self.nav_view.nav_goal:
                    QMessageBox.warning(self, "提示", "您未勾选“使用当前位姿”，\n请先在地图上按 Ctrl+拖拽 选择一个目标点。")
                    return
                
                gx, gy = self.nav_view.nav_goal
                gyaw = 0.0 
                
                # 清除地图上的临时十字标
                self.nav_view.nav_goal = None
                self.nav_view.update()
            
            # 3. 写入坐标数据
            new_task['pose'] = {'x': gx, 'y': gy, 'yaw': gyaw} 
            new_task['action'] = self.combo_task_action.currentData()
            
        # --- 分支 2: 等待任务 ---
        elif task_type == TASK_TYPE_WAIT:
            new_task['condition'] = {
                'key': self.inp_sig_key.text().strip(),
                'val': self.inp_sig_val.text().strip()
            }
            
        # 4. 添加到 UI 卡片流
        self.task_list_widget.add_task_card(new_task)
        
        # 5. 保存并刷新地图上的任务图标
        self._save_ui_to_manager()

    def _on_task_type_changed(self, idx):
        # 0 -> MOVE, 1 -> WAIT
        self.stack_editor.setCurrentIndex(idx)

    
        
    def _on_list_reordered(self):
        print("列表顺序已改变，准备同步数据...")
        # new_list = self.task_list_widget.get_all_tasks()
        # task_manager.save_ordered_list(new_list) 

    # 占位函数，防止报错
    def _on_card_edit(self, uuid): print(f"Edit {uuid}")
    def _on_card_delete(self, uuid): print(f"Delete {uuid}")
    def _on_update_task_clicked(self): pass
    def _on_exec_all_clicked(self): pass

    def _load_tasks_to_ui(self):
        """从 TaskManager 读取数据并填充到列表"""
        self.task_list_widget.clear() # 清空 UI
        tasks = task_manager.get_all_tasks()
        
        for t in tasks:
            self.task_list_widget.add_task_card(t)

  # 替换 ui/tabs/nav_tab.py 中的这两个方法

    def _save_ui_to_manager(self):
        """把 UI 上的列表顺序保存到 Manager，并刷新地图"""
        tasks = self.task_list_widget.get_all_tasks()
        task_manager.save_ordered_list(tasks)
        
        # [关键修复] 这里的 tasks 是最新的列表，同步给地图组件进行重绘
        self.nav_view.set_task_list(tasks)

    

    # 实现列表重排的回调
    def _on_list_reordered(self):
        # 当用户拖拽排序后，自动保存
        self._save_ui_to_manager()
        
    # 实现卡片删除的回调 (需要在 _init_task_panel 里连接信号)
    def _on_card_delete(self, uuid):
        # UI 控件 TaskFlowList 已经删除了 Item，这里只需要同步保存文件
        self._save_ui_to_manager()
    def _on_exec_all_clicked(self):
        """发送整个任务链给后台执行"""
        tasks = self.task_list_widget.get_all_tasks()
        if not tasks:
            QMessageBox.warning(self, "提示", "任务列表为空")
            return
            
        # 再次保存确保一致
        self._save_ui_to_manager()
        
        reply = QMessageBox.question(
            self, "确认执行", 
            f"即将按顺序执行 {len(tasks)} 个任务。\n\n"
            "包含移动、动作及信号等待。\n是否开始？",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # 发送新指令 CMD_EXEC_TASK_CHAIN
            # 这是一个新指令，我们需要在 Worker 和 RobotProcess 里处理它
            # 由于这只是给 Worker 发信号，我们这里直接构造数据
            self.sig_req_task_exec.emit({'type': 'CHAIN', 'tasks': tasks})
