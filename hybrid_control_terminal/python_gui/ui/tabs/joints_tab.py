# ui/tabs/joints_tab.py
import math
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QTableWidget, 
                             QTableWidgetItem, QHeaderView, QAbstractItemView)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QFont

# 引入配置和样式
from config import G1_JOINT_MAP
from ui.styles import Styles

class JointsTab(QWidget):
    """
    Tab 2: 关节数据详情页
    职责：以表格形式展示 23-DOF 关节的实时状态 (角度、速度、温度)
    """
    def __init__(self):
        super().__init__()
        # 预先排序关节 ID，保证显示顺序一致
        self.valid_joint_ids = sorted(G1_JOINT_MAP.keys())
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        
        # 创建表格
        self.tbl_joints = QTableWidget()
        self.tbl_joints.setColumnCount(5) # [ID, 名称, 位置, 速度, 温度]
        self.tbl_joints.setHorizontalHeaderLabels(["ID", "部位名称", "位置 (°)", "速度", "温度"])
        
        # 表格样式配置
        self.tbl_joints.verticalHeader().setVisible(False)
        self.tbl_joints.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tbl_joints.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents) # ID 列窄一点
        
        # 根据真实关节数量设置行数
        self.tbl_joints.setRowCount(len(self.valid_joint_ids))
        
        # 交互配置
        self.tbl_joints.setFocusPolicy(Qt.NoFocus)
        self.tbl_joints.setSelectionMode(QAbstractItemView.NoSelection)
        self.tbl_joints.setAlternatingRowColors(True) # 开启斑马纹
        
        # 应用 CSS 样式
        self.tbl_joints.setStyleSheet(Styles.JOINTS_TABLE)

        # 初始化静态内容 (ID 和 名称)
        # 这些内容在运行期间不会变，只需设置一次
        for row_idx, jid in enumerate(self.valid_joint_ids):
            # 1. ID 列
            self.tbl_joints.setItem(row_idx, 0, QTableWidgetItem(str(jid)))
            
            # 2. 名称 列 (带颜色区分)
            name_item = QTableWidgetItem(G1_JOINT_MAP[jid])
            
            # 简单的颜色区分逻辑：
            # 腿部(0-11): 绿色
            # 腰部(12): 紫色
            # 手臂(>12): 青色
            if jid <= 11: 
                name_item.setForeground(QColor("#00e676"))
            elif jid == 12: 
                name_item.setForeground(QColor("#d500f9"))
            else: 
                name_item.setForeground(QColor("#00e5ff"))
            
            self.tbl_joints.setItem(row_idx, 1, name_item)
            
            # 3. 初始化数据列占位符
            self.tbl_joints.setItem(row_idx, 2, QTableWidgetItem("-"))
            self.tbl_joints.setItem(row_idx, 3, QTableWidgetItem("-"))
            self.tbl_joints.setItem(row_idx, 4, QTableWidgetItem("N/A"))

        layout.addWidget(self.tbl_joints)

    # =============================================================
    # 数据更新接口
    # =============================================================
    def update_data(self, status_data):
        """
        接收全量状态包，更新表格数据
        """
        if not status_data: return
        
        joints_data = status_data.get('joints', {})
        if not joints_data: return

        # 遍历每一行进行更新
        for row_idx, jid in enumerate(self.valid_joint_ids):
            # 兼容 key 可能是 int 或 str 的情况
            d = joints_data.get(jid) or joints_data.get(str(jid))
            
            if d:
                # 1. 位置 (弧度 -> 角度)
                q_rad = d.get('q', 0.0)
                q_deg = math.degrees(q_rad)
                self.tbl_joints.setItem(row_idx, 2, QTableWidgetItem(f"{q_deg:.2f}"))
                
                # 2. 速度
                dq = d.get('dq', 0.0)
                self.tbl_joints.setItem(row_idx, 3, QTableWidgetItem(f"{dq:.2f}"))
                
                # 3. 温度 (带颜色报警)
                temp_val = int(d.get('temp', 0))
                t_item = QTableWidgetItem(f"{temp_val} °C")
                t_item.setTextAlignment(Qt.AlignCenter)
                
                if temp_val > 60: 
                    # 严重高温：红色加粗
                    t_item.setForeground(QColor("#ff5252"))
                    t_item.setFont(QFont("Arial", 9, QFont.Bold))
                elif temp_val > 45: 
                    # 轻微发热：黄色
                    t_item.setForeground(QColor("#ffeb3b"))
                else: 
                    # 正常：浅灰
                    t_item.setForeground(QColor("#e0e0e0"))
                
                self.tbl_joints.setItem(row_idx, 4, t_item)
            else:
                # 如果没有数据，保持占位符或设为 N/A
                pass
