import sys
import os
import subprocess
import socket
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QFormLayout, 
                             QLineEdit, QComboBox, QCheckBox, QPushButton, 
                             QLabel, QFileDialog, QGroupBox, QMessageBox, QHBoxLayout)
from PyQt5.QtCore import Qt
from core.settings_manager import settings

class Launcher(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("G1 控制终端启动器")
        self.resize(500, 600)
        self.setStyleSheet("""
            QWidget { background-color: #2d2d2d; color: #eee; font-size: 14px; }
            QLineEdit, QComboBox { padding: 5px; background: #444; border: 1px solid #555; border-radius: 4px; color: #fff; }
            QGroupBox { border: 1px solid #555; border-radius: 5px; margin-top: 10px; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; color: #00e5ff; }
            QPushButton { padding: 8px; border-radius: 5px; font-weight: bold; }
        """)
        
        self.layout = QVBoxLayout(self)
        self.layout.setSpacing(15)
        
        # 标题
        title = QLabel("G1 Hybrid Control Configuration")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 18px; color: #00e5ff; font-weight: bold; margin-bottom: 10px;")
        self.layout.addWidget(title)

        # 1. 网络配置
        self.create_network_group()
        
        # 2. 机器人配置
        self.create_robot_group()
        
        # 3. ROS 配置
        self.create_ros_group()

        # 4. AI & 校准
        self.create_misc_group()

        self.layout.addStretch()

        # 底部按钮
        btn_layout = QHBoxLayout()
        
        self.btn_save = QPushButton("💾 保存配置")
        self.btn_save.setStyleSheet("background-color: #2e7d32; color: white;")
        self.btn_save.clicked.connect(self.save_settings)
        
        self.btn_launch = QPushButton("🚀 启动终端 (Launch)")
        self.btn_launch.setStyleSheet("background-color: #1565c0; color: white; font-size: 16px;")
        self.btn_launch.clicked.connect(self.launch_app)
        
        btn_layout.addWidget(self.btn_save)
        btn_layout.addWidget(self.btn_launch)
        self.layout.addLayout(btn_layout)
        
        # 加载当前值
        self.load_current_values()

    def create_network_group(self):
        g = QGroupBox("网络连接 (Network)")
        fl = QFormLayout(g)
        
        # 自动扫描网卡
        self.combo_iface = QComboBox()
        interfaces = [i[1] for i in socket.if_nameindex()]
        self.combo_iface.addItems(interfaces)
        
        self.inp_ip = QLineEdit()
        self.inp_port = QLineEdit()
        self.inp_domain = QLineEdit()
        
        fl.addRow("网卡接口:", self.combo_iface)
        fl.addRow("机器人 IP:", self.inp_ip)
        fl.addRow("L10 端口:", self.inp_port)
        fl.addRow("DDS Domain:", self.inp_domain)
        self.layout.addWidget(g)

    def create_robot_group(self):
        g = QGroupBox("机器人设置 (Robot)")
        fl = QFormLayout(g)
        
        self.inp_model_path = QLineEdit()
        btn_browse = QPushButton("...")
        btn_browse.setFixedWidth(30)
        btn_browse.clicked.connect(lambda: self.browse_file(self.inp_model_path, "MuJoCo XML (*.xml)"))
        
        h = QHBoxLayout(); h.addWidget(self.inp_model_path); h.addWidget(btn_browse)
        fl.addRow("XML 模型路径:", h)
        
        self.inp_policy_path = QLineEdit()
        btn_browse2 = QPushButton("...")
        btn_browse2.setFixedWidth(30)
        btn_browse2.clicked.connect(lambda: self.browse_file(self.inp_policy_path, "RL Model (*.zip)"))
        
        h2 = QHBoxLayout(); h2.addWidget(self.inp_policy_path); h2.addWidget(btn_browse2)
        fl.addRow("RL 策略路径:", h2)
        
        self.chk_right_arm = QCheckBox("使用右臂 (Use Right Arm)")
        self.chk_sim_only = QCheckBox("仅仿真模式 (Sim Only - No Robot)")
        
        fl.addRow("", self.chk_right_arm)
        fl.addRow("", self.chk_sim_only)
        self.layout.addWidget(g)

    def create_ros_group(self):
        g = QGroupBox("ROS 导航 (Optional)")
        fl = QFormLayout(g)
        
        self.inp_setup_bash = QLineEdit()
        btn_browse = QPushButton("...")
        btn_browse.setFixedWidth(30)
        btn_browse.clicked.connect(lambda: self.browse_file(self.inp_setup_bash, "Bash Script (*.bash)"))
        
        h = QHBoxLayout(); h.addWidget(self.inp_setup_bash); h.addWidget(btn_browse)
        fl.addRow("Setup.bash:", h)
        self.layout.addWidget(g)

    def create_misc_group(self):
        g = QGroupBox("高级设置")
        fl = QFormLayout(g)
        
        self.inp_api_key = QLineEdit()
        self.inp_api_key.setEchoMode(QLineEdit.Password)
        self.inp_api_key.setPlaceholderText("sk-...")
        
        fl.addRow("DeepSeek Key:", self.inp_api_key)
        
        # 简单的校准
        h_calib = QHBoxLayout()
        self.inp_cam_pitch = QLineEdit()
        self.inp_cam_pitch.setPlaceholderText("Deg")
        self.inp_cam_z = QLineEdit()
        self.inp_cam_z.setPlaceholderText("Z Offset")
        
        h_calib.addWidget(QLabel("Cam Pitch:"))
        h_calib.addWidget(self.inp_cam_pitch)
        h_calib.addWidget(QLabel("Cam Z:"))
        h_calib.addWidget(self.inp_cam_z)
        
        fl.addRow("视觉校准:", h_calib)
        self.layout.addWidget(g)

    def browse_file(self, line_edit, filter):
        fname, _ = QFileDialog.getOpenFileName(self, "选择文件", ".", filter)
        if fname:
            # 尝试转为相对路径
            try:
                rel = os.path.relpath(fname, os.getcwd())
                line_edit.setText(rel)
            except:
                line_edit.setText(fname)

    def load_current_values(self):
        # Network
        self.combo_iface.setCurrentText(settings.get("network", "interface"))
        self.inp_ip.setText(settings.get("network", "robot_ip"))
        self.inp_port.setText(str(settings.get("network", "local_port")))
        self.inp_domain.setText(str(settings.get("network", "dds_domain")))
        
        # Robot
        self.inp_model_path.setText(settings.get("robot", "model_xml_path"))
        self.inp_policy_path.setText(settings.get("robot", "rl_policy_path"))
        self.chk_right_arm.setChecked(settings.get("robot", "use_right_arm"))
        self.chk_sim_only.setChecked(settings.get("control", "sim_only"))
        
        # ROS
        self.inp_setup_bash.setText(settings.get("ros", "setup_path"))
        
        # Misc
        self.inp_api_key.setText(settings.get("ai", "deepseek_api_key"))
        self.inp_cam_pitch.setText(str(settings.get("calibration", "cam_pitch_deg")))
        self.inp_cam_z.setText(str(settings.get("calibration", "cam_offset_z")))

    def save_settings(self):
        # Network
        settings.set("network", "interface", self.combo_iface.currentText())
        settings.set("network", "robot_ip", self.inp_ip.text())
        settings.set("network", "local_port", int(self.inp_port.text()))
        settings.set("network", "dds_domain", int(self.inp_domain.text()))
        
        # Robot
        settings.set("robot", "model_xml_path", self.inp_model_path.text())
        settings.set("robot", "rl_policy_path", self.inp_policy_path.text())
        settings.set("robot", "use_right_arm", self.chk_right_arm.isChecked())
        settings.set("control", "sim_only", self.chk_sim_only.isChecked())
        
        # ROS
        settings.set("ros", "setup_path", self.inp_setup_bash.text())
        
        # Misc
        settings.set("ai", "deepseek_api_key", self.inp_api_key.text())
        try:
            settings.set("calibration", "cam_pitch_deg", float(self.inp_cam_pitch.text()))
            settings.set("calibration", "cam_offset_z", float(self.inp_cam_z.text()))
        except: pass
        
        QMessageBox.information(self, "成功", "配置已保存！\nSettings have been saved to settings.json")

    def launch_app(self):
        self.save_settings()
        self.close()
        # 启动 main.py
        # 使用当前 Python 解释器启动
        subprocess.Popen([sys.executable, "main.py"])

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = Launcher()
    w.show()
    sys.exit(app.exec_())
