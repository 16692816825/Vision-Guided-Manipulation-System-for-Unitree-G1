# ui/tabs/voice_tab.py
import datetime
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QFrame, 
                             QTextBrowser, QLineEdit, QPushButton)
from PyQt5.QtCore import Qt, pyqtSignal

from ui.styles import Styles

class VoiceTab(QWidget):
    """
    Tab 7: AI 语音助手界面
    职责：展示聊天记录(QTextBrowser)、采集用户输入并发送信号
    """
    # 信号：用户发送的文本内容
    sig_user_input = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        
        # 1. 聊天记录显示区
        self.chat_browser = QTextBrowser()
        self.chat_browser.setStyleSheet(Styles.VOICE_BROWSER)
        
        # 默认欢迎语
        welcome_html = """
        <div style="text-align: center; color: #666; margin-top: 30px;">
            <h2 style="color: #00e5ff;">G1 语音中枢 (DeepSeek)</h2>
            <p>请按住说话，或输入："给大家打个招呼"</p>
        </div>
        """
        self.chat_browser.setHtml(welcome_html)
        layout.addWidget(self.chat_browser, 1)

        # 2. 底部输入区
        input_frame = QFrame()
        input_frame.setStyleSheet(Styles.VOICE_INPUT_FRAME)
        input_frame.setFixedHeight(70)
        
        input_layout = QHBoxLayout(input_frame)
        input_layout.setContentsMargins(10, 10, 10, 10)

        # 输入框
        self.inp_chat_msg = QLineEdit()
        self.inp_chat_msg.setPlaceholderText("在此输入指令...")
        self.inp_chat_msg.setStyleSheet(Styles.VOICE_INPUT_EDIT)
        self.inp_chat_msg.returnPressed.connect(self._on_send_click)
        input_layout.addWidget(self.inp_chat_msg, 1)

        # 发送按钮
        self.btn_chat_send = QPushButton("发送")
        self.btn_chat_send.setFixedSize(80, 36)
        self.btn_chat_send.setCursor(Qt.PointingHandCursor)
        self.btn_chat_send.setStyleSheet(Styles.VOICE_BTN_SEND)
        self.btn_chat_send.clicked.connect(self._on_send_click)
        input_layout.addWidget(self.btn_chat_send)

        layout.addWidget(input_frame)

    def _on_send_click(self):
        text = self.inp_chat_msg.text().strip()
        if not text: return
        
        # 1. 先在界面上显示用户说的话
        self.append_message("Operator", text, role="user")
        
        # 2. 发送信号给主窗口处理 (调用 DeepSeek)
        self.sig_user_input.emit(text)
        
        # 3. 清空输入框并锁定，等待回复
        self.inp_chat_msg.clear()
        self.btn_chat_send.setEnabled(False)
        self.btn_chat_send.setText("...")

    def reset_input_state(self):
        """恢复输入框可用状态 (在收到回复后调用)"""
        self.btn_chat_send.setEnabled(True)
        self.btn_chat_send.setText("发送")
        self.inp_chat_msg.setFocus()

    def append_message(self, name, text, role="user"):
        """
        向聊天框添加消息气泡
        role: "user" | "robot" | "system"
        """
        time_str = datetime.datetime.now().strftime("%H:%M")
        html = ""
        
        if role == "user":
            html = f"""
            <div align="right" style="margin-bottom: 15px;">
                <div style="font-size: 10px; color: #888; margin-bottom: 4px; margin-right: 5px;">
                    {name} <span style="color: #555;">{time_str}</span>
                </div>
                <span style="background-color: #2e7d32; color: #e0e0e0; font-size: 14px; 
                             padding: 10px 15px; border-radius: 15px 0 15px 15px;">
                    {text}
                </span>
            </div>
            """
        elif role == "robot":
            html = f"""
            <div align="left" style="margin-bottom: 15px;">
                <div style="font-size: 10px; color: #888; margin-bottom: 4px; margin-left: 5px;">
                    <span style="color: #00e5ff; font-weight: bold;">G1 AI</span> <span style="color: #555;">{time_str}</span>
                </div>
                <span style="background-color: #333333; color: #ffffff; font-size: 14px; 
                             padding: 10px 15px; border-radius: 0 15px 15px 15px; border-left: 3px solid #00e5ff;">
                    {text}
                </span>
            </div>
            """
        elif role == "system":
            html = f"""
            <div align="center" style="margin: 10px 0;">
                <span style="background-color: #37474f; color: #ffeb3b; font-size: 11px; 
                             padding: 4px 12px; border-radius: 10px; font-style: italic;">
                    ⚡ {text}
                </span>
            </div>
            """

        self.chat_browser.append(html)
        self.chat_browser.verticalScrollBar().setValue(self.chat_browser.verticalScrollBar().maximum())
