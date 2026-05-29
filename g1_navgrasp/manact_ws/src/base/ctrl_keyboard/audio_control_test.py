#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import os
import json
import signal
import atexit
import sys
import time

# ===================== 配置 =====================
ROS2_PACKAGE = "ctrl_keyboard"
ROS2_NODE = "ctrl_keyboard"

class VoiceController(Node):
    def __init__(self):
        super().__init__('voice_controller')
        
        # 订阅语音转文字的话题
        self.subscription = self.create_subscription(
            String,
            '/audio_msg',
            self.callback,
            10
        )
        
        # 存储活动进程
        self.active_processes = []
        
        # 键盘控制节点进程
        self.keyboard_proc = None
        
        # ========== 命令配置 ==========
        # 移动命令（通过键盘控制节点）
        self.move_commands = {
            '往左': 'left',
            '向左': 'left',
            '左边': 'left',
            '往右': 'right',
            '向右': 'right',
            '右边': 'right',
            '往前': 'forward',
            '向前': 'forward',
            '前进': 'forward',
            '往后': 'backward',
            '向后': 'backward',
            '后退': 'backward',
            '停下': 'stop',
            '停止': 'stop',
            '停': 'stop',
        }
        
        # 脚本命令
        self.script_commands = {
            '握手': '/home/unitree/g1act_ws/g1act_ws/src/arm/armtest2.py',
            '拿着': '/home/unitree/scripts/hold.sh',
            '走吧': '/home/unitree/scripts/go.sh',
        }
        
        # 多脚本命令
        self.multi_commands = {
            '拿着': [
                '/home/unitree/g1act_ws/g1act_ws/src/arm/armtest3.py',
                '/home/unitree/hand/python/revo2/revo2_control2.py',
            ],
        }
        
        # 关键词匹配
        self.keywords = {
            '左': 'left',
            '右': 'right',
            '前': 'forward',
            '进': 'forward',
            '后': 'backward',
            '退': 'backward',
            '停': 'stop',
        }
        
        # 注册退出清理
        atexit.register(self.cleanup)
        
        self.get_logger().info('语音控制器已启动，等待命令...')
        self.get_logger().info('支持命令: 往左/往右/往前/往后/停下')
    
    def cleanup(self):
        """退出时清理所有子进程"""
        self.get_logger().info("正在清理所有子进程...")
        
        # 清理键盘控制节点
        if self.keyboard_proc and self.keyboard_proc.poll() is None:
            self.get_logger().info("正在终止键盘控制节点...")
            self.keyboard_proc.terminate()
            try:
                self.keyboard_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.keyboard_proc.kill()
        
        # 清理其他脚本进程
        for proc in self.active_processes:
            if proc and proc.poll() is None:
                self.get_logger().info(f"终止进程: {proc.pid}")
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
        
        self.get_logger().info("所有进程已清理。")
    
    def start_keyboard_control(self):
        """启动 ROS2 键盘控制节点（后台运行）"""
        if self.keyboard_proc is None or self.keyboard_proc.poll() is not None:
            try:
                self.get_logger().info("启动 ROS2 键盘控制节点...")
                self.keyboard_proc = subprocess.Popen(
                    ["ros2", "run", ROS2_PACKAGE, ROS2_NODE],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid if hasattr(os, 'setsid') else None,
                    text=True
                )
                self.get_logger().info("ROS2 键盘控制节点已启动")
                # 等待节点启动完成
                time.sleep(1)
                return True
            except FileNotFoundError:
                self.get_logger().error("错误：未找到 ros2 命令，请检查 ROS2 环境！")
                return False
            except Exception as e:
                self.get_logger().error(f"启动 ROS2 节点失败: {e}")
                return False
        return True
    
    def send_keyboard_command(self, command):
        """向键盘控制节点发送命令（通过标准输入）"""
        if self.keyboard_proc and self.keyboard_proc.poll() is None:
            try:
                self.keyboard_proc.stdin.write(f'{command}\n')
                self.keyboard_proc.stdin.flush()
                self.get_logger().info(f'发送键盘命令: {command}')
                return True
            except Exception as e:
                self.get_logger().error(f'发送命令失败: {e}')
                return False
        else:
            self.get_logger().warning("键盘控制节点未运行，正在启动...")
            if self.start_keyboard_control():
                return self.send_keyboard_command(command)
        return False
    
    def move_robot(self, direction):
        """控制机器人移动"""
        # 确保键盘控制节点已启动
        if not self.start_keyboard_control():
            self.get_logger().error("无法启动键盘控制节点")
            return
        
        # 发送移动命令
        self.send_keyboard_command(direction)
        
        # 记录日志
        direction_names = {
            'left': '⬅️ 向左移动',
            'right': '➡️ 向右移动',
            'forward': '⬆️ 向前移动',
            'backward': '⬇️ 向后移动',
            'stop': '🛑 停止移动'
        }
        self.get_logger().info(direction_names.get(direction, f'移动: {direction}'))
    
    def run_script(self, script_path, command_text):
        """执行脚本"""
        def execute():
            if not os.path.exists(script_path):
                self.get_logger().error(f'脚本不存在: {script_path}')
                return
            
            try:
                if script_path.endswith('.py'):
                    cmd = [sys.executable, script_path]
                    script_dir = os.path.dirname(script_path)
                    
                    proc = subprocess.Popen(
                        cmd,
                        cwd=script_dir,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        preexec_fn=os.setsid if hasattr(os, 'setsid') else None
                    )
                    self.active_processes.append(proc)
                    self._monitor_process(proc, script_path)
                else:
                    if not os.access(script_path, os.X_OK):
                        os.chmod(script_path, 0o755)
                    
                    proc = subprocess.Popen(
                        [script_path, command_text],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        preexec_fn=os.setsid if hasattr(os, 'setsid') else None
                    )
                    self.active_processes.append(proc)
                    self._monitor_process(proc, script_path)
                    
            except Exception as e:
                self.get_logger().error(f'执行异常: {e}')
        
        thread = threading.Thread(target=execute, daemon=True)
        thread.start()
    
    def run_multiple_scripts(self, script_paths, command_text):
        """同时执行多个脚本"""
        self.get_logger().info(f'🚀 开始执行 {len(script_paths)} 个脚本')
        for script_path in script_paths:
            self.run_script(script_path, command_text)
    
    def _monitor_process(self, proc, script_name):
        """监控进程输出"""
        def monitor():
            for line in iter(proc.stdout.readline, b''):
                if line:
                    self.get_logger().info(f'[{os.path.basename(script_name)}] {line.decode().strip()}')
            
            for line in iter(proc.stderr.readline, b''):
                if line:
                    self.get_logger().error(f'[{os.path.basename(script_name)}] {line.decode().strip()}')
            
            returncode = proc.wait()
            if proc in self.active_processes:
                self.active_processes.remove(proc)
            
            if returncode == 0:
                self.get_logger().info(f'✅ 脚本完成: {script_name}')
            else:
                self.get_logger().error(f'❌ 脚本退出码 {returncode}: {script_name}')
        
        thread = threading.Thread(target=monitor, daemon=True)
        thread.start()
    
    def callback(self, msg):
        try:
            # 解析 JSON 数据
            data = json.loads(msg.data)
            text = data.get('text', '').strip()
            
            # 移除标点符号
            import re
            text = re.sub(r'[。，！？；：""''《》【】（）、\s]', '', text)
            
            confidence = data.get('confidence', 0)
            
            self.get_logger().info(f'识别到语音: "{text}" (置信度: {confidence:.2f})')
            
            # 置信度过滤
            if confidence < 0.5:
                self.get_logger().warning(f'置信度过低，忽略命令')
                return
            
            # 1. 检查移动命令（精确匹配）
            if text in self.move_commands:
                direction = self.move_commands[text]
                self.get_logger().info(f'✅ 移动命令: "{text}" -> {direction}')
                self.move_robot(direction)
                return
            
            # 2. 检查移动命令（关键词匹配）
            for keyword, direction in self.keywords.items():
                if keyword in text:
                    self.get_logger().info(f'✅ 移动命令(关键词): "{keyword}" -> {direction}')
                    self.move_robot(direction)
                    return
            
            # 3. 检查多脚本命令
            if text in self.multi_commands:
                scripts = self.multi_commands[text]
                self.get_logger().info(f'✅ 多脚本命令: "{text}" -> {len(scripts)}个脚本')
                self.run_multiple_scripts(scripts, text)
                return
            
            # 4. 检查单脚本命令
            if text in self.script_commands:
                script = self.script_commands[text]
                self.get_logger().info(f'✅ 脚本命令: "{text}" -> {script}')
                self.run_script(script, text)
                return
            
            # 5. 未匹配
            self.get_logger().warning(f'❌ 未识别的命令: "{text}"')
                    
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON解析失败: {e}')
        except Exception as e:
            self.get_logger().error(f'处理消息时出错: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceController()
    
    # 设置信号处理
    def signal_handler(sig, frame):
        node.get_logger().info("收到退出信号")
        node.cleanup()
        rclpy.shutdown()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
