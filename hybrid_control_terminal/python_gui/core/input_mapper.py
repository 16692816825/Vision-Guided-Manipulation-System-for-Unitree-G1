# core/input_mapper.py
import time
from PyQt5.QtCore import Qt
from config import BASE_WALK_SPEED, BASE_TURN_SPEED, BASE_STRAFE_SPEED

class InputMapper:
    """
    输入映射层：将原始输入（键盘/手柄）转换为控制语义
    """
    def __init__(self):
        self.keys_pressed = set()
        self.last_move_time = 0
        self.is_moving = False

    def update_keys(self, key, pressed):
        if pressed:
            self.keys_pressed.add(key)
        elif key in self.keys_pressed:
            self.keys_pressed.remove(key)

    def get_walking_command(self, speed_mult):
        """返回 (vx, vy, rot, is_moving_flag)"""
        vx, vy, rot = 0.0, 0.0, 0.0
        active = False
        
        if Qt.Key_Up in self.keys_pressed:    vx = BASE_WALK_SPEED * speed_mult; active = True
        if Qt.Key_Down in self.keys_pressed:  vx = -BASE_WALK_SPEED * speed_mult; active = True
        if Qt.Key_Left in self.keys_pressed:  rot = BASE_TURN_SPEED * speed_mult; active = True
        if Qt.Key_Right in self.keys_pressed: rot = -BASE_TURN_SPEED * speed_mult; active = True
        if Qt.Key_4 in self.keys_pressed:     vy = BASE_STRAFE_SPEED * speed_mult; active = True
        if Qt.Key_6 in self.keys_pressed:     vy = -BASE_STRAFE_SPEED * speed_mult; active = True

        if active:
            self.last_move_time = time.time()
            self.is_moving = True
        elif self.is_moving and (time.time() - self.last_move_time > 0.2):
            # 停止后的缓冲期结束，标记为完全静止
            self.is_moving = False
            
        return vx, vy, rot, self.is_moving

    def get_arm_deltas(self):
        """返回 (dx, dy, dz) 目标点增量"""
        dx, dy, dz = 0.0, 0.0, 0.0
        v = 0.02 # 单步增量
        
        if Qt.Key_W in self.keys_pressed: dz += v
        if Qt.Key_S in self.keys_pressed: dz -= v
        if Qt.Key_A in self.keys_pressed: dy += v
        if Qt.Key_D in self.keys_pressed: dy -= v
        if Qt.Key_Q in self.keys_pressed: dx += v
        if Qt.Key_E in self.keys_pressed: dx -= v
        
        return dx, dy, dz

    def check_mode_toggle(self, current_mode):
        """检测模式切换按键，返回新模式或None"""
        if Qt.Key_R in self.keys_pressed and current_mode == 'WALKING':
            return 'ARM_CONTROL'
        if Qt.Key_F in self.keys_pressed and current_mode == 'ARM_CONTROL':
            return 'WALKING'
        return None
        
    def check_reset(self):
        return Qt.Key_Space in self.keys_pressed
