# ui/nav_widget.py
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import (QPainter, QColor, QImage, QPen, QTransform, 
                         QBrush, QFont, QPolygonF, QConicalGradient)
from PyQt5.QtCore import Qt, pyqtSignal, QPointF, QRectF, QTimer, QDateTime
import numpy as np
import math

class NavMapWidget(QWidget):
    set_goal_signal = pyqtSignal(float, float, float)
    set_pose_signal = pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()
        # 数据容器
        self.map_img = None
        self.map_info = None
        self.robot_pose = [0, 0, 0] # [x, y, yaw] (世界坐标)
        self.global_path = []
        self.local_path = []
        self.scan_data = None
        self.nav_goal = None
        
        # 视图参数
        self.scale = 30.0        # 缩放比例 (像素/米)
        self.view_center = QPointF(0, 0) # 当前视野中心 (世界坐标米)
        
        # 交互状态
        self.dragging = False
        self.last_mouse_pos = QPointF()
        self.interaction_mode = None 
        self.input_mode = 'VIEW' # VIEW, RELOC, NAV
        self.click_start_world = None # 记录点击时的世界坐标
        self.current_mouse_world = None
        
        self.is_reloc_mode_active = False 
        self.ignore_pose_update_until = 0
        
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMouseTracking(True)
        self.scan_angle = 0.0
        self.task_list = [] 
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(33) # 30 FPS

    def set_input_mode(self, mode):
        self.input_mode = mode
        self.update()

    def set_reloc_mode(self, active: bool):
        self.is_reloc_mode_active = active
        self.set_input_mode('RELOC' if active else 'VIEW')

    def clear_map(self):
        self.map_img = None
        self.map_info = None
        self.global_path = []
        self.local_path = []
        self.scan_data = None
        self.nav_goal = None
        self.update()

    def update_data(self, map_data, map_info, robot_pose, g_path=None, l_path=None, scan_data=None):
        # 1. 处理地图图片
        if map_data is not None:
            h, w = map_data.shape
            
            # 颜色映射
            img_rgba = np.zeros((h, w, 4), dtype=np.uint8)
            img_rgba[map_data == -1] = [0, 0, 0, 0]        # 未知透明
            img_rgba[map_data == 0] = [40, 50, 60, 255]    # 空闲深灰
            img_rgba[map_data == 100] = [0, 229, 255, 200] # 障碍物青色
            
            # 【关键步骤】
            # ROS GridData: Row 0 is BOTTOM.
            # Qt QImage: Row 0 is TOP.
            # 所以我们需要上下翻转数据，让它变成一张符合人类直觉的图片
            img_rgba = np.flipud(img_rgba)
            img_rgba = np.ascontiguousarray(img_rgba)
            
            self._keep_alive_buffer = img_rgba
            self.map_img = QImage(self._keep_alive_buffer.tobytes(), w, h, w*4, QImage.Format_RGBA8888).copy()
            self.map_info = map_info
            
        # 2. 更新机器人位姿 (带保护)
        now = QDateTime.currentMSecsSinceEpoch()
        if robot_pose and now > self.ignore_pose_update_until and not self.click_start_world:
            if not (math.isnan(robot_pose[0]) or math.isinf(robot_pose[0])):
                self.robot_pose = robot_pose
                
                # 如果没有手动拖拽，让视野跟随机器人
                # self.view_center = QPointF(self.robot_pose[0], self.robot_pose[1])

        if g_path: self.global_path = g_path
        if l_path: self.local_path = l_path
        if scan_data: self.scan_data = scan_data
        
        self.update()
    # [新增方法] 请添加到 update_data 方法下方
    def set_task_list(self, tasks):
        """更新任务点列表"""
        self.task_list = tasks
        self.update()
    # =========================================================================
    # 核心：坐标变换矩阵 (World -> Screen)
    # =========================================================================
    def _get_world_transform(self):
        """构建从世界坐标(米)到屏幕像素的变换矩阵"""
        w, h = self.width(), self.height()
        
        t = QTransform()
        
        # 1. 移动屏幕中心作为基准点
        t.translate(w / 2.0, h / 2.0)
        
        # 2. 应用缩放 (Scale)
        # ROS世界坐标系: Y轴向上
        # Qt屏幕坐标系: Y轴向下
        # 所以 Y 轴缩放必须是负的 (-self.scale)
        t.scale(self.scale, -self.scale)
        
        # 3. 移动视野中心 (View Center)
        # 我们希望 view_center 对应屏幕中心，所以要反向移动世界
        t.translate(-self.view_center.x(), -self.view_center.y())
        
        return t

    def _screen_to_world(self, pos_screen):
        """屏幕像素 -> 世界坐标(米)"""
        t = self._get_world_transform()
        t_inv, valid = t.inverted()
        return t_inv.map(pos_screen) if valid else QPointF(0, 0)

    # =========================================================================
    # 绘图逻辑
    # =========================================================================
    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#05080a")) # 背景色

        # 获取变换矩阵
        t = self._get_world_transform()
        
        # 画网格 (辅助线)
        self._draw_grid(p, t)

        # 应用变换，进入“世界坐标系”绘图模式
        p.setTransform(t)

        # --- 以下所有绘制都使用 米(m) 为单位，坐标系为 ROS Map ---

        # 1. 绘制地图
        if self.map_img and self.map_info:
            self._draw_map_layer(p)

        # 2. 绘制雷达点云
        if self.scan_data:
            self._draw_laser_scan(p)

        # 3. 绘制路径
        self._draw_path(p, self.global_path, QColor("#00e676"), 0.08)
        self._draw_path(p, self.local_path, QColor("#ffea00"), 0.10)

        # 4. 绘制导航目标标记
        if self.nav_goal:
            self._draw_goal_marker(p, self.nav_goal)
        # [新增] 绘制任务点 (在机器人下方绘制，防止遮挡机器人)
        if self.task_list:
            self._draw_task_markers(p)
        # 5. 绘制机器人本体
        self._draw_robot(p)

        # 6. 绘制交互箭头 (拖拽中)
        if self.click_start_world and self.current_mouse_world:
            color = QColor("#d500f9") if self.input_mode == 'RELOC' else QColor("#00e676")
            self._draw_arrow(p, self.click_start_world, self.current_mouse_world, color)

        # 重置变换，绘制 HUD (屏幕坐标系)
        p.resetTransform()
        self._draw_hud(p)
        self.scan_angle = (self.scan_angle + 5) % 360

    def _draw_map_layer(self, p):
        """绘制地图，严格对齐 Origin"""
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        
        p.save()
        
        # 1. 移动到地图原点 (左下角)
        p.translate(ox, oy)
        
        # 2. 缩放: 1像素 = res 米
        p.scale(res, res)
        
        # 3. 坐标系调整 (这是最关键的一步)
        # 当前坐标系: 原点在地图左下角，Y轴向上 (因为父级Transform是Y向上的)
        # 我们的图片: 在 update_data 里已经 flipud 了，所以是一张“正”的图片
        # Qt drawImage: 从左上角开始画，向 +Y (屏幕下/世界下) 延伸
        # 为了把图片“立”在原点之上：
        p.scale(1, -1) # 再次翻转 Y，使得 +Y 变为向下 (图片坐标系)
        p.translate(0, -self.map_img.height()) # 把绘制起点向上提一个图片高度
        
        # 此时绘制起点(0,0)对应 世界坐标的 (ox, oy + height*res)
        # 图片向下延伸，刚好底部对齐 oy
        p.setRenderHint(QPainter.SmoothPixmapTransform, False)
        p.drawImage(0, 0, self.map_img)
        
        p.restore()

    def _draw_robot(self, p):
        rx, ry, ryaw = self.robot_pose
        p.save()
        p.translate(rx, ry)
        p.rotate(math.degrees(ryaw)) # ROS 角度直接用
        
        # 绘制机器人 (尺寸: 0.5m)
        p.setPen(QPen(QColor("#00e676"), 0.02))
        p.setBrush(QColor(0,0,0, 200))
        poly = QPolygonF([QPointF(0.3, 0), QPointF(-0.15, 0.15), QPointF(-0.15, -0.15)])
        p.drawPolygon(poly)
        
        # 方向指示扇形 (仅在非重定位模式显示)
        if not self.is_reloc_mode_active:
            p.setPen(Qt.NoPen); p.setBrush(QColor(0, 230, 118, 30))
            p.drawPie(QRectF(-0.5, -0.5, 1.0, 1.0), 60*16, -120*16)
            
        p.restore()

    def _draw_laser_scan(self, p):
        if not self.robot_pose: return
        ranges, angles = self.scan_data
        
        p.save()
        # 雷达数据是相对于 Base Link 的 (pointcloud_to_laserscan 设置了 target_frame)
        p.translate(self.robot_pose[0], self.robot_pose[1])
        p.rotate(math.degrees(self.robot_pose[2]))
        
        p.setPen(QPen(QColor(255, 0, 0, 150), 0.05)) # 红色点
        
        # 极坐标 -> 笛卡尔
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        
        for x, y in zip(xs, ys):
            p.drawPoint(QPointF(x, y))
            
        p.restore()

    def _draw_path(self, p, points, color, width):
        if not points or len(points) < 2: return
        p.save()
        pen = QPen(color, width)
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        
        poly = QPolygonF()
        for pt in points:
            poly.append(QPointF(pt[0], pt[1]))
        p.drawPolyline(poly)
        p.restore()

    def _draw_arrow(self, p, start, end, color):
        p.save()
        p.setPen(QPen(color, 0.05))
        p.drawLine(start, end)
        
        # 画箭头头部
        angle = math.atan2(end.y() - start.y(), end.x() - start.x())
        p.translate(end.x(), end.y())
        p.rotate(math.degrees(angle))
        p.drawLine(QPointF(0,0), QPointF(-0.2, 0.1))
        p.drawLine(QPointF(0,0), QPointF(-0.2, -0.1))
        p.restore()

    def _draw_goal_marker(self, p, pos):
        p.save()
        p.translate(pos[0], pos[1])
        p.setPen(QPen(QColor("#ff0055"), 0.08))
        # 画个叉
        d = 0.2
        p.drawLine(QPointF(-d, -d), QPointF(d, d))
        p.drawLine(QPointF(-d, d), QPointF(d, -d))
        p.restore()

    def _draw_grid(self, p, t):
        # 简单的屏幕十字线
        p.setPen(QPen(QColor(255, 255, 255, 30), 1))
        xc, yc = self.width()/2, self.height()/2
        p.drawLine(xc-20, yc, xc+20, yc)
        p.drawLine(xc, yc-20, xc, yc+20)

    def _draw_hud(self, p):
        """绘制平视显示层 (HUD) - 美化版"""
        w, h = self.width(), self.height()
        
        # 1. 左上角状态信息 (带半透明背景)
        info_text = f"Pose: {self.robot_pose[0]:.2f}, {self.robot_pose[1]:.2f} | Yaw: {math.degrees(self.robot_pose[2]):.1f}°"
        
        p.setFont(QFont("Consolas", 10, QFont.Bold))
        fm = p.fontMetrics()
        txt_w = fm.width(info_text) + 20
        txt_h = fm.height() + 10
        
        # 绘制圆角背景
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 150)) # 半透明黑
        p.drawRoundedRect(10, 10, txt_w, txt_h, 5, 5)
        
        # 绘制文字
        p.setPen(QColor("#00e5ff")) # 青色文字
        p.drawText(20, 10 + fm.ascent() + 5, info_text)

        # 2. 底部操作提示 (根据模式变色)
        if self.input_mode != 'VIEW':
            hint_text = ""
            bg_color = QColor(0, 0, 0, 0)
            txt_color = QColor("white")
            
            if self.input_mode == 'RELOC':
                hint_text = "模式：重定位 (按住 Shift + 拖拽箭头)"
                bg_color = QColor(213, 0, 249, 180) # 紫色背景
            elif self.input_mode == 'NAV':
                hint_text = "模式：导航目标 (按住 Ctrl + 拖拽箭头)"
                bg_color = QColor(0, 230, 118, 180) # 绿色背景
            
            # 绘制底部提示条
            p.setBrush(bg_color)
            p.setPen(Qt.NoPen)
            # 底部居中条
            hint_w = fm.width(hint_text) + 40
            p.drawRoundedRect((w - hint_w)//2, h - 40, hint_w, 30, 15, 15)
            
            p.setPen(txt_color)
            p.drawText(QRectF(0, h-40, w, 30), Qt.AlignCenter, hint_text)

        # 3. 右下角：可视化罗盘 (Compass)
        comp_center_x = w - 50
        comp_center_y = h - 50
        radius = 30
        
        p.save()
        p.translate(comp_center_x, comp_center_y)
        
        # 背景圆
        p.setBrush(QColor(0, 0, 0, 100))
        p.setPen(QPen(QColor("white"), 1))
        p.drawEllipse(-radius, -radius, radius*2, radius*2)
        
        # 旋转 (ROS Yaw 是逆时针为正，Qt 也是，但为了视觉对齐可能需要调整相位)
        # 这里假设 robot_pose[2] 0度向右(X轴)
        # 我们希望画一个箭头指向当前车头
        p.rotate(-math.degrees(self.robot_pose[2])) # 负号是因为屏幕坐标系Y向下，旋转方向可能反
        
        # 画箭头 (代表车头)
        p.setBrush(QColor("#00e5ff"))
        p.setPen(Qt.NoPen)
        arrow = QPolygonF([
            QPointF(radius-5, 0),  # 头
            QPointF(-radius+10, 8), # 尾左
            QPointF(-radius+10, -8) # 尾右
        ])
        p.drawPolygon(arrow)
        
        # 标记 "N" (北/X轴方向，视你的地图定义而定，通常 X 是 Forward)
        # 这里简单标示车头即可，不用标 N
        
        p.restore()
    # =========================================================================
    # 交互事件 (鼠标)
    # =========================================================================
    def mousePressEvent(self, e):
        self.setFocus()
        # 【修改】强制转换为 QPointF，确保后续计算保留小数
        pos_f = QPointF(e.pos())
        
        self.last_mouse_pos = pos_f
        self.click_start_world = self._screen_to_world(pos_f)
        self.current_mouse_world = self.click_start_world
        
        if e.button() == Qt.RightButton:
            self.dragging = True
            self.setCursor(Qt.ClosedHandCursor)
        elif e.button() == Qt.LeftButton:
            if self.input_mode == 'RELOC' and (e.modifiers() & Qt.ShiftModifier):
                self.interaction_mode = 'POSE'
            elif self.input_mode == 'NAV' and (e.modifiers() & Qt.ControlModifier):
                self.interaction_mode = 'GOAL'

    def mouseMoveEvent(self, e):
        # 【修改】强制浮点
        pos_f = QPointF(e.pos())
        self.current_mouse_world = self._screen_to_world(pos_f)
        
        if self.dragging:
            # 拖拽计算
            start_w = self._screen_to_world(self.last_mouse_pos)
            end_w = self._screen_to_world(pos_f)
            move_vec = start_w - end_w
            self.view_center += move_vec
            self.update()
            
        self.last_mouse_pos = pos_f
        if self.interaction_mode:
            self.update()

    def mouseReleaseEvent(self, e):
        # 【修改】强制浮点
        pos_f = QPointF(e.pos())
        
        if self.interaction_mode and self.click_start_world:
            end_world = self._screen_to_world(pos_f)
            
            dx = end_world.x() - self.click_start_world.x()
            dy = end_world.y() - self.click_start_world.y()
            dist = math.sqrt(dx*dx + dy*dy)
            yaw = math.atan2(dy, dx)
            
            if dist > 0.05:
                if self.interaction_mode == 'GOAL':
                    # 这里的 s.x() 和 s.y() 现在绝对是浮点数了
                    s = self.click_start_world
                    self.nav_goal = (s.x(), s.y())
                    self.set_goal_signal.emit(s.x(), s.y(), yaw)
                    
                elif self.interaction_mode == 'POSE':
                    s = self.click_start_world
                    corrected_yaw = yaw + math.pi
                    if corrected_yaw > math.pi: corrected_yaw -= 2*math.pi
                    
                    self.robot_pose = [s.x(), s.y(), corrected_yaw]
                    self.ignore_pose_update_until = QDateTime.currentMSecsSinceEpoch() + 2000
                    self.set_pose_signal.emit(s.x(), s.y(), corrected_yaw)

        self.dragging = False
        self.interaction_mode = None
        self.click_start_world = None
        self.setCursor(Qt.ArrowCursor)
        self.update()

    def wheelEvent(self, e):
        # 滚轮缩放
        factor = 1.1 if e.angleDelta().y() > 0 else 0.9
        self.scale = max(1.0, min(500.0, self.scale * factor))
        self.update()
    # [新增方法] 绘制任务点图标
    # 替换 ui/nav_widget.py 中的 _draw_task_markers 方法

    def _draw_task_markers(self, p):
        """绘制任务点图标 (过滤掉等待任务，显示自定义名称)"""
        p.save()
        
        for task in self.task_list:
            # === [修改 1] 过滤逻辑: 如果是等待任务(WAIT)，则不显示图标 ===
            t_type = task.get('type', 'MOVE')
            if t_type == 'WAIT':
                continue

            # 获取坐标
            pose = task.get('pose', {})
            if not pose: continue # 防御性编程
            
            tx = pose.get('x', 0)
            ty = pose.get('y', 0)
            yaw = pose.get('yaw', 0)
            
            # === [修改 2] 名称显示逻辑 ===
            # 优先级: name (新版) > trigger_id (旧版) > uuid (ID) > "?"
            display_text = task.get('name')
            if not display_text:
                display_text = task.get('trigger_id')
            if not display_text:
                display_text = task.get('uuid', '?')[:4] # 如果没名，显示ID前4位

            p.save()
            p.translate(tx, ty)
            
            # 1. 绘制菱形标记
            p.setPen(QPen(QColor("#ffea00"), 0.02))
            p.setBrush(QColor(255, 234, 0, 150))
            
            d = 0.15 # 尺寸
            path = QPolygonF([
                QPointF(0, -d),
                QPointF(d*0.8, 0),
                QPointF(0, d),
                QPointF(-d*0.8, 0)
            ])
            p.drawPolygon(path)
            
            # 2. 绘制朝向箭头
            p.rotate(math.degrees(yaw))
            p.setPen(QPen(QColor("#d500f9"), 0.03))
            p.drawLine(QPointF(0, 0), QPointF(0.4, 0))
            p.drawLine(QPointF(0.4, 0), QPointF(0.3, 0.1))
            p.drawLine(QPointF(0.4, 0), QPointF(0.3, -0.1))
            
            p.restore()
            
            # 3. 绘制文字 (保持文字水平且大小固定)
            p.save()
            p.translate(tx, ty - 0.2) # 文字画在点下方 0.2米处
            
            # 抵消地图的缩放和翻转，确保文字在屏幕上看起来是正的且大小固定
            scale_factor = 1.0 / self.scale 
            p.scale(scale_factor, -scale_factor) 
            
            p.setPen(QColor("#00e5ff"))
            # 使用微软雅黑，字号设大一点看起来更清晰
            font = QFont("Microsoft YaHei", 12, QFont.Bold)
            p.setFont(font)
            
            # 计算文字宽度以居中
            fm = p.fontMetrics()
            txt_w = fm.width(display_text)
            
            # 绘制文字 (居中)
            p.drawText(-txt_w // 2, 0, display_text)
            p.restore()
            
        p.restore()