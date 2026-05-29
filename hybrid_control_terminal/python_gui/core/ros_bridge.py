# core/ros_bridge.py
import threading
import time
import numpy as np
import subprocess
import os
import rospy
import cv2
import math
import tf 
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

try:
    from cv_bridge import CvBridge
except ImportError:
    print("[RosBridge] 警告: 未安装 cv_bridge, 视觉功能不可用")
    CvBridge = None

# --- 视觉外参配置 ---
CAM_OFFSET_X = 0.0476
CAM_OFFSET_Y = 0.0
CAM_OFFSET_Z = 0.4627
CAM_PITCH_DEG = 42.0

class RosBridgeThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.daemon = True
        self.running = True
        self.lock = threading.Lock()
        self.cmd_lock = threading.Lock()
        
        # 数据缓存
        self.latest_scan = None
        self.latest_map = None
        self.map_info = None
        self.robot_pose = [0, 0, 0] 
        self.nav_cmd_vel = [0, 0, 0]
        self.global_path = []
        self.local_path = []

        # 视觉缓存
        self.latest_image = None
        self.latest_depth = None
        self.image_lock = threading.Lock()
        self.cv_bridge = CvBridge() if CvBridge else None
        
        # 相机参数
        self.camera_intrinsics = None 
        theta = math.radians(CAM_PITCH_DEG)
        c, s = math.cos(theta), math.sin(theta)
        self.R_cam2base = np.array([[0, -s, c], [-1, 0, 0], [0, -c, -s]])
        self.T_cam2base = np.array([CAM_OFFSET_X, CAM_OFFSET_Y, CAM_OFFSET_Z])

        self.tf_listener = None 

    def run(self):
        if not rospy.core.is_initialized():
            rospy.init_node('g1_gui_bridge', anonymous=True, disable_signals=True)
        
        self.tf_listener = tf.TransformListener()

        # 订阅
        rospy.Subscriber('/map', OccupancyGrid, self._map_cb)
        rospy.Subscriber('/cmd_vel', Twist, self._cmd_cb)
        rospy.Subscriber('/scan', LaserScan, self._scan_cb, queue_size=1)
        
        # 路径规划
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self._global_path_cb)
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self._global_path_cb)
        rospy.Subscriber('/move_base/plan', Path, self._global_path_cb) 
        rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, self._local_path_cb)
        rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, self._local_path_cb)

        # 视觉
        if self.cv_bridge:
            rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self._img_cb, queue_size=1)
            rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self._depth_cb, queue_size=1)
            rospy.Subscriber('/camera/color/camera_info', CameraInfo, self._info_cb)

        # 发布
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_init_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        
        print("[ROS1 Bridge] 核心桥接已启动")
        
        rate = rospy.Rate(20) 
        while self.running and not rospy.is_shutdown():
            self._update_pose_from_tf()
            rate.sleep()

    def _update_pose_from_tf(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            yaw = euler[2]
            with self.lock:
                self.robot_pose = [trans[0], trans[1], yaw]
        except: pass

    # --- 回调函数 ---
    def _map_cb(self, msg):
        try:
            with self.lock:
                w, h = msg.info.width, msg.info.height
                raw_data = np.array(msg.data, dtype=np.int8).reshape((h, w))
                self.latest_map = raw_data
                self.map_info = msg.info
        except: pass

    def _cmd_cb(self, msg):
        with self.cmd_lock: self.nav_cmd_vel = [msg.linear.x, msg.linear.y, msg.angular.z]

    def _global_path_cb(self, msg):
        if not msg.poses: return
        path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self.lock: self.global_path = path

    def _local_path_cb(self, msg):
        if not msg.poses: 
            with self.lock: self.local_path = []
            return
        src_frame = msg.header.frame_id
        target_frame = "map"
        final_path = []
        if src_frame.strip("/") == target_frame:
            final_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        else:
            try:
                (trans, rot) = self.tf_listener.lookupTransform(target_frame, src_frame, rospy.Time(0))
                mat = self.tf_listener.fromTranslationRotation(trans, rot)
                for p in msg.poses:
                    vec = np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z, 1.0])
                    transformed = np.dot(mat, vec)
                    final_path.append((transformed[0], transformed[1]))
            except: return
        with self.lock: self.local_path = final_path

    # [核心优化点] 大幅降低点云采样率
    def _scan_cb(self, msg):
        # [新增] 节流阀：每 5 帧处理一次 (丢弃 80% 的雷达包)
        # 导航只需要低频避障即可，不需要 100Hz 的雷达数据
        if not hasattr(self, '_scan_skip'): self._scan_skip = 0
        self._scan_skip += 1
        if self._scan_skip % 5 != 0: return 

        
        try:
            # [修改] 增大采样步长，减少点云数量
            # 原来 stride = 3 或 8
            stride = 20 # 极其稀疏，但足够显示障碍物轮廓，极大降低 CPU 负担
            
            ranges = np.array(msg.ranges)
            valid_mask = (ranges > msg.range_min) & (ranges < msg.range_max)
            angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

            valid_ranges = ranges[valid_mask][::stride]
            valid_angles = angles[valid_mask][::stride]
            
            with self.lock:
                self.latest_scan = (valid_ranges, valid_angles)
        except: pass

    def _img_cb(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            with self.image_lock: self.latest_image = cv_img
        except: pass

    def _depth_cb(self, msg):
        if self.cv_bridge is None: return
        try:
            cv_depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
            with self.image_lock: self.latest_depth = cv_depth
        except: pass

    def _info_cb(self, msg):
        if self.camera_intrinsics is None:
            k = msg.K
            self.camera_intrinsics = {'fx': k[0], 'fy': k[4], 'cx': k[2], 'cy': k[5]}

    def get_camera_image(self):
        with self.image_lock: return self.latest_image.copy() if self.latest_image is not None else None

    def get_3d_pos_from_pixel(self, u, v):
        with self.image_lock:
            if self.latest_depth is None or self.camera_intrinsics is None: return None
            h, w = self.latest_depth.shape
            roi_size = 8
            u_min, u_max = max(0, u - roi_size), min(w - 1, u + roi_size)
            v_min, v_max = max(0, v - roi_size), min(h - 1, v + roi_size)
            roi_depth = self.latest_depth[v_min:v_max, u_min:u_max].astype(np.float32)
            mask = (roi_depth > 300) & (roi_depth < 1200)
            valid_depths = roi_depth[mask]
            if len(valid_depths) < 15: return None
            q25, q75 = np.percentile(valid_depths, [25, 75])
            iqr_mask = (valid_depths >= q25) & (valid_depths <= q75)
            final_depths = valid_depths[iqr_mask]
            if len(final_depths) == 0: return None
            z_c = np.mean(final_depths) / 1000.0
            intr = self.camera_intrinsics
            x_c = (u - intr['cx']) * z_c / intr['fx']
            y_c = (v - intr['cy']) * z_c / intr['fy']
            p_cam = np.array([x_c, y_c, z_c])
            p_base = self.R_cam2base @ p_cam + self.T_cam2base
            return p_base

    def publish_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x; goal.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = q[0]; goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]; goal.pose.orientation.w = q[3]
        self.pub_goal.publish(goal)

    def publish_initial_pose(self, x, y, yaw):
        p = PoseWithCovarianceStamped()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()
        p.pose.pose.position.x = x; p.pose.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        p.pose.pose.orientation.z = q[2]; p.pose.pose.orientation.w = q[3]
        cov = [0.0] * 36; cov[0] = 0.25; cov[7] = 0.25; cov[35] = 0.068
        p.pose.covariance = tuple(cov)
        self.pub_init_pose.publish(p)

    def get_nav_data(self):
        with self.lock:
            pose_list = [float(self.robot_pose[0]), float(self.robot_pose[1]), float(self.robot_pose[2])]
            scan_data = self.latest_scan
            if self.latest_map is None:
                return (None, None, pose_list, list(self.global_path), list(self.local_path), scan_data)
            return (self.latest_map.copy(), self.map_info, pose_list, 
                    list(self.global_path), list(self.local_path), scan_data)
    
    def get_cmd_vel(self):
        with self.cmd_lock: return list(self.nav_cmd_vel)
