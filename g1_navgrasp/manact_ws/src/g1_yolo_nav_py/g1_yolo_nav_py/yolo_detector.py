"""YOLO 目标检测节点 — 订阅相机图像，运行 YOLO 推理，发布 2D 检测结果。"""

import os
import sys
from collections import deque

# ROS2 colcon 会隔离 PYTHONPATH，必须在所有 import 之前追加路径
for _p in [
    "/usr/lib/python3/dist-packages",
    os.path.expanduser("~/.local/lib/python3.8/site-packages"),
    "/usr/local/lib/python3.8/dist-packages",
]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from typing import List  # 类型注解（Python 3.8 兼容）
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np

# ultralytics: YOLO 目标检测模型库（可选依赖）
try:
    from ultralytics import YOLO
except Exception as _e:
    print(f"[DEBUG] ultralytics import failed: {type(_e).__name__}: {_e}")
    YOLO = None

class YoloDetectorNode(Node):
    """YOLO 目标检测节点。"""

    def __init__(self) -> None:
        super().__init__("g1_yolo_detector_node")

        self.declare_parameter("model_path", "yolo_v11s_best.pt")
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("input_image_topic", "/D455_1/color/image_raw")
        self.declare_parameter("output_detection_topic", "/g1/vision/detections")
        self.declare_parameter("target_classes", ["chair"])  # 按类别名称过滤
        self.declare_parameter("max_image_size", 640)
        self.declare_parameter("use_tta", False)  # 测试时增强（TTA），提高精度但降低帧率
        self.declare_parameter("clahe_enabled", True)  # CLAHE 对比度增强
        self.declare_parameter("skip_frames", 1)  # 跳帧：每 N 帧推理一次（1=不跳，2=每2帧推理1次）
        self.declare_parameter("voting_window", 3)  # 多帧投票窗口大小（1 表示关闭）

        model_path = self.get_parameter("model_path").value
        # 若为相对路径，自动解析为 share 目录下的 models 子目录
        if not os.path.isabs(model_path):
            pkg_share = get_package_share_directory("g1_yolo_nav_py")
            model_path = os.path.join(pkg_share, "models", model_path)
        self._conf_thresh = float(self.get_parameter("confidence_threshold").value)
        self._nms_thresh = float(self.get_parameter("nms_threshold").value)
        self._max_size = int(self.get_parameter("max_image_size").value)
        self._use_tta = bool(self.get_parameter("use_tta").value)
        self._clahe_enabled = bool(self.get_parameter("clahe_enabled").value)
        self._skip_frames = max(1, int(self.get_parameter("skip_frames").value))
        self._voting_window = int(self.get_parameter("voting_window").value)
        self._frame_counter = 0  # 跳帧计数器
        self._last_det_array = None  # 缓存上一次检测结果（跳帧复用）

        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        self._det_buffer: deque = deque(maxlen=max(self._voting_window, 1))

        self.get_logger().info(
            f"模型路径: {model_path}, 置信度阈值: {self._conf_thresh}, "
            f"imgsz: {self._max_size}, TTA: {self._use_tta}, "
            f"CLAHE: {self._clahe_enabled}, 跳帧: {self._skip_frames}, 投票窗口: {self._voting_window}"
        )

        if YOLO is None:
            self.get_logger().error("ultralytics 未安装，请执行: pip3 install ultralytics")
            raise RuntimeError("ultralytics package not found")

        try:
            self._model = YOLO(model_path)
            self.get_logger().info(f"YOLO 模型加载成功: {model_path}")
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {e}")
            raise

        # model.names: {0: "chair", 1: "table", ...}
        self._name_to_id = {v: int(k) for k, v in self._model.names.items()}
        self._target_class_ids: List[int] = []
        raw_targets = list(self.get_parameter("target_classes").value)
        for t in raw_targets:
            if isinstance(t, (int, float)):
                self._target_class_ids.append(int(t))
            elif isinstance(t, str) and t in self._name_to_id:
                self._target_class_ids.append(self._name_to_id[t])
            else:
                self.get_logger().warn(f"未知目标类别 '{t}'，可用类别: {list(self._name_to_id.keys())}")
        self.get_logger().info(f"目标类别 ID: {self._target_class_ids} ({raw_targets})")

        self._bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        input_topic = self.get_parameter("input_image_topic").value
        output_topic = self.get_parameter("output_detection_topic").value
        self._sub = self.create_subscription(
            Image, input_topic, self._image_callback, sensor_qos
        )
        self._pub = self.create_publisher(Detection2DArray, output_topic, 10)

        self.get_logger().info(f"订阅图像: {input_topic}, 发布检测: {output_topic}")

    def _preprocess(self, image: np.ndarray) -> np.ndarray:
        """CLAHE 对比度增强，改善低光/逆光场景检测。"""
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = self._clahe.apply(l)
        lab = cv2.merge([l, a, b])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def _image_callback(self, msg: Image) -> None:
        """图像回调：执行推理并发布检测结果。"""
        self._frame_counter += 1

        # 跳帧：不是每帧都推理，复用上一次结果
        if self._frame_counter % self._skip_frames != 0:
            if self._last_det_array is not None:
                self._last_det_array.header = msg.header
                self._pub.publish(self._last_det_array)
            return

        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        orig_h, orig_w = cv_image.shape[:2]

        if self._clahe_enabled:
            cv_image = self._preprocess(cv_image)

        infer_kwargs = dict(
            conf=self._conf_thresh,
            iou=self._nms_thresh,
            classes=self._target_class_ids if self._target_class_ids else None,
            imgsz=self._max_size,
            verbose=False,
        )
        if self._use_tta:
            infer_kwargs["augment"] = True

        results = self._model(cv_image, **infer_kwargs)

        cur_dets = []
        if results and len(results) > 0:
            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2.0 / orig_w
                cy = (y1 + y2) / 2.0 / orig_h
                w = (x2 - x1) / orig_w
                h = (y2 - y1) / orig_h
                cur_dets.append({
                    "id": self._model.names[int(box.cls[0])],
                    "score": float(box.conf[0]),
                    "cx": cx, "cy": cy, "w": w, "h": h,
                })

        # 多帧投票：只有连续 N 帧中都出现的检测才输出
        self._det_buffer.append(cur_dets)
        stable_dets = self._vote_detections() if len(self._det_buffer) >= self._voting_window else cur_dets

        det_array = Detection2DArray()
        det_array.header = msg.header

        for d in stable_dets:
            det = Detection2D()
            hyp = ObjectHypothesisWithPose()
            hyp.id = d["id"]
            hyp.score = d["score"]
            bbox = BoundingBox2D()
            bbox.center.x = d["cx"]
            bbox.center.y = d["cy"]
            bbox.size_x = d["w"]
            bbox.size_y = d["h"]
            det.bbox = bbox
            det.results.append(hyp)
            det_array.detections.append(det)

            self.get_logger().info(
                f'检测到目标: {hyp.id} (置信度={hyp.score:.0%}), '
                f'中心=({bbox.center.x:.2f},{bbox.center.y:.2f})'
            )

        self._pub.publish(det_array)
        self._last_det_array = det_array  # 缓存供跳帧复用

    def _vote_detections(self) -> list:
        """多帧投票：IoU 匹配 + 出现次数计数，过滤单帧闪烁误检。"""
        from math import sqrt

        window = list(self._det_buffer)
        latest = window[-1]
        if not latest:
            return []

        confirmed = []
        for det in latest:
            count = 1
            for prev_frame in window[:-1]:
                for prev_det in prev_frame:
                    # 简化 IoU：用中心点距离判断是否为同一目标
                    dist = sqrt((det["cx"] - prev_det["cx"]) ** 2 + (det["cy"] - prev_det["cy"]) ** 2)
                    if dist < 0.1 and det["id"] == prev_det["id"]:
                        count += 1
                        break
            if count >= self._voting_window:
                confirmed.append(det)

        return confirmed

def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()