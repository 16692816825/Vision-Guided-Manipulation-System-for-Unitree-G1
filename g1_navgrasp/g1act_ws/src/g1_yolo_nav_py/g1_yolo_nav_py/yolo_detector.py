"""YOLO 目标检测节点 — 订阅相机图像，运行 YOLO 推理，发布 2D 检测结果。"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from std_msgs.msg import Header
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class YoloDetectorNode(Node):
    """YOLO 目标检测节点。"""

    def __init__(self) -> None:
        super().__init__("g1_yolo_detector_node")

        # ---- 参数 ----
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("input_image_topic", "/camera/color/image_raw")
        self.declare_parameter("output_detection_topic", "/g1/vision/detections")
        self.declare_parameter("target_classes", [0])  # 默认检测 person (COCO id=0)
        self.declare_parameter("max_image_size", 640)

        model_path = self.get_parameter("model_path").value
        self._conf_thresh = float(self.get_parameter("confidence_threshold").value)
        self._nms_thresh = float(self.get_parameter("nms_threshold").value)
        self._target_classes = list(self.get_parameter("target_classes").value)
        self._max_size = int(self.get_parameter("max_image_size").value)

        self.get_logger().info(
            f"模型路径: {model_path}, 置信度阈值: {self._conf_thresh}, "
            f"目标类别: {self._target_classes}"
        )

        # ---- 加载模型 ----
        if YOLO is None:
            self.get_logger().error("ultralytics 未安装，请执行: pip3 install ultralytics")
            raise RuntimeError("ultralytics package not found")

        try:
            self._model = YOLO(model_path)
            self.get_logger().info(f"YOLO 模型加载成功: {model_path}")
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {e}")
            raise

        # ---- CV Bridge ----
        self._bridge = CvBridge()

        # ---- QoS ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ---- 话题 ----
        input_topic = self.get_parameter("input_image_topic").value
        output_topic = self.get_parameter("output_detection_topic").value
        self._sub = self.create_subscription(
            Image, input_topic, self._image_callback, sensor_qos
        )
        self._pub = self.create_publisher(Detection2DArray, output_topic, 10)

        self.get_logger().info(f"订阅图像: {input_topic}, 发布检测: {output_topic}")

    # ------------------------------------------------------------------
    def _image_callback(self, msg: Image) -> None:
        """图像回调：执行推理并发布检测结果。"""
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        # 保持原始尺寸用于坐标还原
        orig_h, orig_w = cv_image.shape[:2]

        # 推理
        results = self._model(
            cv_image,
            conf=self._conf_thresh,
            iou=self._nms_thresh,
            classes=self._target_classes if self._target_classes else None,
            verbose=False,
        )

        # 构造检测结果
        det_array = Detection2DArray()
        det_array.header = msg.header

        if results and len(results) > 0:
            boxes = results[0].boxes
            for box in boxes:
                det = Detection2D()
                hyp = ObjectHypothesisWithPose()
                hyp.id = str(int(box.cls[0]))
                hyp.score = float(box.conf[0])

                # BoundingBox2D（归一化坐标）
                bbox = BoundingBox2D()
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                w = x2 - x1
                h = y2 - y1
                bbox.center.position.x = cx / orig_w
                bbox.center.position.y = cy / orig_h
                bbox.size_x = w / orig_w
                bbox.size_y = h / orig_h

                det.bbox = bbox
                det.results.append(hyp)
                det_array.detections.append(det)

        self._pub.publish(det_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
