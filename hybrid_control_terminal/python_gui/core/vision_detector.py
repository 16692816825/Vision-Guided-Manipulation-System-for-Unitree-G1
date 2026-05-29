# core/vision_detector.py
import cv2
import numpy as np
import torch
from ultralytics import YOLO

class ObjectDetector:
    def __init__(self, model_path="yolov8n.pt"):
        print(f"[Vision] 正在加载 YOLO 模型: {model_path} ...")
        # 自动利用 GPU，如果没有则使用 CPU
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO(model_path)
        print(f"[Vision] 模型加载完成，运行设备: {self.device}")
        
        # 目标类别过滤器 (COCO数据集)
        # 39: bottle, 41: cup, 67: cell phone, etc.
        # 如果为 None，则检测所有
        self.target_classes = [39, 41] 

    def detect(self, cv_image, conf_thres=0.5):
        """
        输入: OpenCV 图像 (BGR)
        输出: list of dict {'class': int, 'conf': float, 'box': [x1, y1, x2, y2], 'center': (u, v)}
        """
        if cv_image is None:
            return []

        # 运行推理
        results = self.model(cv_image, conf=conf_thres, device=self.device, verbose=False)
        
        detections = []
        
        # 解析结果
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                
                # 过滤类别
                if self.target_classes and cls_id not in self.target_classes:
                    continue
                
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                
                # 计算中心点
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)
                
                detections.append({
                    'class': cls_id,
                    'class_name': self.model.names[cls_id],
                    'conf': conf,
                    'box': [x1, y1, x2, y2],
                    'center': (u, v)
                })
        
        return detections

    def draw_results(self, cv_image, detections):
        """
        在图像上绘制检测框和标签
        """
        img_copy = cv_image.copy()
        
        for d in detections:
            x1, y1, x2, y2 = d['box']
            label = f"{d['class_name']} {d['conf']:.2f}"
            
            # 颜色策略：瓶子用绿色，其他用红色
            color = (0, 255, 0) if d['class_name'] == 'bottle' else (0, 0, 255)
            
            # 1. 画矩形框
            cv2.rectangle(img_copy, (x1, y1), (x2, y2), color, 2)
            
            # 2. 画标签背景条 (让文字更清晰)
            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv2.rectangle(img_copy, (x1, y1 - 20), (x1 + w, y1), color, -1)
            
            # 3. 写文字 (例如 "bottle 0.85")
            cv2.putText(img_copy, label, (x1, y1 - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # 4. 画中心点
            u, v = d['center']
            cv2.circle(img_copy, (u, v), 4, (0, 255, 255), -1)
            
        return img_copy