"""可视化工具 — 共享颜色表、绘制函数和图像转换。

提供：
    get_color              — 根据类别 ID 获取稳定颜色
    draw_detections_on_frame — 在图像上绘制检测框和标签
    cv2_to_tk              — OpenCV BGR 图像 → tkinter PhotoImage
"""

from typing import Tuple

import cv2
import numpy as np

# 预定义颜色表 (BGR) — 与设计系统功能色对应
COLORS = [
    (178, 145, 8),     # Teal-600 BGR
    (246, 130, 59),    # Blue-500 BGR
    (68, 68, 239),     # Red-500 BGR
    (11, 158, 245),    # Amber-500 BGR
    (129, 182, 16),    # Emerald-500 BGR
    (246, 92, 139),    # Violet-500 BGR
]

def get_color(class_id: str) -> Tuple[int, int, int]:
    """根据类别 ID 获取稳定颜色（相同 ID 始终返回相同颜色）。"""
    idx = hash(class_id) % len(COLORS)
    return COLORS[idx]

def draw_detections_on_frame(frame: np.ndarray, detections) -> np.ndarray:
    """在图像上绘制检测框和标签（返回新图像，不修改原图）。

    Args:
        frame: BGR 图像 (numpy array, HxWx3)
        detections: Detection2DArray 消息（含 .detections 属性）
                    或 Detection2D 的列表

    Returns:
        标注后的图像副本
    """
    out = frame.copy()
    h, w = out.shape[:2]

    det_list = detections.detections if hasattr(detections, "detections") else detections
    for det in det_list:
        if not det.results:
            continue
        class_id = det.results[0].id
        score = det.results[0].score
        color = get_color(class_id)

        cx = det.bbox.center.x * w
        cy = det.bbox.center.y * h
        bw = det.bbox.size_x * w
        bh = det.bbox.size_y * h
        x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
        x2, y2 = int(cx + bw / 2), int(cy + bh / 2)

        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        label = f"{class_id} {score:.0%}"
        cv2.rectangle(out, (x1, y1 - 22), (x1 + len(label) * 10, y1), color, -1)
        cv2.putText(out, label, (x1 + 3, y1 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    return out

def cv2_to_tk(frame: np.ndarray, width: int, height: int):
    """OpenCV BGR 图像 → tkinter PhotoImage（缩放适配目标尺寸）。

    Args:
        frame: BGR 图像 (numpy array)
        width: 目标宽度（像素）
        height: 目标高度（像素）

    Returns:
        ImageTk.PhotoImage
    """
    from PIL import Image as PILImage, ImageTk

    resized = cv2.resize(frame, (width, height), interpolation=cv2.INTER_LINEAR)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    rgb = np.ascontiguousarray(rgb)
    pil_img = PILImage.fromarray(rgb)
    return ImageTk.PhotoImage(image=pil_img)
