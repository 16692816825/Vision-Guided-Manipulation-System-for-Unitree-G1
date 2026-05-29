"""检测结果工具函数 — 从 YOLO 检测结果中提取目标信息。

提供：
    find_best_detection  — 查找指定类别的最高置信度目标
    sample_depth_at_pixel — 在深度图上采样区域中位数
    depth_to_meters       — 深度值单位转换
"""

import numpy as np

def find_best_detection(detections, target_class):
    """从检测结果中查找指定类别的最高置信度目标。

    Args:
        detections: Detection2DArray 消息或 .detections 列表
        target_class: 目标类别 ID（字符串，如 "chair"）

    Returns:
        (best_detection, best_score) — 未找到时返回 (None, 0.0)
    """
    best_det = None
    best_score = 0.0
    for det in detections:
        if det.results and det.results[0].id == target_class:
            if det.results[0].score > best_score:
                best_score = det.results[0].score
                best_det = det
    return best_det, best_score

def sample_depth_at_pixel(depth_image, u, v, radius=5):
    """在深度图上以归一化坐标为中心采样区域中位数。

    Args:
        depth_image: 深度图 (numpy array, HxW)
        u: 归一化列坐标 (0~1)
        v: 归一化行坐标 (0~1)
        radius: 采样区域半径（像素）

    Returns:
        区域中位数深度值（原始单位），无效时返回 None
    """
    if depth_image is None:
        return None
    h, w = depth_image.shape[:2]
    px = int(np.clip(u, 0.0, 1.0) * (w - 1))
    py = int(np.clip(v, 0.0, 1.0) * (h - 1))
    y0 = max(0, py - radius)
    y1 = min(h, py + radius + 1)
    x0 = max(0, px - radius)
    x1 = min(w, px + radius + 1)
    region = depth_image[y0:y1, x0:x1].astype(np.float32)
    valid = region[np.isfinite(region) & (region > 0.0)]
    if len(valid) == 0:
        return None
    return float(np.median(valid))

def depth_to_meters(value, encoding):
    """深度值单位转换。

    16UC1/mono16（毫米）→ 除以 1000 → 米
    其他编码（如 32FC1，已是米）→ 直接返回

    Args:
        value: 原始深度值
        encoding: 图像编码字符串

    Returns:
        深度值（米）
    """
    if encoding in ("16UC1", "mono16"):
        return value / 1000.0
    return value
