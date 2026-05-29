"""前进接近器 — 从 loco_forward.py 提取的前进逻辑。

控制逻辑（与 loco_forward.py 完全一致）：
    1. 检测目标是否居中（误差 < 容差）
    2. 居中稳定后开始前进
    3. 深度距离或 bbox 占比到达时停止
    4. 目标偏离过大时停止前进

用法：
    approach = ForwardApproach(
        move_fn=sport.move,
        stop_fn=sport.stop,
        use_depth=True,
        stop_distance=0.5,
        arrive_bbox_ratio=0.45,
    )
    action, msg = approach.tick(
        target_u=0.5, target_distance=0.8,
        bbox_size_x=0.3, bbox_size_y=0.3,
    )
"""

from enum import Enum, auto
from typing import Optional, Callable, Tuple, Any

class ApproachAction(Enum):
    """ForwardApproach.tick() 返回的动作类型。"""
    APPROACHING = auto()  # 正在前进
    ARRIVED = auto()      # 已到达目标
    DRIFTED = auto()      # 目标偏离过大，需重新对齐
    WAIT = auto()         # 等待中（目标丢失、未居中、未稳定）

class ForwardApproach:
    """前进接近器 — 居中稳定后前进，到达后停止。

    逻辑与 loco_forward.py._tick() 的前进部分一致。
    """

    def __init__(
        self,
        move_fn: Callable,
        stop_fn: Callable,
        *,
        logger: Any = None,
        forward_speed: float = 0.2,
        center_tolerance: float = 0.08,
        align_stable_time: float = 0.8,
        use_depth: bool = True,
        stop_distance: float = 0.5,
        arrive_bbox_ratio: float = 0.45,
        drifted_ratio: float = 2.0,
    ) -> None:
        self._move_fn = move_fn
        self._stop_fn = stop_fn
        self._log = logger

        self._speed = forward_speed
        self._center_tol = center_tolerance
        self._stable_time = align_stable_time
        self._use_depth = use_depth
        self._stop_dist = stop_distance
        self._arrive_ratio = arrive_bbox_ratio
        self._drifted_ratio = drifted_ratio

        self._aligned_start: Optional[float] = None
        self._approaching: bool = False

    @property
    def approaching(self) -> bool:
        """是否正在前进。"""
        return self._approaching

    def tick(
        self,
        target_u: Optional[float],
        target_distance: Optional[float],
        bbox_size_x: float,
        bbox_size_y: float,
    ) -> Tuple[ApproachAction, str]:
        """执行一步前进逻辑。

        Args:
            target_u: 目标归一化 u 坐标，None 表示未检测到。
            target_distance: 深度距离（米），None 表示不可用。
            bbox_size_x: 检测框宽度（归一化）。
            bbox_size_y: 检测框高度（归一化）。

        Returns:
            (action, msg)
        """
        import time
        now = time.time()

        if target_u is None:
            self.stop()
            return ApproachAction.WAIT, ""

        error = abs(target_u - 0.5)

        if error > self._center_tol:
            self.stop()
            return ApproachAction.WAIT, ""

        if self._aligned_start is None:
            self._aligned_start = now

        if now - self._aligned_start < self._stable_time:
            return ApproachAction.WAIT, ""

        if self._use_depth and target_distance is not None:
            if target_distance <= self._stop_dist:
                self.stop()
                return ApproachAction.ARRIVED, (
                    f"到达目标 (深度={target_distance:.2f}m <= {self._stop_dist:.2f}m)"
                )

        bbox_max = max(bbox_size_x, bbox_size_y)
        if bbox_max >= self._arrive_ratio:
            self.stop()
            return ApproachAction.ARRIVED, (
                f"到达目标 (bbox={bbox_max:.2f} >= {self._arrive_ratio})"
            )

        if error > self._center_tol * self._drifted_ratio:
            self.stop()
            return ApproachAction.DRIFTED, "目标偏离，需重新对齐"

        self._move_fn(vx=self._speed)
        self._approaching = True
        return ApproachAction.APPROACHING, ""

    def stop(self) -> None:
        """停止前进。"""
        if self._approaching:
            self._stop_fn()
            self._approaching = False
        self._aligned_start = None

    def reset(self) -> None:
        """重置内部状态。"""
        self._aligned_start = None
        self._approaching = False
