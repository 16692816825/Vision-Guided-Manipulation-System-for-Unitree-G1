"""步进式旋转对齐器 — 从 yaw_align.py 提取的对齐逻辑。

控制逻辑（与 yaw_align.py 完全一致）：
    1. 检测目标位置 u（归一化 0~1，0.5 = 画面中央）
    2. 若目标偏离中心，发送一次短时间小幅度旋转（move(vyaw=..., duration=...)）
    3. 等待 camera_settle_time 让相机更新
    4. 重新检测目标位置，重复步骤 2~3
    5. 目标居中后停止

用法：
    def _do_move(vyaw, duration):
        sport.move(vyaw=vyaw, duration=duration)

    aligner = StepAligner(move_fn=_do_move, logger=logger)
    action, extra = aligner.tick(target_u=0.3)

    if action == AlignAction.ROTATING:
        pass  # 已发送旋转指令
    elif action == AlignAction.ALIGNED:
        print("已对齐")
"""

from enum import Enum, auto
from typing import Optional, Callable, Tuple, Any

class AlignAction(Enum):
    """StepAligner.tick() 返回的动作类型。"""
    ROTATING = auto()   # 已发送旋转指令，正在等待相机更新
    ALIGNED = auto()    # 目标已居中（误差 < 容差）
    LOST = auto()       # 目标丢失或超时
    WAIT = auto()       # 等待中（settling 或目标居中稳定）

class StepAligner:
    """步进式旋转对齐器 — 每次移动一小步，等相机更新后再决定下一步。

    逻辑与 yaw_align.py._tick() 完全一致。
    """

    def __init__(
        self,
        move_fn: Callable[[float, Optional[float]], None],
        *,
        logger: Any = None,
        center_tolerance: float = 0.08,
        step_yaw_speed: float = 0.3,
        step_duration: float = 0.8,
        camera_settle_time: float = 2.0,
        max_consecutive_steps: int = 10,
    ) -> None:
        self._move_fn = move_fn
        self._log = logger

        self._center_tol = center_tolerance
        self._step_speed = step_yaw_speed
        self._step_dur = step_duration
        self._settle_time = camera_settle_time
        self._max_steps = max_consecutive_steps

        self._target_u: Optional[float] = None
        self._settling: bool = False
        self._settle_start: float = 0.0
        self._step_count: int = 0

    @property
    def is_centered(self) -> bool:
        """目标当前是否居中（不触发旋转）。"""
        if self._target_u is None:
            return False
        return abs(self._target_u - 0.5) < self._center_tol

    @property
    def settling(self) -> bool:
        """是否正在等待相机更新。"""
        return self._settling

    @property
    def settle_start(self) -> float:
        """settling 开始的时间戳。"""
        return self._settle_start

    def tick(self, target_u: Optional[float]) -> Tuple[AlignAction, str]:
        """执行一步对齐逻辑（与 yaw_align.py._tick 完全一致）。

        Args:
            target_u: 目标归一化 u 坐标（0~1），None 表示未检测到。

        Returns:
            (action, extra) — extra 为日志信息或错误描述。
        """
        import time
        import math

        self._target_u = target_u
        now = time.time()

        if target_u is None:
            if self._settling:
                self._move_fn(0.0, None)
                self._settling = False
                self._step_count = 0
                return AlignAction.LOST, "等待中目标丢失，停止旋转"
            return AlignAction.WAIT, ""

        if self._settling:
            elapsed = now - self._settle_start
            if elapsed < self._settle_time:
                return AlignAction.WAIT, ""
            self._settling = False

        error = target_u - 0.5

        if abs(error) < self._center_tol:
            self._move_fn(0.0, None)
            return AlignAction.ALIGNED, (
                f"目标已居中: u={target_u:.3f}, "
                f"误差={error:.3f} < 容差={self._center_tol}"
            )

        vyaw = -self._step_speed if error > 0 else self._step_speed
        self._move_fn(vyaw, self._step_dur)
        self._step_count += 1

        turn_deg = math.degrees(vyaw * self._step_dur)
        msg = (
            f"第{self._step_count}步: u={target_u:.3f}, "
            f"误差={error:+.3f}, 旋转≈{turn_deg:+.1f}°, "
            f"等待{self._settle_time}s..."
        )

        self._settling = True
        self._settle_start = now

        if self._step_count >= self._max_steps:
            self._move_fn(0.0, None)
            self._settling = False
            self._step_count = 0
            return AlignAction.LOST, f"已连续旋转 {self._max_steps} 步仍未居中，停止"

        return AlignAction.ROTATING, msg

    def stop(self) -> None:
        """停止旋转。"""
        self._move_fn(0.0, None)

    def reset(self) -> None:
        """重置内部状态。"""
        self._settling = False
        self._settle_start = 0.0
        self._step_count = 0
