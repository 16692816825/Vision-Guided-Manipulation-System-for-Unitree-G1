#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 手臂抓取控制 (armup)
========================
控制机器人手臂执行抓取动作：伸出 → 抬起 → 夹紧保持。

姿态序列：
  1. reach_forward — 伸手接近目标
  2. arms_up       — 抬起目标
  3. pray          — 夹紧保持（不释放 arm_sdk）

运行：
    python3 armup.py           # 自动检测网络接口
    python3 armup.py eth0      # 指定网络接口

依赖：
    pip install unitree_sdk2py numpy
"""

import time
import sys

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from arm_common import BaseArmController, build_timeline

def _pose_arms_up():
    """抬起姿态"""
    return [
        -1.0,   0.7,  0.0,  0.6, -0.8,
        -1.0,  -0.7,  0.0,  0.6,  0.8,
         0.0,   0.0,  0.0,
    ]

def _pose_pray():
    """夹紧保持姿态。"""
    return [
        -1.15,  0.5, -0.3,  0.3, -1.8,
        -1.15, -0.5,  0.3,  0.3,  1.8,
         0.0,   0.0,  0.0,
    ]

def _pose_reach_forward():
    """伸手接近目标姿态。"""
    return [
        -0.8,   0.5, -0.4,  0.15, -1.8,
        -0.8,  -0.5,  0.4,  0.15,  1.8,
         0.0,   0.0,  0.0,
    ]

POSE_SEQUENCE = [
    ("reach_forward",  _pose_reach_forward(),  3.0),
    ("arms_up",        _pose_arms_up(),        3.0),
    ("pray",           _pose_pray(),           3.0),
]

class GrabController(BaseArmController):
    """G1 抓取控制器 — 伸手 → 抬起 → 夹紧保持。

    完成后不释放 arm_sdk，机器人保持夹紧姿态等待放下指令。
    """

    def _build_timeline(self):
        self.timeline, self.total_time = build_timeline(
            self.poses, self.transition_time,
            include_init_zero=True, include_final_zero=False, include_release=False,
        )

    def _on_complete(self):
        """保持最后的夹紧姿态，不释放 arm_sdk。"""
        last_angles = self.poses[-1][1]
        self._send_joint_cmd(last_angles)
        if not self.done:
            self.done = True
            print("\n  [抓取完成] 保持夹紧姿态，等待放下指令")

if __name__ == '__main__':
    print("WARNING: 确保机器人周围无障碍物!")
    if sys.stdin.isatty():
        input("按 Enter 开始抓取...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    ctrl = GrabController(POSE_SEQUENCE)
    ctrl.Init()
    ctrl.Start()

    start = time.time()
    while not ctrl.done:
        time.sleep(1)
        elapsed = time.time() - start
        progress = min(ctrl.time / ctrl.total_time * 100, 100) if ctrl.total_time > 0 else 0
        print(f"\r  进度: {progress:5.1f}%  ({elapsed:.0f}s/{ctrl.total_time:.0f}s)",
              end="", flush=True)

    total = time.time() - start
    print(f"\r  进度: 100.0%  ({total:.0f}s/{ctrl.total_time:.0f}s)")
    print("  抓取完成，保持夹紧中...")
