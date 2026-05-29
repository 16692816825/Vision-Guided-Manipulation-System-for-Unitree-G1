#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 手臂放下控制 (armdown)
===========================
控制机器人手臂执行放下动作：从夹紧姿态 → 下放 → 归零释放。

姿态序列：
  1. wave       — 伸展下放
  2. wave_body  — 自然下垂
  最后归零并释放 arm_sdk

运行：
    python3 armdown.py           # 自动检测网络接口
    python3 armdown.py eth0      # 指定网络接口

依赖：
    pip install unitree_sdk2py numpy
"""

import time
import sys

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from arm_common import BaseArmController, build_timeline

def _pose_wave():
    """伸展下放姿态。"""
    return [
        -1.1,   0.55,  -0.45,   0.2,  -1.8,
        -1.1,  -0.55,   0.45,   0.2,   1.8,
         0.0,   0.0,    0.0,
    ]

def _pose_wave_body():
    """自然下垂姿态。"""
    return [
        -0.7,   0.7,   0.0,   0.6,  -0.8,
        -0.7,  -0.7,   0.0,   0.6,   0.8,
         0.0,   0.0,   0.0,
    ]

# 放下序列：伸展下放 → 自然下垂（之后归零释放）
POSE_SEQUENCE = [
    ("wave",       _pose_wave(),       3.0),
    ("wave_body",  _pose_wave_body(),  3.0),
]

class ReleaseController(BaseArmController):
    """G1 放下控制器 — 从夹紧姿态 → 下放 → 归零 → 释放 arm_sdk。

    完成后归零并释放 arm_sdk，机器人回到默认待机状态。
    """

    def _build_timeline(self):
        self.timeline, self.total_time = build_timeline(
            self.poses, self.transition_time,
            include_init_zero=False, include_final_zero=True, include_release=True,
        )

if __name__ == '__main__':
    print("WARNING: 确保机器人周围无障碍物!")
    if sys.stdin.isatty():
        input("按 Enter 开始放下...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    ctrl = ReleaseController(POSE_SEQUENCE)
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
    print("  放下完成，arm_sdk 已释放")
