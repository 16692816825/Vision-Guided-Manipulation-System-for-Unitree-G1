#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 人形机器人手臂多姿态演示
===========================
在 arm_common.py 基础上，支持定义多个目标姿态，手臂在姿态之间平滑循环切换。

预设姿态：见 POSE_SEQUENCE（5 个姿态循环切换）

运行方式：
    python3 arm_multi_pose_demo.py              # 自动检测网络接口
    python3 arm_multi_pose_demo.py eth0         # 指定网络接口

依赖：
    pip install unitree_sdk2py numpy
"""

import time
import sys

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from arm_common import BaseArmController, build_timeline

def _pose_arms_up():
    """姿态 1：预备姿势"1" """
    return [
        -1.0,   0.7,  0.0,  0.6, -0.8,
        -1.0,  -0.7,  0.0,  0.6,  0.8,
         0.0,   0.0,  0.0,
    ]

def _pose_pray():
    """姿态 2：抓取"2" """
    return [
        -1.15,  0.5, -0.3,  0.3, -1.8,
        -1.15, -0.5,  0.3,  0.3,  1.8,
         0.0,   0.0,  0.0,
    ]

def _pose_wave():
    """姿态 3：保持"3" """
    return [
        -1.1,   0.55, -0.45,  0.2, -1.8,
        -1.1,  -0.55,  0.45,  0.2,  1.8,
         0.0,   0.0,   0.0,
    ]

def _pose_reach_forward():
    """姿态 4：放下"4" """
    return [
        -0.8,   0.5, -0.4,  0.15, -1.8,
        -0.8,  -0.5,  0.4,  0.15,  1.8,
         0.0,   0.0,  0.0,
    ]

def _pose_wave_body():
    """姿态 5：松手"5" """
    return [
        -0.7,   0.7,  0.0,  0.6, -0.8,
        -0.7,  -0.7,  0.0,  0.6,  0.8,
         0.0,   0.0,  0.0,
    ]

# 姿态列表：名称 + 角度数组 + 保持时间(秒)
POSE_SEQUENCE = [
    ("1",  _pose_arms_up(),         2.0),
    ("2",  _pose_pray(),            2.0),
    ("3",  _pose_reach_forward(),   3.0),
    ("4",  _pose_wave(),            3.0),
    ("5",  _pose_wave_body(),       3.0),
]

class MultiPoseController(BaseArmController):
    """G1 手臂多姿态循环控制器。

    完成所有姿态后归零并释放 arm_sdk。
    """

    def _build_timeline(self):
        self.timeline, self.total_time = build_timeline(
            self.poses, self.transition_time,
            include_init_zero=True, include_final_zero=True, include_release=True,
        )

if __name__ == '__main__':
    input("按 Enter 继续...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    ctrl = MultiPoseController(POSE_SEQUENCE)
    ctrl.Init()
    ctrl.Start()

    start = time.time()
    while True:
        time.sleep(1)
        elapsed = time.time() - start
        progress = min(ctrl.time / ctrl.total_time * 100, 100) if ctrl.total_time > 0 else 0
        print(f"\r  进度: {progress:5.1f}%  ({elapsed:.0f}s/{ctrl.total_time:.0f}s)", end="", flush=True)

        if ctrl.done:
            total = time.time() - start
            print(f"\r  进度: 100.0%  ({total:.0f}s/{ctrl.total_time:.0f}s)")
            print("\n  Done!")
            sys.exit(0)
