#!/usr/bin/env python3
import subprocess
import sys
import time
import atexit
import signal
import os

# ===================== 顺序修改 =====================
# 1. 第一个执行：ROS2 键盘控制节点
ROS2_PACKAGE = "ctrl_keyboard"
ROS2_NODE = "ctrl_keyboard"

# 2. 第二个执行：机械臂脚本
SCRIPT1 = "/home/unitree/g1act_ws/g1act_ws/src/arm/armtest.py"

# 3. 第三个执行：手控脚本
SCRIPT2 = "/home/unitree/hand/python/revo2/revo2_simple_control.py"

# 存储进程对象
proc1 = None  # SCRIPT1 进程
ros2_proc = None  # ROS2 键盘节点进程

def cleanup():
    """退出时清理所有子进程"""
    global ros2_proc, proc1
    # 清理 ROS2 进程
    if ros2_proc and ros2_proc.poll() is None:
        print("正在终止 ROS2 键盘控制进程...")
        ros2_proc.terminate()
        try:
            ros2_proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            ros2_proc.kill()
    # 清理 SCRIPT1 进程
    if proc1 and proc1.poll() is None:
        print("正在终止机械臂脚本进程...")
        proc1.terminate()
        try:
            proc1.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc1.kill()
    print("所有进程已清理。")

def main():
    global ros2_proc, proc1

    # ===================== 第一步：启动 ROS2 键盘控制 =====================
    print("=" * 60)
    print("第一步：启动 ROS2 键盘控制节点")
    print("命令：ros2 run ctrl_keyboard ctrl_keyboard")
    print("=" * 60)

    # 启动 ROS2 节点（阻塞运行，直到它退出才继续下一步）
    try:
        ros2_proc = subprocess.Popen(
            ["ros2", "run", ROS2_PACKAGE, ROS2_NODE],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        # 等待 ROS2 节点运行结束（必须关闭它才会往下走）
        ros2_returncode = ros2_proc.wait()
        print(f"\nROS2 键盘控制节点已退出，返回码: {ros2_returncode}")

    except FileNotFoundError:
        print("错误：未找到 ros2 命令，请检查 ROS2 环境是否正确加载！")
        sys.exit(1)
    except Exception as e:
        print(f"启动 ROS2 节点失败: {e}")
        sys.exit(1)

    # ===================== 第二步：启动机械臂脚本 =====================
    print("\n" + "=" * 60)
    print("第二步：启动机械臂脚本（后台运行）")
    print("=" * 60)

    proc1 = subprocess.Popen(
        [sys.executable, SCRIPT1],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid
    )
    print("机械臂脚本已在后台启动")

    # 等待初始化
    time.sleep(1)
    print("等待 4 秒确保初始化完成...")
    time.sleep(4)

    # ===================== 第三步：启动手控脚本 =====================
    print("\n" + "=" * 60)
    print("第三步：启动手部控制脚本")
    print("=" * 60)

    try:
        subprocess.run([sys.executable, SCRIPT2], check=True)
        print("\n手部脚本执行完毕！")
    except subprocess.CalledProcessError as e:
        print(f"\n手部脚本执行失败，返回码: {e.returncode}")
    except FileNotFoundError:
        print(f"\n错误：未找到脚本: {SCRIPT2}")

    # 等待机械臂脚本结束
    print("\n等待机械臂脚本运行结束...")
    proc1.wait()
    print("所有脚本执行完成！")

if __name__ == "__main__":
    atexit.register(cleanup)
    signal.signal(signal.SIGINT, lambda sig, frame: (print("\n收到退出信号"), cleanup(), sys.exit(0)))
    main()

