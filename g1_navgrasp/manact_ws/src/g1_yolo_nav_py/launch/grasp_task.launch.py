"""
Launch 文件：G1 抓取任务一键全流程
===================================
从相机驱动到抓取完成，一条命令搞定。
需手动将机器人切换到走跑模式后再启动。

启动节点：
    1. RealSense D455 相机驱动
    2. g1_yolo_detector_node    - YOLO 目标检测
    3. g1_grasp_task_node       - 抓取任务主控（搜索→StepAligner 对齐→ForwardApproach 接近→抓取→菜单）
    4. D455 相机静态 TF

使用示例：
    ros2 launch g1_yolo_nav_py grasp_task.launch.py

    ros2 launch g1_yolo_nav_py grasp_task.launch.py target_class:=bottle

    ros2 launch g1_yolo_nav_py grasp_task.launch.py \\
        network_interface:=eth0 forward_speed:=0.15

    ros2 launch g1_yolo_nav_py grasp_task.launch.py start_camera:=false
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    pkg_dir = get_package_share_directory("g1_yolo_nav_py")
    config_file = os.path.join(pkg_dir, "config", "yolo_nav.yaml")

    start_camera = DeclareLaunchArgument(
        name="start_camera",
        default_value="true",
        description="是否启动 RealSense 相机驱动",
    )
    target_class = DeclareLaunchArgument(
        name="target_class",
        default_value="chair",
        description="YOLO 检测目标类别",
    )
    model_path_arg = DeclareLaunchArgument(
        name="model_path",
        default_value="yolo_v11s_best.pt",
        description="YOLO 模型文件名",
    )
    network_interface = DeclareLaunchArgument(
        name="network_interface",
        default_value="",
        description="网卡接口名（空=自动检测）",
    )
    forward_speed = DeclareLaunchArgument(
        name="forward_speed",
        default_value="0.2",
        description="接近速度 m/s",
    )
    arrive_bbox_ratio = DeclareLaunchArgument(
        name="arrive_bbox_ratio",
        default_value="0.45",
        description="到达判定阈值（检测框占比）",
    )
    arm_script_dir = DeclareLaunchArgument(
        name="arm_script_dir",
        default_value=os.path.expanduser(
            "~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm"
        ),
        description="arm 脚本目录（armup.py / armdown.py 所在目录）",
    )

    camera_launch = ExecuteProcess(
        cmd=[
            "ros2", "launch", "realsense2_camera", "rs_launch.py",
            "camera_namespace:=robot1",
            "camera_name:=D455_1",
            "align_depth.enable:=true",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )

    # 2. D455 相机静态 TF（torso_link → camera link）
    camera_pitch_deg = -42.0
    camera_pitch_rad = math.radians(camera_pitch_deg)
    q_x = math.sin(camera_pitch_rad / 2.0)
    q_w = math.cos(camera_pitch_rad / 2.0)

    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d455_camera_tf_publisher",
        arguments=[
            "--x", "0.04765",
            "--y", "0.0",
            "--z", "0.0",
            "--qx", str(q_x),
            "--qy", "0.0",
            "--qz", "0.0",
            "--qw", str(q_w),
            "--frame-id", "torso_link",
            "--child-frame-id", "robot1/D455_1_link",
        ],
    )

    #    aarch64 上需要 LD_PRELOAD，ros2 launch 的 Node 无法设置环境变量，
    #    因此用 ExecuteProcess 代替 Node，手动注入 LD_PRELOAD。
    # 检测是否为 aarch64（机器人端）
    import platform
    _is_aarch64 = platform.machine() == "aarch64"

    if _is_aarch64:
        yolo_detector_cmd = ExecuteProcess(
            cmd=[
                "python3", "-m", "g1_yolo_nav_py.yolo_detector",
                "--ros-args",
                "--params-file", config_file,
                "-p", ["model_path:=", LaunchConfiguration("model_path")],
                "-p", ["target_classes:=[", LaunchConfiguration("target_class"), "]"],
            ],
            additional_env={
                "LD_PRELOAD": os.path.expanduser(
                    "~/.local/lib/python3.8/site-packages/torch.libs/libgomp-804f19d4.so.1.0.0"
                ),
                "DISPLAY": os.environ.get("DISPLAY", ":0"),
            },
            output="screen",
            name="g1_yolo_detector_node",
        )
    else:
        yolo_detector_cmd = Node(
            package="g1_yolo_nav_py",
            executable="yolo_detector",
            name="g1_yolo_detector_node",
            parameters=[
                config_file,
                {"model_path": LaunchConfiguration("model_path")},
                {"target_classes": [LaunchConfiguration("target_class")]},
            ],
        )

    grasp_task_node = Node(
        package="g1_yolo_nav_py",
        executable="grasp_task",
        name="g1_grasp_task_node",
        output="screen",  # 菜单交互需要看到输出
        parameters=[
            config_file,
            {
                "target_class_id": LaunchConfiguration("target_class"),
                "forward_speed": LaunchConfiguration("forward_speed"),
                "arrive_bbox_ratio": LaunchConfiguration("arrive_bbox_ratio"),
                "network_interface": LaunchConfiguration("network_interface"),
                "arm_script_dir": LaunchConfiguration("arm_script_dir"),
            },
        ],
    )

    return LaunchDescription([
        start_camera,
        target_class,
        model_path_arg,
        network_interface,
        forward_speed,
        arrive_bbox_ratio,
        arm_script_dir,
        camera_launch,
        camera_static_tf,
        yolo_detector_cmd,
        grasp_task_node,
    ])
