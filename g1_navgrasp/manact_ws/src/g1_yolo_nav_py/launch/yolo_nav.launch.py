"""
Launch 文件：启动 YOLO 目标识别 + 偏航对齐 + 前进接近
需手动将机器人切换到走跑模式后再启动。

启动节点：
    1. g1_yolo_detector_node     - YOLO 目标检测
    2. g1_yaw_align_node         - 偏航对齐（StepAligner 步进式旋转）
    3. g1_loco_forward_node      - Loco API 前进控制
    4. d455_camera_tf_publisher   - D455 相机静态 TF

使用示例：
    ros2 launch g1_yolo_nav_py yolo_nav.launch.py
    ros2 launch g1_yolo_nav_py yolo_nav.launch.py use_depth_sensor:=true
    ros2 launch g1_yolo_nav_py yolo_nav.launch.py target_class:=person model_path:=yolov8n.pt

TF 树结构：
    odom → base_link → pelvis → torso_link → robot1/D455_1_link → camera_optical_frame
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    """生成 Launch 描述。"""
    pkg_dir = get_package_share_directory("g1_yolo_nav_py")
    config_file = os.path.join(pkg_dir, "config", "yolo_nav.yaml")

    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="false",
        description="是否启动 RViz2 可视化界面",
    )

    model_path_arg = DeclareLaunchArgument(
        name="model_path",
        default_value="yolo_v11s_best.pt",
        description="YOLO 模型文件名（相对于 share/models/ 目录，或绝对路径）",
    )

    use_depth = DeclareLaunchArgument(
        name="use_depth_sensor",
        default_value="false",
        description="是否使用深度传感器（false 时使用默认距离）",
    )

    target_class = DeclareLaunchArgument(
        name="target_class",
        default_value="chair",
        description="目标类别名称（如 chair, person，也支持数字 ID）",
    )

    enable_approach = DeclareLaunchArgument(
        name="enable_approach",
        default_value="true",
        description="是否启用偏航对齐+前进接近",
    )

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

    yolo_detector_node = Node(
        package="g1_yolo_nav_py",
        executable="yolo_detector",
        name="g1_yolo_detector_node",
        parameters=[
            config_file,
            {"model_path": LaunchConfiguration("model_path")},
            {"target_classes": [LaunchConfiguration("target_class")]},
        ],
    )

    spatial_target_node = Node(
        package="g1_yolo_nav_py",
        executable="spatial_target",
        name="g1_spatial_target_node",
        parameters=[
            config_file,
            {"use_depth_sensor": LaunchConfiguration("use_depth_sensor")},
        ],
    )

    # 偏航对齐节点（通过 Loco API SET_VELOCITY 旋转机器人）
    yaw_align_node = Node(
        package="g1_yolo_nav_py",
        executable="yaw_align",
        name="g1_yaw_align_node",
        condition=IfCondition(LaunchConfiguration("enable_approach")),
        parameters=[config_file],
    )

    # 前进控制节点（Loco API SET_VELOCITY）
    loco_forward_node = Node(
        package="g1_yolo_nav_py",
        executable="loco_forward",
        name="g1_loco_forward_node",
        condition=IfCondition(LaunchConfiguration("enable_approach")),
        parameters=[config_file],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=["-d", os.path.join(pkg_dir, "rviz", "yolo_nav.rviz")]
        if os.path.exists(os.path.join(pkg_dir, "rviz", "yolo_nav.rviz"))
        else [],
    )

    return LaunchDescription([
        use_rviz,
        model_path_arg,
        use_depth,
        target_class,
        enable_approach,
        camera_static_tf,
        yolo_detector_node,
        spatial_target_node,
        yaw_align_node,
        loco_forward_node,
        rviz_node,
    ])
