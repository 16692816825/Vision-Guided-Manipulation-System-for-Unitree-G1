"""Launch 文件：启动 YOLO 目标识别 + 路径规划导航。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_dir = get_package_share_directory("g1_yolo_nav_py")
    config_file = os.path.join(pkg_dir, "config", "yolo_nav.yaml")

    # ---- Launch 参数 ----
    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="false",
        description="是否启动 RViz2",
    )
    use_nav2 = DeclareLaunchArgument(
        name="use_nav2",
        default_value="false",
        description="是否使用 Nav2 导航（false 时使用简单趋近模式）",
    )
    model_path = DeclareLaunchArgument(
        name="model_path",
        default_value="yolo_v11x_best.pt",
        description="YOLO 模型文件路径",
    )
    use_depth = DeclareLaunchArgument(
        name="use_depth_sensor",
        default_value="false",
        description="是否使用深度传感器",
    )
    target_class = DeclareLaunchArgument(
        name="target_class",
        default_value="0",
        description="目标类别 ID（COCO: 0=person, 1=bicycle, ...）",
    )

    # ---- 节点 ----
    yolo_detector_node = Node(
        package="g1_yolo_nav_py",
        executable="yolo_detector",
        name="g1_yolo_detector_node",
        output="screen",
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
        output="screen",
        parameters=[
            config_file,
            {"use_depth_sensor": LaunchConfiguration("use_depth_sensor")},
        ],
    )

    nav_planner_node = Node(
        package="g1_yolo_nav_py",
        executable="nav_planner",
        name="g1_nav_planner_node",
        output="screen",
        parameters=[
            config_file,
            {"use_nav2": LaunchConfiguration("use_nav2")},
        ],
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
        # 参数声明
        use_rviz,
        use_nav2,
        model_path,
        use_depth,
        target_class,
        # 节点
        yolo_detector_node,
        spatial_target_node,
        nav_planner_node,
        rviz_node,
    ])
