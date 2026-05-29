from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os


def generate_launch_description():
    g1_description_pkg = get_package_share_directory("g1_description")
    g1_driver_pkg = get_package_share_directory("g1_driver_py")

    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true",
    )

    return LaunchDescription([
        use_rviz,
        # G1 模型加载（使用 robot_state_publisher 直接加载 URDF）
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            arguments=[
                os.path.join(g1_description_pkg, "g1_29dof.urdf")
            ],
        ),
        # 关节状态发布（由驱动节点发布，此处关闭 joint_state_publisher）
        # 驱动节点
        Node(
            package="g1_driver_py",
            executable="g1_driver",
            name="g1_driver_node",
            output="screen",
            parameters=[
                os.path.join(g1_driver_pkg, "params", "driver.yaml")
            ],
        ),
        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", os.path.join(g1_driver_pkg, "rviz", "display.rviz")]
            if os.path.exists(os.path.join(g1_driver_pkg, "rviz", "display.rviz"))
            else [],
        ),
        # 速度指令转换节点（Twist → unitree_api Request）
        Node(
            package="g1_twist_bridge_py",
            executable="g1_twist_bridge"
        ),
    ])
