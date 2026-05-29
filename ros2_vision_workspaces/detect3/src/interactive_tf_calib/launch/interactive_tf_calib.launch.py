from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parent_frame_arg = DeclareLaunchArgument(
        'parent_frame',
        default_value='camera_color_optical_frame',
    )
    child_frame_arg = DeclareLaunchArgument(
        'child_frame',
        default_value='livox_frame',
    )
    publish_hz_arg = DeclareLaunchArgument(
        'publish_hz',
        default_value='30.0',
    )
    save_path_arg = DeclareLaunchArgument(
        'save_path',
        default_value='~/lidar_to_camera_extrinsics.json',
    )
    initial_xyz_m_arg = DeclareLaunchArgument(
        'initial_xyz_m',
        default_value='[0.0, 0.0, 0.0]',
    )
    initial_rpy_rad_arg = DeclareLaunchArgument(
        'initial_rpy_rad',
        default_value='[0.0, 0.0, 0.0]',
    )

    color_topic_arg = DeclareLaunchArgument(
        'color_topic',
        default_value='/camera/color/image_raw',
    )
    color_info_topic_arg = DeclareLaunchArgument(
        'color_info_topic',
        default_value='/camera/color/camera_info',
    )
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/livox/lidar',
    )
    overlay_topic_arg = DeclareLaunchArgument(
        'overlay_topic',
        default_value='/lidar/overlay/image',
    )
    overlay_max_points_arg = DeclareLaunchArgument(
        'overlay_max_points',
        default_value='8000',
    )
    overlay_z_min_m_arg = DeclareLaunchArgument(
        'overlay_z_min_m',
        default_value='0.1',
    )
    overlay_z_max_m_arg = DeclareLaunchArgument(
        'overlay_z_max_m',
        default_value='20.0',
    )

    node = Node(
        package='interactive_tf_calib',
        executable='interactive_tf_calib_node',
        name='interactive_tf_calib',
        output='screen',
        parameters=[
            {
                'parent_frame': LaunchConfiguration('parent_frame'),
                'child_frame': LaunchConfiguration('child_frame'),
                'publish_hz': LaunchConfiguration('publish_hz'),
                'save_path': LaunchConfiguration('save_path'),
                'initial_xyz_m': LaunchConfiguration('initial_xyz_m'),
                'initial_rpy_rad': LaunchConfiguration('initial_rpy_rad'),
            }
        ],
    )

    overlay_node = Node(
        package='interactive_tf_calib',
        executable='lidar_overlay_node',
        name='lidar_overlay',
        output='screen',
        parameters=[
            {
                'color_topic': LaunchConfiguration('color_topic'),
                'color_info_topic': LaunchConfiguration('color_info_topic'),
                'lidar_topic': LaunchConfiguration('lidar_topic'),
                'overlay_topic': LaunchConfiguration('overlay_topic'),
                'camera_frame': LaunchConfiguration('parent_frame'),
                'lidar_frame': LaunchConfiguration('child_frame'),
                'max_points': LaunchConfiguration('overlay_max_points'),
                'z_min_m': LaunchConfiguration('overlay_z_min_m'),
                'z_max_m': LaunchConfiguration('overlay_z_max_m'),
            }
        ],
    )

    return LaunchDescription(
        [
            parent_frame_arg,
            child_frame_arg,
            publish_hz_arg,
            save_path_arg,
            initial_xyz_m_arg,
            initial_rpy_rad_arg,
            color_topic_arg,
            color_info_topic_arg,
            lidar_topic_arg,
            overlay_topic_arg,
            overlay_max_points_arg,
            overlay_z_min_m_arg,
            overlay_z_max_m_arg,
            node,
            overlay_node,
        ]
    )
