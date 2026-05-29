from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    board_meta_arg = DeclareLaunchArgument('board_meta', default_value='')
    dict_arg = DeclareLaunchArgument('dict', default_value='APRILTAG_36H11')

    color_topic_arg = DeclareLaunchArgument('color_topic', default_value='/camera/camera/color/image_raw')
    camera_info_topic_arg = DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info')

    assume_parallel_arg = DeclareLaunchArgument('assume_board_parallel_robot', default_value='false')
    robot_board_center_xyz_arg = DeclareLaunchArgument('robot_board_center_xyz', default_value='')
    robot_tag_id_arg = DeclareLaunchArgument('robot_tag_id', default_value='-1')
    robot_tag_xyz_arg = DeclareLaunchArgument('robot_tag_xyz', default_value='')
    robot_board_xyz_arg = DeclareLaunchArgument('robot_board_xyz', default_value='0,0,0')
    robot_board_quat_arg = DeclareLaunchArgument('robot_board_quat_xyzw', default_value='0,0,0,1')

    save_cam_info_arg = DeclareLaunchArgument('save_camera_info_json', default_value='')

    save_robot_cam_arg = DeclareLaunchArgument('save_robot_cam_json', default_value='')
    save_robot_cam_every_n_arg = DeclareLaunchArgument('save_robot_cam_json_every_n', default_value='1')

    parent_frame_arg = DeclareLaunchArgument('parent_frame', default_value='robot_center')
    child_frame_arg = DeclareLaunchArgument('child_frame', default_value='')

    publish_tf_arg = DeclareLaunchArgument('publish_tf', default_value='true')
    publish_rate_hz_arg = DeclareLaunchArgument('publish_rate_hz', default_value='10.0')

    return LaunchDescription(
        [
            board_meta_arg,
            dict_arg,
            color_topic_arg,
            camera_info_topic_arg,
            assume_parallel_arg,
            robot_board_center_xyz_arg,
            robot_tag_id_arg,
            robot_tag_xyz_arg,
            robot_board_xyz_arg,
            robot_board_quat_arg,
            save_cam_info_arg,
            save_robot_cam_arg,
            save_robot_cam_every_n_arg,
            parent_frame_arg,
            child_frame_arg,
            publish_tf_arg,
            publish_rate_hz_arg,
            Node(
                package='apriltag_static_calib',
                executable='apriltag_static_calib_node',
                name='apriltag_static_calib_node',
                output='screen',
                parameters=[
                    {
                        'board_meta': LaunchConfiguration('board_meta'),
                        'dict': LaunchConfiguration('dict'),
                        'color_topic': LaunchConfiguration('color_topic'),
                        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                        'assume_board_parallel_robot': LaunchConfiguration('assume_board_parallel_robot'),
                        'robot_board_center_xyz': LaunchConfiguration('robot_board_center_xyz'),
                        'robot_tag_id': LaunchConfiguration('robot_tag_id'),
                        'robot_tag_xyz': LaunchConfiguration('robot_tag_xyz'),
                        'robot_board_xyz': LaunchConfiguration('robot_board_xyz'),
                        'robot_board_quat_xyzw': LaunchConfiguration('robot_board_quat_xyzw'),
                        'save_camera_info_json': LaunchConfiguration('save_camera_info_json'),
                        'save_robot_cam_json': LaunchConfiguration('save_robot_cam_json'),
                        'save_robot_cam_json_every_n': LaunchConfiguration('save_robot_cam_json_every_n'),
                        'parent_frame': LaunchConfiguration('parent_frame'),
                        'child_frame': LaunchConfiguration('child_frame'),
                        'publish_tf': LaunchConfiguration('publish_tf'),
                        'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                    }
                ],
            ),
        ]
    )
