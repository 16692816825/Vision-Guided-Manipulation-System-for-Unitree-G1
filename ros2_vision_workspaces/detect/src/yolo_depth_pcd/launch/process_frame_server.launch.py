from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    weights_arg = DeclareLaunchArgument(
        'weights',
        default_value='/home/xinghg/detect/best.pt',
    )
    conf_arg = DeclareLaunchArgument(
        'conf',
        default_value='0.25',
    )

    viz_arg = DeclareLaunchArgument(
        'publish_viz',
        default_value='true',
    )
    viz_frame_arg = DeclareLaunchArgument(
        'viz_frame_id',
        default_value='world',
    )
    viz_img_frame_arg = DeclareLaunchArgument(
        'viz_image_frame_id',
        default_value='camera',
    )

    return LaunchDescription(
        [
            weights_arg,
            conf_arg,
            viz_arg,
            viz_frame_arg,
            viz_img_frame_arg,
            Node(
                package='yolo_depth_pcd',
                executable='process_frame_server',
                name='process_frame_server',
                output='screen',
                parameters=[
                    {
                        'weights': LaunchConfiguration('weights'),
                        'conf': LaunchConfiguration('conf'),
                        'class_name': '',
                        'class_id': -1,
                        'depth_scale': 1000.0,
                        'forward_neg_z': True,
                        'image_y_down': True,
                        'polygon_max_points': 40,
                        'publish_viz': LaunchConfiguration('publish_viz'),
                        'viz_frame_id': LaunchConfiguration('viz_frame_id'),
                        'viz_image_frame_id': LaunchConfiguration('viz_image_frame_id'),
                    }
                ],
            ),
        ]
    )
