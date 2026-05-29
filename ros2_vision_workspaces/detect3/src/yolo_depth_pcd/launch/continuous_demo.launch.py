from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('weights', default_value='/home/xinghg/detect/best.pt'),
            DeclareLaunchArgument('conf', default_value='0.25'),
            DeclareLaunchArgument('dataset_root', default_value='/home/xinghg/detect/datasets/demo1'),
            DeclareLaunchArgument('start_idx', default_value='0'),
            DeclareLaunchArgument('end_idx', default_value='46'),
            DeclareLaunchArgument('step', default_value='1'),
            DeclareLaunchArgument('rate_hz', default_value='1.0'),
            DeclareLaunchArgument('save_debug', default_value='false'),
            Node(
                package='yolo_depth_pcd',
                executable='process_frame_server',
                name='process_frame_server',
                output='screen',
                parameters=[
                    {
                        'weights': LaunchConfiguration('weights'),
                        'conf': LaunchConfiguration('conf'),
                        'publish_viz': True,
                        'viz_frame_id': 'world',
                    }
                ],
            ),
            Node(
                package='yolo_depth_pcd',
                executable='process_frame_client',
                name='process_frame_client',
                output='screen',
                parameters=[
                    {
                        'dataset_root': LaunchConfiguration('dataset_root'),
                        'start_idx': LaunchConfiguration('start_idx'),
                        'end_idx': LaunchConfiguration('end_idx'),
                        'step': LaunchConfiguration('step'),
                        'rate_hz': LaunchConfiguration('rate_hz'),
                        'save_debug': LaunchConfiguration('save_debug'),
                    }
                ],
            ),
        ]
    )
