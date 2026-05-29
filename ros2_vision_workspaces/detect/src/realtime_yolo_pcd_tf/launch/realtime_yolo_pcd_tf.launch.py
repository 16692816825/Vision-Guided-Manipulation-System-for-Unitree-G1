from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('weights', default_value=''),
            DeclareLaunchArgument('conf', default_value='0.25'),
            DeclareLaunchArgument('class_name', default_value='handle'),
            DeclareLaunchArgument('class_id', default_value='-1'),
            DeclareLaunchArgument('device', default_value=''),
            DeclareLaunchArgument('half', default_value='false'),
            DeclareLaunchArgument('color_topic', default_value='/camera/color/image_raw'),
            DeclareLaunchArgument('depth_topic', default_value='/camera/aligned_depth_to_color/image_raw'),
            DeclareLaunchArgument('color_info_topic', default_value='/camera/color/camera_info'),
            DeclareLaunchArgument('depth_scale', default_value='1000.0'),
            DeclareLaunchArgument('sync_slop_sec', default_value='0.3'),
            DeclareLaunchArgument('publish_every_n', default_value='1'),
            DeclareLaunchArgument('yolo_every_n', default_value='2'),
            DeclareLaunchArgument('cloud_stride', default_value='4'),
            DeclareLaunchArgument('extrinsics_json', default_value='/home/unitree/detect/robot_center_to_camera.json'),
            DeclareLaunchArgument('tf_publish_hz', default_value='10.0'),
            DeclareLaunchArgument('output_frame', default_value='robot_center'),
            DeclareLaunchArgument('camera_frame', default_value='camera_color_optical_frame'),
            Node(
                package='realtime_yolo_pcd_tf',
                executable='realtime_node',
                name='realtime_yolo_pcd_tf',
                output='screen',
                parameters=[
                    {
                        'weights': LaunchConfiguration('weights'),
                        'conf': LaunchConfiguration('conf'),
                        'class_name': LaunchConfiguration('class_name'),
                        'class_id': LaunchConfiguration('class_id'),
                        'device': LaunchConfiguration('device'),
                        'half': LaunchConfiguration('half'),
                        'color_topic': LaunchConfiguration('color_topic'),
                        'depth_topic': LaunchConfiguration('depth_topic'),
                        'color_info_topic': LaunchConfiguration('color_info_topic'),
                        'depth_scale': LaunchConfiguration('depth_scale'),
                        'sync_slop_sec': LaunchConfiguration('sync_slop_sec'),
                        'publish_every_n': LaunchConfiguration('publish_every_n'),
                        'yolo_every_n': LaunchConfiguration('yolo_every_n'),
                        'cloud_stride': LaunchConfiguration('cloud_stride'),
                        'extrinsics_json': LaunchConfiguration('extrinsics_json'),
                        'tf_publish_hz': LaunchConfiguration('tf_publish_hz'),
                        'output_frame': LaunchConfiguration('output_frame'),
                        'camera_frame': LaunchConfiguration('camera_frame'),
                    }
                ],
            ),
        ]
    )
