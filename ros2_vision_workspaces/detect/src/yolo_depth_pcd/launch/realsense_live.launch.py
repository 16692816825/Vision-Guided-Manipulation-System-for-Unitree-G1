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

    color_topic_arg = DeclareLaunchArgument(
        'color_topic',
        default_value='/camera/camera/color/image_raw',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/camera/aligned_depth_to_color/image_raw',
    )
    color_info_topic_arg = DeclareLaunchArgument(
        'color_info_topic',
        default_value='/camera/camera/color/camera_info',
    )

    depth_scale_arg = DeclareLaunchArgument(
        'depth_scale',
        default_value='1000.0',
    )

    publish_viz_arg = DeclareLaunchArgument(
        'publish_viz',
        default_value='true',
    )

    viz_frame_id_arg = DeclareLaunchArgument(
        'viz_frame_id',
        default_value='',
    )
    viz_image_frame_id_arg = DeclareLaunchArgument(
        'viz_image_frame_id',
        default_value='',
    )

    cloud_stride_arg = DeclareLaunchArgument(
        'cloud_stride',
        default_value='4',
    )

    depth_to_color_mode_arg = DeclareLaunchArgument(
        'depth_to_color_mode',
        default_value='scale',
    )

    return LaunchDescription(
        [
            weights_arg,
            conf_arg,
            color_topic_arg,
            depth_topic_arg,
            color_info_topic_arg,
            depth_scale_arg,
            publish_viz_arg,
            viz_frame_id_arg,
            viz_image_frame_id_arg,
            cloud_stride_arg,
            depth_to_color_mode_arg,
            Node(
                package='yolo_depth_pcd',
                executable='live_pipeline_node',
                name='live_pipeline_node',
                output='screen',
                parameters=[
                    {
                        'weights': LaunchConfiguration('weights'),
                        'conf': LaunchConfiguration('conf'),
                        'color_topic': LaunchConfiguration('color_topic'),
                        'depth_topic': LaunchConfiguration('depth_topic'),
                        'color_info_topic': LaunchConfiguration('color_info_topic'),
                        'depth_scale': LaunchConfiguration('depth_scale'),
                        'publish_viz': LaunchConfiguration('publish_viz'),
                        'viz_frame_id': LaunchConfiguration('viz_frame_id'),
                        'viz_image_frame_id': LaunchConfiguration('viz_image_frame_id'),
                        'cloud_stride': LaunchConfiguration('cloud_stride'),
                        'depth_to_color_mode': LaunchConfiguration('depth_to_color_mode'),
                    }
                ],
            ),
        ]
    )
