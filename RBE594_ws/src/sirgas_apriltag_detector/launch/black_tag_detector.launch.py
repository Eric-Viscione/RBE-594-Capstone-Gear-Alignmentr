# minimal_tag_pose.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera_feed/image'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera_feed/camera_info'),
        DeclareLaunchArgument('tag_size', default_value='0.05'),
        DeclareLaunchArgument('camera_optical_frame', default_value='camera_color_optical_frame'),

        Node(
            package='sirgas_apriltag_detector',   # change to your package
            executable='black_tag_detector',        # entry-point name you install
            name='black_tag_detector',
            output='screen',
            parameters=[{
                'image_topic': LC('image_topic'),
                'camera_info_topic': LC('camera_info_topic'),
                'tag_size': LC('tag_size'),
                'camera_optical_frame': LC('camera_optical_frame'),
            }],
        )
    ])
