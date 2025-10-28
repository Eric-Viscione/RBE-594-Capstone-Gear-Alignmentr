from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # --- Launch args (override on CLI if needed) ---
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera_feed/image',
        description='Image topic to subscribe to'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/camera_feed/camera_info',
        description='Camera info topic to subscribe to'
    )
    tag_size_arg = DeclareLaunchArgument(
        'tag_size', default_value='0.5',
        description='Tag edge length in meters'
    )
    camera_optical_frame_arg = DeclareLaunchArgument(
        'camera_optical_frame', default_value='camera_color_optical_frame',
        description='Camera optical frame id'
    )
    tag_frame_arg = DeclareLaunchArgument(
        'tag_frame', default_value='first_gear_apriltag',
        description='Child TF frame for the detected tag'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from simulation'
    )

    # --- Configs ---
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    tag_size = LaunchConfiguration('tag_size')
    camera_optical_frame = LaunchConfiguration('camera_optical_frame')
    tag_frame = LaunchConfiguration('tag_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')

    node = Node(
        package='sirgas_apriltag_detector',
        executable='black_tag_detector',
        name='black_tag_detector',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'camera_info_topic': camera_info_topic,
            'tag_size': tag_size,
            'camera_optical_frame': camera_optical_frame,
            'tag_frame': tag_frame,
            'use_sim_time': use_sim_time,
        }],
        # You can add remappings if your topics differ at runtime:
        # remappings=[('/camera_feed/image', '/your/image'), ('/camera_feed/camera_info', '/your/camera_info')]
    )

    return LaunchDescription([
        image_topic_arg,
        camera_info_topic_arg,
        tag_size_arg,
        camera_optical_frame_arg,
        tag_frame_arg,
        use_sim_time_arg,
        node,
    ])
