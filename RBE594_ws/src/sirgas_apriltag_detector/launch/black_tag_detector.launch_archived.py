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
        'tag_size', default_value='0.05',
        description='Tag edge length in meters'
    )
    camera_optical_frame_arg = DeclareLaunchArgument(
        'camera_optical_frame', default_value='camera_color_optical_frame',
        description='Camera optical frame id'
    )
    tag_frame_arg = DeclareLaunchArgument(
        'tag_frame', default_value='first_gear_apriltag_detected',
        description='Child TF frame for the detected tag'
    )
    world_frame_arg = DeclareLaunchArgument(
        'world_frame', default_value='world',
        description='World/base frame used for composing the tag pose'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from simulation'
    )

    # NEW: logging controls
    save_log_images_arg = DeclareLaunchArgument(
        'save_log_images', default_value='false',
        description='If true, save the first N frames to disk'
    )
    log_image_count_arg = DeclareLaunchArgument(
        'log_image_count', default_value='5',
        description='How many frames to save when logging is enabled'
    )
    debug_dir_arg = DeclareLaunchArgument(
        'debug_dir', default_value='/tmp',
        description='Directory to write logged images'
    )

    # --- Configs ---
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    tag_size = LaunchConfiguration('tag_size')
    camera_optical_frame = LaunchConfiguration('camera_optical_frame')
    tag_frame = LaunchConfiguration('tag_frame')
    world_frame = LaunchConfiguration('world_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    save_log_images = LaunchConfiguration('save_log_images')
    log_image_count = LaunchConfiguration('log_image_count')
    debug_dir = LaunchConfiguration('debug_dir')

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
            # set BOTH, in case the node uses one or the other
            'tag_frame': tag_frame,
            'detected_tag_frame': tag_frame,
            'world_frame': world_frame,
            'use_sim_time': use_sim_time,

            # NEW logging params
            'save_log_images': save_log_images,
            'log_image_count': log_image_count,
            'debug_dir': debug_dir,
        }],
    )

    return LaunchDescription([
        image_topic_arg,
        camera_info_topic_arg,
        tag_size_arg,
        camera_optical_frame_arg,
        tag_frame_arg,
        world_frame_arg,
        use_sim_time_arg,
        save_log_images_arg,
        log_image_count_arg,
        debug_dir_arg,
        node,
    ])
