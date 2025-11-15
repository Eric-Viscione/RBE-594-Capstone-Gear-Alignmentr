# tag_processing.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # --- Substitutions ---
    pkg_name = 'sirgas_apriltag_detector'
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # --- 1. Green Tag Detector Node ---
    green_detector_node = Node(
        package=pkg_name, 
        executable='black_tag_detector', # Uses the black_tag_detector logic
        name='green_tag_detector',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'t_dark': 20}, # Assuming green uses a dark threshold or specific config
            {'color_mode': 'hsv'}, # Assuming specific color mode for green
            {'hsv_low': [50, 100, 100]},
            {'hsv_high': [70, 255, 255]},
            
            # Topics
            {'pose_topic_cam': '/tag_pose_cam/green'},
            {'pose_topic_world': '/tag_pose_world/green'},
            {'long_axis_cam_topic': '/tag_long_axis_cam/green'},
            {'long_axis_world_topic': '/tag_long_axis_world/green'},
        ]
    )
    

    # --- 2. Black Tag Detector Node ---
    black_detector_node = Node(
        package=pkg_name, 
        executable='black_tag_detector',
        name='black_tag_detector',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'color_mode': 'dark'},
            {'t_dark': 10},
            
            # Topics
            {'pose_topic_cam': '/tag_pose_cam/black'},
            {'pose_topic_world': '/tag_pose_world/black'},
            {'long_axis_cam_topic': '/tag_long_axis_cam/black'},
            {'long_axis_world_topic': '/tag_long_axis_world/black'},
        ]
    )
    
    # --- 3. Axis Comparator Node ---
    axis_comparator_node = Node(
        package=pkg_name,
        executable='tag_axis_comparator',
        name='axis_comparator',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            # Must match the topics defined above
            {'green_axis_topic': '/tag_long_axis_world/green'},
            {'black_axis_topic': '/tag_long_axis_world/black'},
            {'output_topic': '/tag_axis_difference'}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        green_detector_node,
        black_detector_node,
        axis_comparator_node,
    ])