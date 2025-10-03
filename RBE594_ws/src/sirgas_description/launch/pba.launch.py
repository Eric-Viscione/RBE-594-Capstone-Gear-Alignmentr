import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node

def generate_launch_description():
    # --- Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # --- Paths ---
    pkg_sirgas_description = get_package_share_directory('sirgas_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    urdf_path = PathJoinSubstitution(
        [pkg_sirgas_description, 'urdf', 'pba.urdf.xacro']
    )
    
    controller_config_path = PathJoinSubstitution(
        [pkg_sirgas_description, 'config', 'pba_controller.yaml']
    )

    # Robot Description command for xacro
    robot_description_content = Command(['xacro ', urdf_path])
    
    # --- Actions (Defined bottom-up for correct TimerAction chaining) ---

    # 1. Spawn velocity controller (final step in the chain)
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_velocity_controller'],
        output='screen',
    )
    
    # Wait for joint state broadcaster (2.0s delay after joint state broadcaster is spawned)
    # Original delay was 10.0s, which is 2.0s after the 8.0s CM start.
    timer_wait_for_jsb = TimerAction(
        period=2.0, 
        actions=[velocity_controller_spawner]
    )

    # 2. Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    # Wait for controller manager to start (3.0s delay after controller manager starts)
    # Original delay was 8.0s, which is 3.0s after the 5.0s robot spawn.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_config_path, 
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_content}
        ],
        output='screen',
        respawn=True
    )
    timer_wait_for_controller_manager = TimerAction(
        period=3.0, 
        actions=[
            joint_state_broadcaster_spawner,
            timer_wait_for_jsb
        ]
    )

    # 3. Load controllers (ros2_control_node)
    
    # Wait for robot to spawn (2.0s delay after robot is spawned)
    # Original delay was 5.0s, which is 2.0s after the 3.0s Gazebo start.
    timer_wait_for_spawn = TimerAction(
        period=2.0, 
        actions=[
            ros2_control_node,
            timer_wait_for_controller_manager
        ]
    )
    
    # 4. Robot State Publisher & Spawning
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }],
        output='screen'
    )
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'peg_board_assembly', 
            '-z', '0.0'
        ],
        output='screen'
    )
    
    # Wait for Gazebo to start (3.0s delay after Gazebo launch)
    timer_wait_for_gazebo = TimerAction(
        period=3.0, 
        actions=[
            robot_state_publisher_node,
            spawn_robot_node,
            timer_wait_for_spawn
        ]
    )
    
    # 5. Launch Ignition Gazebo
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'use_sim_time': use_sim_time
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        
        # Start Gazebo, then wait 3.0s and execute the actions in timer_wait_for_gazebo
        gz_sim_launch,
        timer_wait_for_gazebo,
    ])