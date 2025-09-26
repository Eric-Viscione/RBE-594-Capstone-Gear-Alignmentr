import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

# --- Configuration Constants ---
# Assuming the package containing the Xacro file is 'sirgas_description' based on the path
PACKAGE_NAME = 'sirgas_description'
XACRO_FILE_NAME = 'gear_starter_motor.urdf.xacro'

# --- Function to load the Xacro file and convert to URDF string ---
def load_robot_description(context):
    """
    Finds the Xacro file and processes it into a URDF XML string.
    """
    # Use the specified path structure: /home/tamar/RBE594_ws/src/sirgas_description/urdf/gear_starter_motor.urdf.xacro
    # For a correctly installed package, get_package_share_directory is the standard way.
    try:
        share_dir = get_package_share_directory(PACKAGE_NAME)
        # The provided path suggests the file is directly under 'urdf' in the source directory.
        # In a typical install, it would be 'share/sirgas_description/urdf/...'
        # We will use the canonical ROS 2 way, assuming the urdf directory is exported in the install.
        xacro_file_path = os.path.join(share_dir, 'urdf', XACRO_FILE_NAME)
    except Exception as e:
        # Fallback/Debug for when the package isn't installed but is run from the source space.
        # For a user-specific path like the one provided, it's safer to use the canonical ROS way
        # and assume the user will run the build/install step.
        print(f"Warning: Could not find package '{PACKAGE_NAME}'. Ensure it is built and installed. Error: {e}")
        # Placeholder path - WILL ONLY WORK if launch file is run from the *root* of the workspace
        # and sirgas_description is a sibling package. This is a bad practice but provided for context.
        xacro_file_path = os.path.join(os.getcwd(), '..', PACKAGE_NAME, 'urdf', XACRO_FILE_NAME)
        
    print(f"Attempting to load Xacro file from: {xacro_file_path}")

    # Process the Xacro file
    robot_description_config = xacro.process_file(xacro_file_path)
    robot_desc = robot_description_config.toxml()
    
    return {'robot_description': robot_desc}

# --- OpaqueFunction to determine the robot description at runtime ---
def set_robot_description(context):
    """
    Declares the 'robot_description' launch argument with the processed URDF content.
    """
    return [
        DeclareLaunchArgument(
            'robot_description',
            default_value=load_robot_description(context)['robot_description'],
            description='The URDF content',
        )
    ]

# --- Main launch function ---
def generate_launch_description():
    
    # 1. Launch Arguments
    
    # Specify ROS distribution (not strictly needed for these nodes, but kept for context)
    ros_dist_arg = DeclareLaunchArgument(
        'ros_dist',
        default_value='humble',
        description="ROS distribution: 'humble' or 'jazzy'.",
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2',
    )
    
    # 2. Xacro to URDF conversion and loading
    set_robot_desc_action = OpaqueFunction(function=set_robot_description)
    robot_description = LaunchConfiguration('robot_description')

    # 3. Nodes and Actions

    # 3.1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # 3.2. Gazebo Launch (Starts the simulator)
    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                gazebo_ros_share_dir, 
                'launch', 
                'gazebo.launch.py'
            ])
        )
    )
    
    # 3.3. Spawn Entity Node (Loads the model into Gazebo)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'gear_starter_motor',
        ],
        output='screen',
    )
    
    # 3.4. RViz Node
    # A default RViz config file is recommended for a real-world package.
    # rviz_config_file = PathJoinSubstitution([
    #     get_package_share_directory(PACKAGE_NAME), 
    #     'rviz', 
    #     'view_robot.rviz'
    # ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', rviz_config_file], # Uncomment and provide path to custom config
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        # Launch Arguments
        ros_dist_arg,
        use_rviz_arg,

        # Set Robot Description (Xacro conversion)
        set_robot_desc_action,
        
        # Launch Gazebo
        gazebo,
        
        # Nodes
        robot_state_publisher_node,
        spawn_entity_node,
        rviz_node,
    ])