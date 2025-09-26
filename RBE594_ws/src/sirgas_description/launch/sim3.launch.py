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
PACKAGE_NAME = 'sirgas_description'

# Robot 1: Peg Board Assembly (pba.urdf.xacro)
PBA_XACRO_FILE = 'pba.urdf.xacro'
PBA_MODEL_NAME = 'peg_board_assembly'
PBA_TOPIC = '/robot_description_pba'

# Robot 2: Gear Starter Motor (gear_starter_motor.urdf.xacro)
GSM_XACRO_FILE = 'gear_starter_motor.urdf.xacro'
GSM_MODEL_NAME = 'gear_starter_motor'
GSM_TOPIC = '/robot_description_gsm'


# --- Helper Function: Load and Process Xacro ---
def load_xacro_to_urdf(file_name):
    """
    Finds a specific Xacro file, processes it, and returns the URDF XML string.
    """
    try:
        share_dir = get_package_share_directory(PACKAGE_NAME)
        xacro_file_path = os.path.join(share_dir, 'urdf', file_name)
    except Exception as e:
        print(f"ERROR: Could not find package '{PACKAGE_NAME}'. Ensure it is built and installed. Error: {e}")
        # Fallback for source-space testing (less reliable)
        xacro_file_path = os.path.join(os.getcwd(), '..', PACKAGE_NAME, 'urdf', file_name)
        
    print(f"Loading Xacro file from: {xacro_file_path}")

    # Process the Xacro file
    robot_description_config = xacro.process_file(xacro_file_path)
    return robot_description_config.toxml()

# --- OpaqueFunction to determine the robot descriptions at runtime ---
def set_robot_descriptions(context):
    """
    Loads both Xacro files and sets the default values for the launch arguments.
    """
    pba_desc = load_xacro_to_urdf(PBA_XACRO_FILE)
    gsm_desc = load_xacro_to_urdf(GSM_XACRO_FILE)
    
    return [
        DeclareLaunchArgument(
            'pba_description',
            default_value=pba_desc,
            description='The URDF content for the Peg Board Assembly.',
        ),
        DeclareLaunchArgument(
            'gsm_description',
            default_value=gsm_desc,
            description='The URDF content for the Gear Starter Motor.',
        )
    ]

# --- Main launch function ---
def generate_launch_description():
    
    # 1. Launch Arguments for Configuration
    
    # Generic Arguments
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
    
    # Arguments for PBA Pose (Peg Board Assembly)
    pba_x_pos_arg = DeclareLaunchArgument(
        'pba_x_pos', default_value='0.0', description='X spawn position for Peg Board Assembly.'
    )
    pba_y_pos_arg = DeclareLaunchArgument(
        'pba_y_pos', default_value='0.0', description='Y spawn position for Peg Board Assembly.'
    )
    pba_z_pos_arg = DeclareLaunchArgument(
        'pba_z_pos', default_value='0.0', description='Z spawn position for Peg Board Assembly.'
    )

    # Arguments for GSM Pose (Gear Starter Motor)
    gsm_x_pos_arg = DeclareLaunchArgument(
        'gsm_x_pos', default_value='-0.665', description='X spawn position for Gear Starter Motor.'
    )
    gsm_y_pos_arg = DeclareLaunchArgument(
        'gsm_y_pos', default_value='0.0', description='Y spawn position for Gear Starter Motor.'
    )
    gsm_z_pos_arg = DeclareLaunchArgument(
        'gsm_z_pos', default_value='0.0', description='Z spawn position for Gear Starter Motor.'
    )
    
    # 2. Xacro to URDF conversion and loading
    set_robot_desc_action = OpaqueFunction(function=set_robot_descriptions)
    pba_description = LaunchConfiguration('pba_description')
    gsm_description = LaunchConfiguration('gsm_description')

    # 3. Nodes and Actions

    # 3.1. Gazebo Launch (Starts the simulator)
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
    
    # 3.2. Peg Board Assembly (PBA) Nodes
    
    # PBA Robot State Publisher
    pba_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='pba_robot_state_publisher',
        output='screen',
        namespace=PBA_MODEL_NAME,
        parameters=[
            {'robot_description': pba_description},
            {'frame_prefix': PBA_MODEL_NAME + '/'},
        ],
        remappings=[
            ('robot_description', PBA_TOPIC)
        ]
    )

    # PBA Spawn Entity (using position arguments)
    pba_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', PBA_TOPIC,
            '-entity', PBA_MODEL_NAME,
            '-x', LaunchConfiguration('pba_x_pos'), # Use launch argument
            '-y', LaunchConfiguration('pba_y_pos'), # Use launch argument
            '-z', LaunchConfiguration('pba_z_pos'), # Use launch argument
        ],
        output='screen',
    )
    
    # 3.3. Gear Starter Motor (GSM) Nodes

    # GSM Robot State Publisher
    gsm_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='gsm_robot_state_publisher',
        output='screen',
        namespace=GSM_MODEL_NAME,
        parameters=[
            {'robot_description': gsm_description},
            {'frame_prefix': GSM_MODEL_NAME + '/'},
        ],
        remappings=[
            ('robot_description', GSM_TOPIC)
        ]
    )
    
    # GSM Spawn Entity (using position arguments)
    gsm_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', GSM_TOPIC,
            '-entity', GSM_MODEL_NAME,
            '-x', LaunchConfiguration('gsm_x_pos'), # Use launch argument
            '-y', LaunchConfiguration('gsm_y_pos'), # Use launch argument
            '-z', LaunchConfiguration('gsm_z_pos'), # Use launch argument
        ],
        output='screen',
    )

    # 3.4. RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        # Pose and Configuration Arguments
        ros_dist_arg,
        use_rviz_arg,
        pba_x_pos_arg, pba_y_pos_arg, pba_z_pos_arg,
        gsm_x_pos_arg, gsm_y_pos_arg, gsm_z_pos_arg,

        # Set Robot Descriptions (Xacro conversion for both)
        set_robot_desc_action,
        
        # Launch Gazebo
        gazebo,
        
        # Launch Robot Nodes
        pba_rsp_node,
        pba_spawn_node,
        gsm_rsp_node,
        gsm_spawn_node,
        
        # Launch RViz
        #rviz_node,
    ])