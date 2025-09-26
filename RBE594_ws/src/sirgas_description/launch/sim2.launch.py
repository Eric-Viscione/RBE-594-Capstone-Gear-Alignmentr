import os
import shutil
import subprocess
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# --- Franka Robot Setup Functions ---
def get_franka_robot_description(context, arm_id, load_gripper, franka_hand, namespace):
    """Processes the Franka URDF.xacro file to get the robot description and creates the state publisher node."""
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)
    namespace_str = context.perform_substitution(namespace)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str
        }
    )

    robot_description = {'robot_description': robot_description_config.toxml()}

    # Publish the Franka robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        namespace=namespace_str,
        parameters=[
            robot_description,
        ],
        # Publishes to the absolute topic /franka/robot_description
        remappings=[('/robot_description', '/' + namespace_str + '/robot_description')]
    )

    return [robot_state_publisher, robot_description] # Return robot_state_publisher and robot_description dict

# --- Peg Board Assembly Setup Functions ---
def get_pba_description(context, ros_distro, pba_package_name):
    """Processes the Peg Board Assembly (PBA) URDF.xacro and creates the state publisher node."""
    ros_distro_value = context.perform_substitution(ros_distro)
    pba_pkg_name = context.perform_substitution(pba_package_name)
    
    sirgas_description_dir = get_package_share_directory(pba_pkg_name)

    peg_board_urdf_path = os.path.join(
        sirgas_description_dir,
        'urdf',
        'pba.urdf.xacro'
    )
    
    xacro_executable = shutil.which('xacro')
    if xacro_executable is None:
        raise RuntimeError("xacro executable not found in PATH. Please install it.")
        
    try:
        peg_board_description_content = subprocess.check_output(
            [xacro_executable, '--inorder', peg_board_urdf_path],
            universal_newlines=True
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Failed to process xacro file: {e}")

    # Publish the Peg Board Assembly state
    peg_board_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='peg_board_state_publisher',
        output='screen',
        parameters=[{'robot_description': peg_board_description_content}],
        # Remap to a unique absolute topic for spawning the PBA
        remappings=[('/robot_description', '/peg_board_assembly/robot_description')]
    )

    # Spawn the peg board entity in Gazebo Sim
    spawn_peg_board = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/peg_board_assembly/robot_description', 
            '-name', 'peg_board_assembly',
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.0'
        ],
        output='screen'
    )
    
    return [
        peg_board_state_publisher,
        spawn_peg_board,
    ]

# --- Opaque Function Wrapper for Franka Nodes ---
def setup_franka_robot(context):
    arm_id = LaunchConfiguration('franka_arm_id')
    load_gripper = LaunchConfiguration('load_gripper')
    franka_hand = LaunchConfiguration('franka_hand')
    namespace = LaunchConfiguration('franka_namespace')
    
    result = get_franka_robot_description(context, arm_id, load_gripper, franka_hand, namespace)
    
    # Unpack the results: actions are the first elements, robot_description dict is the last.
    franka_robot_state_publisher = result[0]
    franka_robot_description = result[1]

    # ✅ FIXED: Use the absolute path provided by the user
    controllers_yaml_path = '/home/tamar/RBE594_ws/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml'
    
    # Node 1: Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=context.perform_substitution(namespace),
        parameters=[
            franka_robot_description, 
            controllers_yaml_path
        ],
        output='both',
        remappings=[('/controller_manager/robot_description', PathJoinSubstitution(['/', context.perform_substitution(namespace), 'robot_description']))],
        arguments=['--ros-args', '--log-level', 'warn'] 
    )

    # Node 2: Spawn Franka
    spawn_franka = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=context.perform_substitution(namespace),
        arguments=[
            '-topic', PathJoinSubstitution(['/', context.perform_substitution(namespace), 'robot_description']), 
            '-name', 'franka'
        ],
        output='screen',
    )
    
    # Node 3: Joint State Publisher (for non-controlled parts)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=context.perform_substitution(namespace),
        parameters=[
            {'source_list': ['joint_states'], 
             'rate': 30}],
    )

    # Node 4: RViz
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             namespace=context.perform_substitution(namespace),
             arguments=['--display-config', rviz_file, '-f', 'world'],
    )

    # Node 5: Load joint state broadcaster controller
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster', '--controller-manager', PathJoinSubstitution(['/', context.perform_substitution(namespace), 'controller_manager'])],
        output='screen'
    )
    
    return [
        franka_robot_state_publisher,
        controller_manager,
        spawn_franka,
        joint_state_publisher,
        rviz,
        # Load controller only after the robot has been spawned (which sets up gz_ros2_control)
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_franka,
                    on_exit=[load_joint_state_broadcaster],
                )
        ),
    ]

# --- Main Launch Description ---
def generate_launch_description():
    # 1. Launch Arguments
    load_gripper_launch_argument = DeclareLaunchArgument(
            'load_gripper', default_value='false', description='true/false for activating the Franka gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
            'franka_hand', default_value='franka_hand', description='Franka Hand ID')
    arm_id_launch_argument = DeclareLaunchArgument(
            'franka_arm_id', default_value='fr3', description='Available Franka values: fr3, fp3 and fer')
    namespace_launch_argument = DeclareLaunchArgument(
        'franka_namespace', default_value='franka', description='Namespace for the Franka robot.')
    ros_distro_arg = DeclareLaunchArgument(
        'ros_distro', default_value='jazzy', description='ROS 2 distribution to use (e.g., "humble" or "jazzy") - relevant for PBA xacro processing')
    pba_package_name_arg = DeclareLaunchArgument(
        'pba_package_name', default_value='sirgas_description', description='Name of the package containing the PBA URDF file')


    # 2. Gazebo Sim Setup
    # Ensure Franka resources are found by Gazebo
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    )

    # 3. Franka Robot Nodes (RSP, Controller Manager, Spawner, RViz)
    franka_all_nodes = OpaqueFunction(function=setup_franka_robot)

    # 4. Peg Board Assembly State Publisher and Spawner
    pba_nodes = OpaqueFunction(
        function=get_pba_description,
        args=[LaunchConfiguration('ros_distro'), LaunchConfiguration('pba_package_name')])
        

    return LaunchDescription([
        # Arguments
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        namespace_launch_argument,
        ros_distro_arg,
        pba_package_name_arg,

        # Simulation Backend
        gazebo_empty_world,

        # Franka and PBA Nodes
        franka_all_nodes,
        pba_nodes,
    ])