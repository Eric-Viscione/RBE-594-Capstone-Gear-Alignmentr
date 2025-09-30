import os
import shutil
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    ros_distro_value = LaunchConfiguration('ros_distro').perform(context)
    
    # Determine the Gazebo package and executable based on the ROS distribution
    gazebo_package = ''
    gazebo_launch_executable = ''
    spawn_executable = ''
    spawn_arguments = []
    world_arg = {}
    
    # Get package share directory paths
    sirgas_description_dir = get_package_share_directory('sirgas_description')

    if ros_distro_value == 'humble':
        gazebo_package = 'gazebo_ros'
        gazebo_launch_executable = 'gazebo.launch.py'
        spawn_executable = 'spawn_entity.py'
        spawn_arguments = ['-topic', 'robot_description', '-entity', 'peg_board_assembly']
        world_arg = {'world': 'empty.world'}
    else: # Default to Jazzy
        gazebo_package = 'ros_gz_sim'
        gazebo_launch_executable = 'gz_sim.launch.py'
        spawn_executable = 'create'
        spawn_arguments = ['-topic', 'robot_description', '-name', 'peg_board_assembly']
        world_arg = {'gz_args': 'empty.sdf'}

    # 1. FIND CONTROLLER CONFIGURATION FILE 
    controller_config_file = os.path.join(
        sirgas_description_dir, 
        'config',
        'pba_controller.yaml'
    )

    # Path to the peg board URDF
    peg_board_urdf_path = os.path.join(
        sirgas_description_dir,
        'urdf',
        'pba.urdf.xacro'
    )
    
    # Find xacro executable and process the file directly
    xacro_executable = shutil.which('xacro')
    if xacro_executable is None:
        raise RuntimeError("xacro executable not found in PATH. Please install it.")
        
    try:
        # Run xacro as a subprocess to get the processed URDF as a string
        peg_board_description_content = subprocess.check_output(
            [xacro_executable, '--inorder', peg_board_urdf_path],
            universal_newlines=True
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Failed to process xacro file: {e}")

    # Gazebo launch file
    gazebo_ros_dir = get_package_share_directory(gazebo_package)
    gazebo_launch_file_path = os.path.join(gazebo_ros_dir, 'launch', gazebo_launch_executable)

    # Start Gazebo with an empty world
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file_path),
        launch_arguments=world_arg.items()
    )

    # Publish the robot state
    peg_board_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': peg_board_description_content}],
        remappings=[('/robot_description', '/robot_description')]
    )

    # Spawn the peg board entity in Gazebo
    spawn_peg_board = Node(
        package=gazebo_package,
        executable=spawn_executable,
        arguments=spawn_arguments,
        output='screen'
    )
    
    # 2. CONTROLLER SPAWNER NODE DEFINITION (FIXED ARGUMENTS FOR HUMBLE)
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'pb_starter_joint_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            controller_config_file, 
            # Use the Humble-compatible argument for service timeout:
            '--controller-manager-timeout', '30.0', # Set timeout to 30 seconds
        ],
        output='screen',
    )
    
    # 3. ADD EVENT HANDLER
    # Start the spawner ONLY after the robot entity has been spawned into Gazebo.
    delay_controller_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_peg_board,
            on_exit=[controller_spawner],
        )
    )
    
    return [
        gazebo_server,
        peg_board_state_publisher,
        spawn_peg_board,
        delay_controller_spawner, 
    ]


def generate_launch_description():
    # Declare the ros_distro argument
    ros_distro = DeclareLaunchArgument(
        'ros_distro',
        default_value='jazzy',
        description='ROS 2 distribution to use (e.g., "humble" or "jazzy")'
    )

    return LaunchDescription([
        ros_distro,
        OpaqueFunction(function=launch_setup),
    ])