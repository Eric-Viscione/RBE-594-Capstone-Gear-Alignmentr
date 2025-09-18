import os
import shutil
import subprocess
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_name = 'sirgas_description'
    ros_distro_value = LaunchConfiguration('ros_distro').perform(context)
    
    gazebo_package = ''
    gazebo_launch_executable = ''
    spawn_executable = ''
    world_arg = {}

    if ros_distro_value == 'humble':
        gazebo_package = 'gazebo_ros'
        gazebo_launch_executable = 'gazebo.launch.py'
        spawn_executable = 'spawn_entity.py'
        world_arg = {'world': 'empty.world'}
    else:
        gazebo_package = 'ros_gz_sim'
        gazebo_launch_executable = 'gz_sim.launch.py'
        spawn_executable = 'create'
        world_arg = {'gz_args': 'empty.sdf'}

    # Use subprocess to process the xacro files
    def process_xacro(file_path):
        try:
            cmd = ['xacro', '--inorder', file_path]
            return subprocess.check_output(cmd).decode('utf-8')
        except FileNotFoundError:
            raise RuntimeError("xacro executable not found in PATH. Please install it.")
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"xacro command failed for {file_path}: {e.output.decode('utf-8')}")

    sirgas_description_dir = get_package_share_directory(pkg_name)
    
    pba_urdf_path = os.path.join(sirgas_description_dir, 'urdf', 'pba.urdf.xacro')
    ur20_starter_urdf_path = os.path.join(sirgas_description_dir, 'urdf', 'ur20_starter.urdf.xacro')
    ur20_picker_urdf_path = os.path.join(sirgas_description_dir, 'urdf', 'ur20_picker.urdf.xacro')

    pba_description_content = process_xacro(pba_urdf_path)
    ur20_starter_description_content = process_xacro(ur20_starter_urdf_path)
    ur20_picker_description_content = process_xacro(ur20_picker_urdf_path)

    gazebo_ros_dir = get_package_share_directory(gazebo_package)
    gazebo_launch_file_path = os.path.join(gazebo_ros_dir, 'launch', gazebo_launch_executable)

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file_path),
        launch_arguments=world_arg.items()
    )

    peg_board_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='peg_board_state_publisher',
        output='screen',
        parameters=[{'robot_description': pba_description_content}],
        remappings=[('/robot_description', '/peg_board_assembly/robot_description')]
    )

    spawn_peg_board = Node(
        package=gazebo_package,
        executable=spawn_executable,
        arguments=[
            '-topic', '/peg_board_assembly/robot_description', 
            '-entity', 'peg_board_assembly'
        ],
        output='screen'
    )

    ur20_starter_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ur20_starter_state_publisher',
        output='screen',
        parameters=[{'robot_description': ur20_starter_description_content}],
        remappings=[('/robot_description', '/ur20_starter/robot_description')]
    )

    spawn_ur20_starter = Node(
        package=gazebo_package,
        executable=spawn_executable,
        arguments=[
            '-topic', '/ur20_starter/robot_description',
            '-entity', 'ur20_starter',
            '-x', '-0.5',
            '-y', '0.5',
            '-z', '0.0',
        ],
        output='screen'
    )
    
    ur20_picker_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ur20_picker_state_publisher',
        output='screen',
        parameters=[{'robot_description': ur20_picker_description_content}],
        remappings=[('/robot_description', '/ur20_picker/robot_description')]
    )

    spawn_ur20_picker = Node(
        package=gazebo_package,
        executable=spawn_executable,
        arguments=[
            '-topic', '/ur20_picker/robot_description',
            '-entity', 'ur20_picker',
            '-x', '0.5',
            '-y', '-0.5',
            '-z', '0.0',
        ],
        output='screen'
    )

    return [
        gazebo_server,
        peg_board_state_publisher,
        spawn_peg_board,
        ur20_starter_state_publisher,
        spawn_ur20_starter,
        ur20_picker_state_publisher,
        spawn_ur20_picker
    ]

def generate_launch_description():
    ros_distro = DeclareLaunchArgument(
        'ros_distro',
        default_value='humble',
        description='ROS 2 distribution to use ("humble" or "jazzy").'
    )

    return LaunchDescription([
        ros_distro,
        OpaqueFunction(function=launch_setup),
    ])